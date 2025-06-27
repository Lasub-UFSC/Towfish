#include <Wire.h>
#include <SoftwareSerial.h>
#include <ModbusRTUSlave.h>

#define pressure_sensor A3

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[] = { 0, 0 };

const int8_t rxPin = 2;
const int8_t txPin = 3;
SoftwareSerial mySerial(rxPin, txPin);
#define MODBUS_SERIAL mySerial
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1
const int8_t dePin = 4;

ModbusRTUSlave modbus(MODBUS_SERIAL, dePin);

const uint8_t numCoils = 0;
const uint8_t numDiscreteInputs = 0;
const uint8_t numHoldingRegisters = 0;
const uint8_t numInputRegisters = 9;

bool coils[numCoils];
bool discreteInputs[numDiscreteInputs];
int16_t holdingRegisters[numHoldingRegisters];
int16_t inputRegisters[numInputRegisters];

const float KalmanGain = 0.000256; // 0.004 * 0.004 * 4 * 4

//Versão Simplificado do Filtro de Kalmann Unidimensional 
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  
  KalmanUncertainty = KalmanUncertainty + KalmanGain; //Esses ganhos podem ser adaptados dependendo do fabricante
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 9); //Esses ganhos podem ser adaptados dependendo do fabricante
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  //Serial.println(GyroX);
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.05;//Calibração manual -> Valores ajustados por teste    |
  AccY=(float)AccYLSB/4096-0.03;//Calibração manual -> Valores ajustados por teste    |-> aqui da pra fazer uma manipulacao bit a bit maluca
  AccZ=(float)AccZLSB/4096-0.41;//Calibração manual -> Valores ajustados por teste    |
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(0.01746);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(0.01746);
}

void setup() {
  Serial.begin(115200);
  Serial.println("RS485 Slave started.");
  
  // IMU
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;    //
  RateCalibrationPitch/=2000;   // -> tem problema dividir por 2048? Ai da pra fazer manipulação bit a bit aqui tbm
  RateCalibrationYaw/=2000;     //
  Serial.println("Modbus config...");

  modbus.configureCoils(coils, numCoils);
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);

  MODBUS_SERIAL.begin(MODBUS_BAUD);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
  Serial.println("Setup finished");
}


void loop() {
    gyro_signals();
    String RateGyro = "RateRoll: " + (String)RateRoll + " RatePitch: " + (String)RatePitch + " RateYaw: " + (String)RateYaw;
    Serial.println(RateGyro);
    inputRegisters[0]= 30000+100*RateRoll;
    Serial.println(inputRegisters[0]);
    inputRegisters[1]= 30000+100*RatePitch;
    inputRegisters[2]= 30000+100*RateYaw;

    
    String AccGyro = "AccX: " + (String)AccX + " AccY: " + (String)AccY + " AccZ: " + (String)AccZ; 
    inputRegisters[3]= 30000+100*AccX;
    inputRegisters[4]= 30000+100*AccY;
    inputRegisters[5]= 30000+100*AccZ;

    String Roll_Pitch = "AngleRoll: " + (String)AngleRoll + " AnglePitch: " + (String)AnglePitch; 
    inputRegisters[6]= 30000+100*AngleRoll;
    inputRegisters[7]= 30000+100*AnglePitch;

    float p = analogRead(pressure_sensor);
    String pressure = "Pressure: " + (String)p;
    inputRegisters[8]= 30000+100*p;
    modbus.poll();
}