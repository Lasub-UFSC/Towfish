#include <Wire.h>
// #include <avr/wdt.h>
// #include "DHT.h"

// #define DHTPIN 7
// #define DHTTYPE DHT11
// DHT dht(DHTPIN, DHTTYPE);

#include <ModbusRTUSlave.h>
#define MODBUS_SERIAL Serial2
#define RXD2 16
#define TXD2 17
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1
const int16_t dePin = 4;
ModbusRTUSlave modbus(MODBUS_SERIAL, dePin);

const uint8_t numCoils = 0;
const uint8_t numDiscreteInputs = 0;
const uint8_t numHoldingRegisters = 0;
const uint8_t numInputRegisters = 10;

bool coils[numCoils];
bool discreteInputs[numDiscreteInputs];
uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

#define pressure_sensor 3
#define I2C_SDA 33
#define I2C_SCL 32

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[] = { 0, 0 };


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
  Serial.println("ESP32 Server Started.");

  // dht.begin();
  // IMU
  Wire.setClock(400000);
  // Wire.begin();
  Wire.begin(I2C_SDA,I2C_SCL);
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

  modbus.configureCoils(coils, numCoils);
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);

  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG,RXD2,TXD2);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
}


void loop() {
  delay(1000);
  gyro_signals();
  inputRegisters[0]= (uint16_t)RateRoll;
  inputRegisters[1]= (uint16_t)RatePitch;
  inputRegisters[2]= (uint16_t)RateYaw;
  String RateGyro = "RateRoll: " + (String)RateRoll + " RatePitch: " + (String)RatePitch + " RateYaw: " + (String)RateYaw;
  Serial.println(RateGyro);
  // // if(rs485.sendMessage(RateGyro)){
  // //   lastCommunicationTime = millis();
  // // }

  inputRegisters[3]= (uint16_t)AccX;
  inputRegisters[4]= (uint16_t)AccY;
  inputRegisters[5]= (uint16_t)AccZ;
  // // String AccGyro = "AccX: " + (String)AccX + " AccY: " + (String)AccY + " AccZ: " + (String)AccZ;
  // // if(rs485.sendMessage(AccGyro)){
  // //   lastCommunicationTime = millis();
  // // }

  inputRegisters[6]= (uint16_t)AngleRoll;
  inputRegisters[7]= (uint16_t)AnglePitch;
  // // String Roll_Pitch = "AngleRoll: " + (String)AngleRoll + " AnglePitch: " + (String)AnglePitch;
  // // if(rs485.sendMessage(Roll_Pitch)){
  // //   lastCommunicationTime = millis();
  // // }
  // int h = dht.readHumidity();
  // inputRegisters[8]= (uint16_t)h;
  // // String Humidity = "Humidity: " + (String) h;
  // // if(rs485.sendMessage(Humidity)){
  // //   lastCommunicationTime = millis();
  // // }

  // float p = analogRead(pressure_sensor);
  // inputRegisters[9]= (uint16_t)p;
  // // String pressure = "Pressure: " + (String)p;
  // // if(rs485.sendMessage(pressure)){
  // //   lastCommunicationTime = millis();
  // // }

  modbus.poll();
}