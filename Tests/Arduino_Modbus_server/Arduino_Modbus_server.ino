#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <ModbusRTUSlave.h>

// #include <avr/wdt.h>

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
const uint8_t numInputRegisters = 14;

bool coils[numCoils];
bool discreteInputs[numDiscreteInputs];
uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

#define pressure_sensor 3
#define I2C_SDA 33
#define I2C_SCL 32

// float RateRoll, RatePitch, RateYaw;
// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch;
// float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
// int RateCalibrationNumber;
Adafruit_MPU6050 mpu;
// float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
// float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
// float Kalman1DOutput[] = { 0, 0 };


// const float KalmanGain = 0.000256; // 0.004 * 0.004 * 4 * 4

// //Versão Simplificado do Filtro de Kalmann Unidimensional
// void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
//   KalmanState = KalmanState + 0.004 * KalmanInput;

//   KalmanUncertainty = KalmanUncertainty + KalmanGain; //Esses ganhos podem ser adaptados dependendo do fabricante
//   float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 9); //Esses ganhos podem ser adaptados dependendo do fabricante
//   KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
//   KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
//   Kalman1DOutput[0] = KalmanState;
//   Kalman1DOutput[1] = KalmanUncertainty;
// }

// void gyro_signals(void) {
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1A);
//   Wire.write(0x05);
//   Wire.endTransmission();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1C);
//   Wire.write(0x10);
//   Wire.endTransmission();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x3B);
//   Wire.endTransmission();
//   Wire.requestFrom(0x68,6);
//   int16_t AccXLSB = Wire.read() << 8 | Wire.read();
//   int16_t AccYLSB = Wire.read() << 8 | Wire.read();
//   int16_t AccZLSB = Wire.read() << 8 | Wire.read();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1B);
//   Wire.write(0x8);
//   Wire.endTransmission();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x43);
//   Wire.endTransmission();
//   Wire.requestFrom(0x68,6);
//   int16_t GyroX=Wire.read()<<8 | Wire.read();
//   int16_t GyroY=Wire.read()<<8 | Wire.read();
//   int16_t GyroZ=Wire.read()<<8 | Wire.read();
//   //Serial.println(GyroX);
//   RateRoll=(float)GyroX/65.5;
//   RatePitch=(float)GyroY/65.5;
//   RateYaw=(float)GyroZ/65.5;
//   AccX=(float)AccXLSB/4096-0.05;//Calibração manual -> Valores ajustados por teste    |
//   AccY=(float)AccYLSB/4096-0.03;//Calibração manual -> Valores ajustados por teste    |-> aqui da pra fazer uma manipulacao bit a bit maluca
//   AccZ=(float)AccZLSB/4096-0.41;//Calibração manual -> Valores ajustados por teste    |
//   AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(0.01746);
//   AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(0.01746);
// }

enum ModbusRegister {
  Timestamp = 0,
  AccX = 2,
  AccY = 4,
  AccZ = 6,
  GyroX = 8,
  GyroY = 10,
  GyroZ = 12
};
void update_input_register(ModbusRegister var, float value){
  uint8_t* floatBytes = (uint8_t*)&value;

  inputRegisters[var] = ((uint16_t)floatBytes[1] << 8) | floatBytes[0];
  inputRegisters[var+1] = ((uint16_t)floatBytes[3] << 8) | floatBytes[2];
}

void update_input_register(ModbusRegister var, unsigned long value){
  uint8_t* valueBytes = (uint8_t*)&value;

  inputRegisters[var] = ((uint16_t)valueBytes[1] << 8) | valueBytes[0];
  inputRegisters[var+1] = ((uint16_t)valueBytes[3] << 8) | valueBytes[2];
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Server Started.");

  // IMU
  Wire.setClock(400000);
  Wire.begin();
    // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
    //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  // delay(250);
  // Wire.beginTransmission(0x68);
  // Wire.write(0x6B);
  // Wire.write(0x00);
  // Wire.endTransmission();
  // for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
  //   gyro_signals();
  //   RateCalibrationRoll+=RateRoll;
  //   RateCalibrationPitch+=RatePitch;
  //   RateCalibrationYaw+=RateYaw;
  //   delay(1);
  // }
  // RateCalibrationRoll/=2000;    //
  // RateCalibrationPitch/=2000;   // -> tem problema dividir por 2048? Ai da pra fazer manipulação bit a bit aqui tbm
  // RateCalibrationYaw/=2000;     //

  modbus.configureCoils(coils, numCoils);
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);

  MODBUS_SERIAL.begin(MODBUS_BAUD);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
}


void loop() {
  update_input_register(Timestamp, millis());
  // if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    update_input_register(AccX, a.acceleration.x);
    update_input_register(AccY, a.acceleration.y);
    update_input_register(AccZ, a.acceleration.z);
    update_input_register(GyroX, g.gyro.x);
    update_input_register(GyroY, g.gyro.y);
    update_input_register(GyroZ, g.gyro.z);
  // }
  modbus.poll();
}