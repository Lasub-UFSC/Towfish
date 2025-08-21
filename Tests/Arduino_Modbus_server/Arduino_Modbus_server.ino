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

bool imu_error=false;
Adafruit_MPU6050 mpu;

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

void start_mpu(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    imu_error = true;
    update_input_register(AccX, (float)-10);
    update_input_register(AccY, (float)-10);
    update_input_register(AccZ, (float)-10);
    update_input_register(GyroX, (float)-10);
    update_input_register(GyroY, (float)-10);
    update_input_register(GyroZ, (float)-10);

    delay(1000);
  } else{
    Serial.println("MPU6050 Found!");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Server Started.");

  // IMU
  Wire.setClock(400000);
  Wire.begin();
  // Try to initialize!
  start_mpu();
  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  modbus.configureCoils(coils, numCoils);
  modbus.configureDiscreteInputs(discreteInputs, numDiscreteInputs);
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  modbus.configureInputRegisters(inputRegisters, numInputRegisters);

  MODBUS_SERIAL.begin(MODBUS_BAUD);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);

  // wdt_enable(WDTO_2S);
}


void loop() {
  Serial.println("Updating timestamp...");
  update_input_register(Timestamp, millis());

  if (!imu_error) {
    Serial.println("Reading IMU data...");
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    update_input_register(AccX, a.acceleration.x);
    update_input_register(AccY, a.acceleration.y);
    update_input_register(AccZ, a.acceleration.z);
    update_input_register(GyroX, g.gyro.x);
    update_input_register(GyroY, g.gyro.y);
    update_input_register(GyroZ, g.gyro.z); 
    Serial.println("IMU data updated.");
  } else {
    Serial.println("IMU error detected. Trying to restart MPU...");
    start_mpu();
  }

  Serial.println("Polling Modbus...");
  if(modbus.poll()){
    // wdt_reset();
  }
  Serial.println("Loop finished.");
}
