#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "ModbusSetup.h"

#define pressure_sensor 3
#define I2C_SDA 33
#define I2C_SCL 32
Adafruit_MPU6050 mpu;
Modbus::ModbusServer modbus(0,0,0,14);


void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Server Started.");

  // IMU
  Wire.setClock(400000);
  Wire.begin(I2C_SDA,I2C_SCL);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}


void loop() {
  modbus.update_input_register(Modbus::Timestamp, millis());
  if(mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    modbus.update_input_register(Modbus::AccX, a.acceleration.x);
    modbus.update_input_register(Modbus::AccY, a.acceleration.y);
    modbus.update_input_register(Modbus::AccZ, a.acceleration.z);
    modbus.update_input_register(Modbus::GyroX, g.gyro.x);
    modbus.update_input_register(Modbus::GyroY, g.gyro.y);
    modbus.update_input_register(Modbus::GyroZ, g.gyro.z);
  }
  modbus.poll();
}