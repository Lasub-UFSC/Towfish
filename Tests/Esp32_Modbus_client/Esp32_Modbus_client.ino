#include <Arduino.h>
#include <ModbusRTUMaster.h>
#define MODBUS_SERIAL Serial2
#define RXD2 16
#define TXD2 17
#define MODBUS_BUAD 38400
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1
const int16_t dePin = 4;
ModbusRTUMaster modbus(MODBUS_SERIAL, dePin);

const uint8_t numCoils = 2;
const uint8_t numDiscreteInputs = 2;
const uint8_t numHoldingRegisters = 2;
const uint8_t numInputRegisters = 2;

bool coils[numCoils];
bool discreteInputs[numDiscreteInputs];
uint16_t holdingRegisters[numHoldingRegisters];
uint16_t inputRegisters[numInputRegisters];

unsigned long transactionCounter = 0;
unsigned long errorCounter = 0;

const char* errorStrings[] = {
  "success",
  "invalid id",
  "invalid buffer",
  "invalid quantity",
  "response timeout",
  "frame error",
  "crc error",
  "unknown comm error",
  "unexpected id",
  "exception response",
  "unexpected function code",
  "unexpected response length",
  "unexpected byte count",
  "unexpected address",
  "unexpected value",
  "unexpected quantity"
};



void setup() {
  Serial.begin(9600);

  // Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  MODBUS_SERIAL.begin(MODBUS_BUAD, MODBUS_CONFIG, RXD2,TXD2);
  modbus.begin(MODBUS_BUAD);
}

void loop() {
  uint8_t error;

  holdingRegisters[0] = random(0,10);
  holdingRegisters[1] = random(10,20);
  Serial.println("Sending:");

  Serial.println(holdingRegisters[0]);
  Serial.println(holdingRegisters[1]);

  error = modbus.writeMultipleHoldingRegisters(MODBUS_UNIT_ID, 0, holdingRegisters, numHoldingRegisters);
  if (error>0){
    Serial.println(errorStrings[error]);
  }
  delay(50);
  // printLog(MODBUS_UNIT_ID, 16, 0, numHoldingRegisters, error);

  // error = modbus.writeMultipleCoils(MODBUS_UNIT_ID, 0, coils, numCoils);
  // printLog(MODBUS_UNIT_ID, 15, 0, numHoldingRegisters, error);

  error = modbus.readInputRegisters(MODBUS_UNIT_ID, 0, inputRegisters, numInputRegisters);
  if (error>0){
    Serial.println(errorStrings[error]);
  }
  delay(50);

  // printLog(MODBUS_UNIT_ID, 4, 0, numHoldingRegisters, error);

  // error = modbus.readDiscreteInputs(MODBUS_UNIT_ID, 0, discreteInputs, numDiscreteInputs);
  // printLog(MODBUS_UNIT_ID, 2, 0, numHoldingRegisters, error);

  // Serial.println(discreteInputs[0]);
  // Serial.println(discreteInputs[1]);
  Serial.println("Recieved:");
  Serial.println(inputRegisters[0]);
  Serial.println(inputRegisters[1]);
  Serial.println();
  delay(2000);
}



void printLog(uint8_t unitId, uint8_t functionCode, uint16_t startingAddress, uint16_t quantity, uint8_t error) {
  transactionCounter++;
  if (error) errorCounter++;
  char string[128];
  sprintf(string, "%ld %ld %02X %02X %04X %04X %s", transactionCounter, errorCounter, unitId, functionCode, startingAddress, quantity, errorStrings[error]);
  Serial.print(string);
  if (error == MODBUS_RTU_MASTER_EXCEPTION_RESPONSE) {
    sprintf(string, ": %02X", modbus.getExceptionResponse());
    Serial.print(string);
  }
  Serial.println();
}
// //Receiver Code
 
// #define RXD2 16
// #define TXD2 17

// void setup() {
//   // Observe que o formato para definir uma porta serial é o seguinte: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
//   Serial.begin(9600);
//   //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
//   Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
//   Serial.println("Serial Txd está no pino: "+String(TX));
//   Serial.println("Serial Rxd está no pino: "+String(RX));
// }

// void loop() { //Escolha Serial1 ou Serial2 conforme necessário
//   while(Serial2.available()) {
//     Serial.println((Serial2.read()));
//   }
// }