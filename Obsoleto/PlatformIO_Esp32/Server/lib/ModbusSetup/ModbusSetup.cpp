#include "ModbusSetup.h"

namespace Modbus{

template<typename T>
void safeDeleteArray(T*& arr) {
    if (arr != nullptr) {
        delete[] arr;
        arr = nullptr;
    }
}

#define MODBUS_SERIAL Serial2
#define RXD2 16
#define TXD2 17
#define MODBUS_BAUD 38400
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1
const int16_t dePin = 4;

void ModbusServer::update_input_register(ModbusRegister var, float value){
  uint8_t* floatBytes = (uint8_t*)&value;

  inputRegisters[var] = ((uint16_t)floatBytes[1] << 8) | floatBytes[0];
  inputRegisters[var+1] = ((uint16_t)floatBytes[3] << 8) | floatBytes[2];
}

void ModbusServer::update_input_register(ModbusRegister var, unsigned long value){
  uint8_t* valueBytes = (uint8_t*)&value;

  inputRegisters[var] = ((uint16_t)valueBytes[1] << 8) | valueBytes[0];
  inputRegisters[var+1] = ((uint16_t)valueBytes[3] << 8) | valueBytes[2];
}

ModbusServer::ModbusServer(uint8_t nCoils,uint8_t nDiscreteInputs,uint8_t nHoldingRegisters,uint8_t nInputRegisters) : numCoils(nCoils),
    numDiscreteInputs(nDiscreteInputs),
    numHoldingRegisters(nHoldingRegisters),
    numInputRegisters(nInputRegisters),
    ModbusRTUSlave(MODBUS_SERIAL,dePin){
    coils = new bool[numCoils];
    discreteInputs = new bool[numDiscreteInputs];
    holdingRegisters = new uint16_t[numHoldingRegisters];
    inputRegisters = new uint16_t[numInputRegisters];
    
    configureCoils(coils, numCoils);
    configureDiscreteInputs(discreteInputs, numDiscreteInputs);
    configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
    configureInputRegisters(inputRegisters, numInputRegisters);
    MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG,RXD2,TXD2);
    begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
}

ModbusServer::~ModbusServer(){
  safeDeleteArray(coils);
  safeDeleteArray(discreteInputs);
  safeDeleteArray(holdingRegisters);
  safeDeleteArray(inputRegisters);
}

}