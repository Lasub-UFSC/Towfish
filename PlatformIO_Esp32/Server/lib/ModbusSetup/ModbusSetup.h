#ifndef MODBUSSTEUP
#define MODBUSSTEUP
#include <ModbusRTUSlave.h>

namespace Modbus{

//MODBUS
enum ModbusRegister {
  Timestamp = 0,
  AccX = 2,
  AccY = 4,
  AccZ = 6,
  GyroX = 8,
  GyroY = 10,
  GyroZ = 12
};


class ModbusServer : public ModbusRTUSlave {
public:
    // Constructor
    ModbusServer(uint8_t numCoils,uint8_t numDiscreteInputs,uint8_t numHoldingRegisters,uint8_t numInputRegisters);


    ~ModbusServer();

    void update_input_register(ModbusRegister var, float value);
    void update_input_register(ModbusRegister var, unsigned long value);

private:
    const uint8_t numCoils;
    const uint8_t numDiscreteInputs;
    const uint8_t numHoldingRegisters;
    const uint8_t numInputRegisters;

    bool* coils;
    bool* discreteInputs;
    uint16_t* holdingRegisters;
    uint16_t* inputRegisters;
};


}

#endif