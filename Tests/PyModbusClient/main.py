# import asyncio
# import pymodbus.client as ModbusClient
# from pymodbus import (
#     FramerType,
#     ModbusException,
#     pymodbus_apply_logging_config,
# )

# import csv
# def get_csv_file(name) :
#     return open(name, 'r+', newline='')

# async def run_async_simple_client(port, framer=FramerType.RTU):
#     """Run async client."""
#     # activate debugging
#     pymodbus_apply_logging_config("DEBUG")

#     print("get client")
#     client = ModbusClient.AsyncModbusSerialClient(
#         port,
#         framer=framer,
#         # timeout=10,
#         # retries=3,
#         baudrate=9600,
#         bytesize=8,
#         parity="N",
#         stopbits=1,
#         # handle_local_echo=False,
#     )

#     print("connect to server")
#     await client.connect()
#     # test client is connected
#     assert client.connected
#     csv_file =get_csv_file("")
#     file_write = csv.writer(csv_file)

#     print("get and verify data")
#     try:
#         # See all calls in client_calls.py
#         rr = await client.read_coils(1, count=1, device_id=1)
#     except ModbusException as exc:
#         print(f"Received ModbusException({exc}) from library")
#         client.close()
#         return
#     if rr.isError():
#         print(f"Received exception from device ({rr})")
#         # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
#         client.close()
#         return
#     try:
#         # See all calls in client_calls.py
#         rr = await client.read_holding_registers(10, count=2, device_id=1)
#     except ModbusException as exc:
#         print(f"Received ModbusException({exc}) from library")
#         client.close()
#         return
#     if rr.isError():
#         print(f"Received exception from device ({rr})")
#         # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
#         client.close()
#         return
#     value_int32 = client.convert_from_registers(rr.registers, data_type=client.DATATYPE.INT32)

#     file_write.writerow([value_int32])
#     print(f"Got int32: {value_int32}")
#     print("close connection")
#     client.close()
#     csv_file.close()


# if __name__ == "__main__":
#     asyncio.run(
#         run_async_simple_client("dev/port0",FramerType.RTU), debug=True
#     )


from pymodbus.client.serial import ModbusSerialClient
import time
import statistics
import struct


def convertData(data):
        convertedData= {}

        byte_data = struct.pack('<HH', data[0], data[1])
        convertedData["timestamp"] = struct.unpack('<I', byte_data)[0]

        byte_data = struct.pack('<HH', data[2], data[3])
        convertedData["accx"] = struct.unpack('<f', byte_data)[0]

        byte_data = struct.pack('<HH', data[4], data[5])
        convertedData["accy"] = struct.unpack('<f', byte_data)[0]

        byte_data = struct.pack('<HH', data[6], data[7])
        convertedData["accz"] = struct.unpack('<f', byte_data)[0]

        byte_data = struct.pack('<HH', data[8], data[9])
        convertedData["gyrox"] = struct.unpack('<f', byte_data)[0]


        byte_data = struct.pack('<HH', data[10], data[11])
        convertedData["gyroy"] = struct.unpack('<f', byte_data)[0]


        byte_data = struct.pack('<HH', data[12], data[13])
        convertedData["gyroz"] = struct.unpack('<f', byte_data)[0]

        return convertedData

client = ModbusSerialClient(
    port='COM5',  # or COM port on Windows
    baudrate=38400,
    timeout=1
)

times=[]
if client.connect():
    print(client.connected)
    lastTime = time.time()
    for i in range(100):
        time.sleep(0.05)
        try:
            result = client.read_input_registers(address=0x00, count=14, slave=1)
            print(convertData(result.registers))
            
            currentTime = time.time()
            times.append(currentTime-lastTime)
            lastTime=currentTime
        
        except:
            print("error at", str(i), "request")
            break
    client.close()
    print("Mean:", statistics.mean(times)*1000,"ms")
    print("Per Secons:", 1/statistics.mean(times))