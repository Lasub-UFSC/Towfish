
from pymodbus.client.serial import ModbusSerialClient
import time
import csv
import threading
import time
from datetime import datetime
import asyncio
import struct

class PollingClient:
    def __init__(self,onNewData, setFrequency=45,verbose=False,port="COM5"):
        print("Constructor")
        self.modbusClient = ModbusSerialClient(
            port=port,
            baudrate=38400,
            timeout=1
        )

        self.verbose=verbose
        self.modbusClient.connect()
        self.t=None
        self.stop_polling = threading.Event()
        self.onNewData = onNewData
        self.setFrequency = setFrequency

    def __del__(self):
       self.stop()

    def start(self):
        if self.t: return
        self.t = threading.Thread(target=self.polling)
        self.t.start()

    def stop(self):
        if self.t==None: return
        self.stop_polling.set()
        while self.t.is_alive():
            pass
        self.t = None

    def polling(self):
        with open( "./data/"+datetime.today().strftime('%Y-%m-%d--%H.%M')+'.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',',
                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
            while not self.stop_polling.is_set():
                if not self.modbusClient.connected:
                    self.modbusClient.connect()
                    print("trying to connect...")
                    time.sleep(1)
                else:
                    init=time.time()
                    data = self.readData()
                    if data ==None: continue
                    spamwriter.writerow(data)
                    if(self.verbose):print("On New Data")
                    asyncio.run(self.onNewData(self.convertData(data)))
                    time.sleep(max((1/self.setFrequency)-(time.time()-init),0))
                    if(self.verbose):print(1/(time.time()-init))
        print("Stopping Thread")
                
    def convertData(self,data):
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


    def readData(self):
        try:
            result = self.modbusClient.read_input_registers(address=0x00, count=14, slave=1)
            return result.registers
        except:
            print("error reading")
            self.modbusClient.close()
            return None