
from pymodbus.client.serial import ModbusSerialClient
import time
from pymongo import MongoClient



class PollingClient:
    def __init__(self,dbHost=None,verbose=False,port="COM11"):
        self.modbusClient = ModbusSerialClient(
            port=port,
            baudrate=38400,
            timeout=1
        )

        self.mongoClient = MongoClient(dbHost)
        self.collection= self.mongoClient.towfishdb.get_collection("test1")
        self.verbose=verbose
        self.modbusClient.connect()
        # if(not self.modbusClient.connected) Exception()

    # def start(self):
    #     data = self.readData()
    #     # self.collection.insert_one()
    #     #send data to DB

    def readData(self):
        init=time.time()
        result = self.modbusClient.read_input_registers(address=0x00, count=14, slave=1)
        if(self.verbose):print(time.time()-init)
        return result.registers
