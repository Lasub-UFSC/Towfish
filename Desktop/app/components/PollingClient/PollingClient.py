from pymodbus.client.serial import ModbusSerialClient
import time
import csv
import threading
import time
from datetime import datetime
import asyncio
from ..DataHandler.DataHandler import DataHandler

class PollingClient:
    def __init__(self,onNewData, setFrequency=30,verbose=False,port="COM10"):
        print("Constructor")
        self.modbusClient = ModbusSerialClient(
            port=port,
            baudrate=38400,
            timeout=0.5,
            retries=3
        )

        self.verbose=verbose
        self.modbusClient.connect()
        self.t=None
        self.stop_polling = threading.Event()
        self.onNewData = onNewData
        self.setFrequency = setFrequency
        self.loop = asyncio.get_running_loop()

        self.startTime = time.time()
        self.dataHandler = DataHandler(setFrequency)


    def __del__(self):
       self.stop()

    def start(self):
        if self.t: return
        self.t = threading.Thread(target=self.polling)
        print("Starting polling...")
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
                    print("trying to connect... ")
                    time.sleep(1)
                    data = self.readData()
                    if data !=None: print("Connected. Polling again...")
                else:
                    init=time.time()
                    data = self.readData()
                    if data ==None: continue
                    data.append(time.time()-self.startTime)
                    spamwriter.writerow(data)
                    if(self.verbose):print("On New Data")
                    asyncio.run_coroutine_threadsafe(self.onNewData(self.dataHandler.filterData(data)),self.loop)
                    time.sleep(max((1/self.setFrequency)-(time.time()-init),0))
                    if(self.verbose):print(1/(time.time()-init))
        print("Stopping Thread")

    def readData(self):
        try:
            result = self.modbusClient.read_input_registers(address=0x00, count=15, slave=1)
            return result.registers
        except:
            print("error reading")
            self.modbusClient.close()
            return None
        