
from pymodbus.client.serial import ModbusSerialClient
import time
import csv
import threading
import time
from datetime import datetime
import asyncio
import struct
import math
from ..KalmanFilter.KalmanFilter import KalmanFilter1D


class PollingClient:
    def __init__(self,onNewData, setFrequency=30,verbose=False,port="COM10"):
        print("Constructor")
        self.modbusClient = ModbusSerialClient(
            port=port,
            baudrate=38400,
            timeout=0.5,
            retries=1
        )

        self.verbose=verbose
        self.modbusClient.connect()
        self.t=None
        self.stop_polling = threading.Event()
        self.onNewData = onNewData
        self.setFrequency = setFrequency

        self.kalmanPitch = KalmanFilter1D()
        self.kalmanRoll = KalmanFilter1D()
        self.startTime = time.time()


    def __del__(self):
       self.stop()

    def start(self):
        if self.t: return
        self.t = threading.Thread(target=self.polling)
        print("Starting polling...")
        # self.calibrate()
        self.pitch = 0.0
        self.roll = 0.0
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
                    asyncio.run(self.onNewData(self.filterData(data)))
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

        convertedData["pressure"] = data[14]
        return convertedData



    def calibrate(self):
        print("calibrating...")
        roll_list=[]
        pitch_list =[]
        for i in range(100):
            if not self.modbusClient.connected:
                    self.modbusClient.connect()
                    print("trying to connect...")
                    time.sleep(1)
            data=self.readData()
            if(data== None): 
                i -=1
                continue
            convertedData = self.convertData(data)
            ax, ay, az =  convertedData["accx"]/10,  convertedData["accy"]/10,  convertedData["accz"]
            # Accelerometer angle estimation (in radians)
            roll_list.append(math.atan2(ay, az))
            pitch_list.append(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            time.sleep(1/20)
        self.pitch = sum(pitch_list)/len(pitch_list)
        self.roll = sum(roll_list)/len(roll_list)
        print(self.pitch,self.roll)
    
    def filterData(self,data):
        convertedData = self.convertData(data)
        ax, ay, az =  convertedData["accx"]/10,  convertedData["accy"]/10,  convertedData["accz"]
        gx, gy, _ =  convertedData["gyrox"],  convertedData["gyroy"],  convertedData["gyroz"]
        dt = 1.0/self.setFrequency 

        # Accelerometer angle estimation (in radians)
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Complementary filter
        alpha = 0.98
        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * pitch_acc

        # (Optional) Convert to degrees
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        return {"Pitch": pitch_deg, "Roll": roll_deg, "Depth":convertedData["pressure"]*100/1023}

    def readData(self):
        try:
            result = self.modbusClient.read_input_registers(address=0x00, count=15, slave=1)
            return result.registers
        except:
            print("error reading")
            self.modbusClient.close()
            return None
        