import struct
import math


class KalmanFilter1D:
    def __init__(self, initial_state=0.0, initial_uncertainty=1.0, process_noise=1.0, measurement_noise=9.0):

        self.state = initial_state
        self.uncertainty = initial_uncertainty
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

class DataHandler:

    def __init__(self,setFrequency):
        self.roll_list = []
        self.pitch_list = []
        self.calibrated=False
        self.setFrequency = setFrequency

    def __convertData(self,data):
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


    def __calibrate(self,data):
        if(len(self.pitch_list)<200):
            convertedData = self.__convertData(data)
            ax, ay, az =  convertedData["accx"]/10,  convertedData["accy"]/10,  convertedData["accz"]
            # Accelerometer angle estimation (in radians)
            self.roll_list.append(math.atan2(ay, az))
            self.pitch_list.append(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        else:
            self.pitch = sum(self.pitch_list)/len(self.pitch_list)
            self.roll = sum(self.roll_list)/len(self.roll_list)
            print(f"Calibration: Pitch {self.pitch} - Roll {self.roll}")
            self.calibrated = True
    
    def filterData(self,data):
        if(not self.calibrated):
            self.__calibrate(data)
            return {"Pitch": 45, "Roll": 45, "Depth":45, "Timestamp":data[-1]}

        convertedData = self.__convertData(data)
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
        return {"Pitch": pitch_deg, "Roll": roll_deg, "Depth":convertedData["pressure"]*100/1023, "Timestamp":data[-1]}