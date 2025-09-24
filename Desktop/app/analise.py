import struct
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def convertData(data):
        convertedData= {}

        byte_data = struct.pack('<HH', int(data[0]), int(data[1]))
        convertedData["timestamp"] = struct.unpack('<I', byte_data)[0]

        byte_data = struct.pack('<HH', int(data[2]), int(data[3]))
        convertedData["accx"] = struct.unpack('<f', byte_data)[0]

        byte_data = struct.pack('<HH', int(data[4]), int(data[5]))
        convertedData["accy"] = struct.unpack('<f', byte_data)[0]

        byte_data = struct.pack('<HH', int(data[6]), int(data[7]))
        convertedData["accz"] = struct.unpack('<f', byte_data)[0]

        byte_data = struct.pack('<HH', int(data[8]), int(data[9]))
        convertedData["gyrox"] = struct.unpack('<f', byte_data)[0]


        byte_data = struct.pack('<HH', int(data[10]), int(data[11]))
        convertedData["gyroy"] = struct.unpack('<f', byte_data)[0]


        byte_data = struct.pack('<HH', int(data[12]), int(data[13]))
        convertedData["gyroz"] = struct.unpack('<f', byte_data)[0]

        return convertedData


df = pd.read_csv(r'data\2025-09-03--11.30.csv',header=None)
timestamps = []
pythonTime=[]
for index, row in df.iterrows():
    result_array = convertData(row.values)

    timestamps.append(result_array["timestamp"])
    pythonTime.append(row.values[-1])
difference=[]

lastTimestamp=timestamps[0]
lastPythonTime=pythonTime[0]
resets=0
for i in range(1, len(timestamps)):
    if(lastTimestamp>timestamps[i]):
         resets+=1
         print(i+1,resets, pythonTime[i]-lastPythonTime, int(pythonTime[i]))
    lastTimestamp=timestamps[i]
    lastPythonTime=pythonTime[i]
    
# for i in range(1, len(timestamps)):
#     # Subtract the previous item from the current item
#     difference.append(timestamps[i] - timestamps[i - 1])


# # Plot the results
# plt.figure(figsize=(10, 6))
# plt.plot(difference, marker='o', linestyle='-')
# plt.grid(True)
# plt.show()
# plt.clf()
# plt.figure(figsize=(10, 6))
# plt.plot(timestamps, marker='o', linestyle='-')
# plt.grid(True)
# plt.show()