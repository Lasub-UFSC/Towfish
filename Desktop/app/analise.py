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


df = pd.read_csv(r'data\2025-08-21--15.13.csv',header=None)
timestamps = []
for index, row in df.iterrows():
    result_array = convertData(row.values)

    timestamps.append(result_array["timestamp"])

difference=[]

for i in range(1, len(timestamps)):
    # Subtract the previous item from the current item
    difference.append(timestamps[i] - timestamps[i - 1])


# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(difference, marker='o', linestyle='-')
plt.grid(True)
plt.show()
plt.clf()
plt.figure(figsize=(10, 6))
plt.plot(timestamps, marker='o', linestyle='-')
plt.grid(True)
plt.show()