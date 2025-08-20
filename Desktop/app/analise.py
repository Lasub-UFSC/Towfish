import struct
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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


df = pd.read_csv(r'data\2025-08-14--15.22.csv',header=None)
timestamps = []
for index, row in df.iterrows():
    result_array = convertData(row.values)

    timestamps.append(result_array["timestamp"])

frequency=[]

for i in range(1, len(timestamps)):
    # Subtract the previous item from the current item
    difference = timestamps[i] - timestamps[i - 1]
    # Append the result to the new array
    frequency.append(1000/difference)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(frequency, marker='o', linestyle='-')
plt.title('Frequency of Data Requests 5 Hz')
plt.xlabel('Request Index')
plt.ylabel('Frequency (Hz)')
plt.grid(True)
plt.show()

# Save the plot
plt.savefig('frequency_plot_5.png')