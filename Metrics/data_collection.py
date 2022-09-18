import time
import serial
import numpy as np
import matplotlib.pyplot as plt

arduinoData = serial.Serial('COM8', 9600)
time.sleep(1)

dt = 0.005
number_of_iterations = 3000

t = np.arange(number_of_iterations)*dt
output = []
input = []

dataPacket = arduinoData.readline() #reply
for i in range(number_of_iterations):
    while (arduinoData.inWaiting()==0):
        pass

    dataPacket = arduinoData.readline() #reply
    dataPacket = str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(",")
    pitch = float(splitPacket[0])
    voltage = float(splitPacket[1])
    print ("pitch = ", pitch, "   voltage = ", voltage)

    output.append(pitch)
    input.append(voltage)

plt.plot(t, output)
plt.plot(t, input)
plt.show()

# Data saving
import csv

header = ["pitch", "voltage"]
data = [[output[i], input[i]] for i in range(number_of_iterations)]

with open('./Metrics/data.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(data)