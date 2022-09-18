import time
import serial
import numpy as np
import matplotlib.pyplot as plt

arduinoData = serial.Serial('COM8', 9600)
time.sleep(1)

dt = 0.005
number_of_iterations = 3000

t = np.arange(number_of_iterations)*dt
inclination_anlges = []
gyros = []
encoders = []
input = []

dataPacket = arduinoData.readline() #reply
for i in range(number_of_iterations):
    while (arduinoData.inWaiting()==0):
        pass

    dataPacket = arduinoData.readline() #reply
    dataPacket = str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(",")
    inclination_anlge = float(splitPacket[0])
    gyro = float(splitPacket[1])
    encoder = float(splitPacket[2])
    voltage = float(splitPacket[3])
    print ("Inclination_angle = ", inclination_anlge, " gyro = ", gyro, " encoder = ", encoder, " voltage = ", voltage)

    inclination_anlges.append(inclination_anlge)
    gyros.append(gyro)
    encoders.append(encoder)
    input.append(voltage)

plt.plot(t, inclination_anlges)
plt.plot(t, gyros)
plt.plot(t, encoders)
plt.plot(t, input)
plt.show()

# Data saving
import csv

header = ["Inclination anlges", "Gyros", "Encoders", "Voltage"]
data = [[inclination_anlges[i], gyros[i], encoders[i], input[i]] for i in range(number_of_iterations)]

with open('./Metrics/data.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(data)