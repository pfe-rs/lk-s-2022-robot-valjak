import time
import serial
import numpy as np
import matplotlib.pyplot as plt

arduinoData = serial.Serial('COM8', 9600)
time.sleep(5)

dt = 0.005
number_of_iterations = 3000

t = np.arange(number_of_iterations)*dt
accel_output = []
gyro_output = []

dataPacket = arduinoData.readline() #reply
for i in range(number_of_iterations):
    while (arduinoData.inWaiting()==0):
        pass

    dataPacket = arduinoData.readline() #reply
    dataPacket = str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(",")
    accAngleY = float(splitPacket[0])
    gyroY = float(splitPacket[1])
    encoder = float(splitPacket[2])
    print ("accAngleY = ", accAngleY, "   gyroY = ", gyroY, "   encoder = ", encoder)

    accel_output.append(accAngleY)
    gyro_output.append(gyroY)

plt.plot(t, accel_output)
plt.show()

# Data saving
import csv

header = ["acceleration angle", "gyroscope velocity"]
data = [[accel_output[i], gyro_output[i]] for i in range(number_of_iterations)]

with open('data.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(data)