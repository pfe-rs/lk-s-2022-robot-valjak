import serial as ps
import os

port = "COM5"
baudrate = 115200

Serial = ps.Serial(port)
print(Serial.name,baudrate)
File = open("data.csv" ,'a')

i = 0

while True:
    #print("ebfyuGEWIFUIEH")
    i = i + 1
    line = Serial.readlines()
    print(line)
    with open('data.csv', 'a') as f:
        f.write('\n'.join(str(line)))
    
