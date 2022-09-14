import csv

header = ["acceleration angle", "gyroscope velocity"]

angles = [1, 1, 1, 1,  1]
gyro = [2, 2, 2, 2,  2]
data = [[angles[i], gyro[i]] for i in range(len(angles))]

with open('test.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)

    writer.writerow(header)

    writer.writerows(data)