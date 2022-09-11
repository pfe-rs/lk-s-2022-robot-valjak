import csv
import numpy as np

measurements_file  = open('test.csv')
measurements = csv.reader(measurements_file)

# Data Extraction
header = next(measurements)
rows = []
for i in range(100):
	row = next(measurements)
	rows.append(row)
rows = np.array(rows)
rows = rows[:,:-1]

# 3D plotting
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d')

def f(x, y, a, b, c, d=0):
    return (d-a*x-b*y)/c

x = np.linspace(-6, 6, 30)
y = np.linspace(-6, 6, 30)

X, Y = np.meshgrid(x, y)
Z = f(X, Y, 1, 1, 1)
ax.contour3D(X, Y, Z, 50, cmap='binary')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z');

plt.show()