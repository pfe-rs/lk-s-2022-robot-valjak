# Progressive implementation of alpha-beta-gama filter

import numpy as np
import matplotlib.pyplot as plt

which_to_plot = 4

# Example 1 - Weighting The Gold
x_0 = 1000
z = [1030, 989, 1017, 1009, 1013, 979, 1008, 1042, 1012, 1011]
x = [x_0]
for i in range(1, 11):
    K_n = 1/i
    x_i = x[-1] + K_n*(z[i-1] - x[-1])
    x.append(x_i)

if which_to_plot == 1:
    plt.plot(np.arange(1, 11), z, "-b", label="Measurements")
    plt.plot(np.arange(1, 11), x[1:], "-r", label="Estimates")
    plt.legend(loc="upper left")
    plt.xlabel("Iterations")
    plt.ylabel("Weight")
    plt.show()


# Example 2 - Tracking The Constant Veloctiy Aircraft On One Dimesnion
dt = 5
alpha = 0.2
beta = 0.1
x_0 = 30000
v_0 = 40
measurement = [30110, 30265, 30740, 30750, 31135, 31015, 31180, 31610, 31960, 31865]
estimation = []
prediction = [[x_0 + v_0*dt, v_0]]
for i in range(10):
    x_i = prediction[-1][0] + alpha*(measurement[i] - prediction[-1][0])
    v_i = prediction[-1][1] + beta*(measurement[i] - prediction[-1][0])/dt

    estimation.append([x_i, v_i])
    prediction.append([x_i + v_i*dt, v_i])

estimation = np.array(estimation)
prediction = np.array(prediction)
# print(estimation, end='\n\n')
# print(prediction)

if which_to_plot == 2:
    plt.plot(np.arange(1, 11)*dt, measurement, "-b", label="Measurements")
    plt.plot(np.arange(1, 11)*dt, estimation[:,0], "-r", label="Estimates")
    plt.plot(np.arange(1, 11)*dt, prediction[:-1,0], "-g", label="Prediction")
    plt.legend(loc="upper left")
    plt.xlabel("Time")
    plt.ylabel("Range")
    plt.show()


# Example 3 - Tracking Accelerating Aircraft In One Dimension
# The goal of this example is to show the existence of the Lag Error
# in constant acceleration
dt = 5
alpha = 0.2
beta = 0.1
x_0 = 30000
v_0 = 50
measurement = [30160, 30365, 30890, 31050, 31785, 32215, 33130, 34510, 36010, 37265]
estimation = []
prediction = [[x_0 + v_0*dt, v_0]]
for i in range(10):
    x_i = prediction[-1][0] + alpha*(measurement[i] - prediction[-1][0])
    v_i = prediction[-1][1] + beta*(measurement[i] - prediction[-1][0])/dt

    estimation.append([x_i, v_i])
    prediction.append([x_i + v_i*dt, v_i])

estimation = np.array(estimation)
prediction = np.array(prediction)
# print(estimation, end='\n\n')
# print(prediction)

if which_to_plot == 3:

    range_or_velocity = 1

    if range_or_velocity == 1:
        plt.plot(np.arange(1, 11)*dt, measurement, "-b", label="Measurements")
        plt.plot(np.arange(1, 11)*dt, estimation[:,0], "-r", label="Estimates")
        plt.plot(np.arange(1, 11)*dt, prediction[:-1,0], "-g", label="Prediction")
        plt.legend(loc="upper left")
        plt.xlabel("Time")
        plt.ylabel("Range")
        plt.show()
    elif range_or_velocity == 2:
        plt.plot(np.arange(1, 11)*dt, estimation[:,1], "-r", label="Estimates")
        plt.plot(np.arange(1, 11)*dt, prediction[:-1,1], "-g", label="Prediction")
        plt.legend(loc="upper left")
        plt.xlabel("Time")
        plt.ylabel("Velocity")
        plt.show()


# Example 4 - Tracking Accelerating Aircraft With The Alpha-Beta-Gama Filter
dt = 5
alpha = 0.5
beta = 0.4
gama = 0.1
x_0 = 30000
v_0 = 50
a_0 = 0
measurement = [30160, 30365, 30890, 31050, 31785, 32215, 33130, 34510, 36010, 37265]
estimation = []
prediction = [[x_0 + v_0*dt + 0.5*a_0*dt*dt, v_0 + a_0*dt, a_0]]
for i in range(10):
    x_i = prediction[-1][0] + alpha*(measurement[i] - prediction[-1][0])
    v_i = prediction[-1][1] + beta*(measurement[i] - prediction[-1][0])/dt
    a_i = prediction[-1][2] + gama*(measurement[i] - prediction[-1][0])/(0.5*dt*dt)

    estimation.append([x_i, v_i, a_i])
    prediction.append([x_i + v_i*dt + 0.5*a_i*dt*dt, v_i + a_i*dt, a_i])

estimation = np.array(estimation)
prediction = np.array(prediction)
# print(estimation, end='\n\n')
# print(prediction)

if which_to_plot == 4:

    range_or_velocity_or_acceleration = 1

    if range_or_velocity_or_acceleration == 1:
        plt.plot(np.arange(1, 11)*dt, measurement, "-b", label="Measurements")
        plt.plot(np.arange(1, 11)*dt, estimation[:,0], "-r", label="Estimates")
        plt.plot(np.arange(1, 11)*dt, prediction[:-1,0], "-g", label="Prediction")
        plt.legend(loc="upper left")
        plt.xlabel("Time")
        plt.ylabel("Range")
        plt.show()
    elif range_or_velocity_or_acceleration == 2:
        plt.plot(np.arange(1, 11)*dt, estimation[:,1], "-r", label="Estimates")
        plt.plot(np.arange(1, 11)*dt, prediction[:-1,1], "-g", label="Prediction")
        plt.legend(loc="upper left")
        plt.xlabel("Time")
        plt.ylabel("Velocity")
        plt.show()
    elif range_or_velocity_or_acceleration == 3:
        plt.plot(np.arange(1, 11)*dt, estimation[:,2], "-r", label="Estimates")
        plt.plot(np.arange(1, 11)*dt, prediction[:-1,2], "-g", label="Prediction")
        plt.legend(loc="upper left")
        plt.xlabel("Time")
        plt.ylabel("Acceleration")
        plt.show()