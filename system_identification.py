import control
import numpy as np
import matplotlib.pyplot as plt
from transfer_function import transfer_function
from scipy.optimize import differential_evolution

import PID_simulation as pid

dt = 0.005
number_of_iterations = 2000
system_t = np.arange(number_of_iterations)*dt
system_input = np.concatenate((np.concatenate((np.zeros(100), np.ones(400)*2, np.zeros(200))),
                               np.concatenate((np.ones(300)*5, np.zeros(150), np.ones(350))),
                               np.concatenate((np.zeros(300), np.ones(50)*10, np.zeros(150)))))

plant = pid.Plant()
inclination_angles = []
for i in range(number_of_iterations):
    y = plant.next_output(system_input[i], noise=True)
    inclination_angles.append(y[0][0])

#print(type(pid.H_voltageToInclinationAngle.num[0][0]))
#          1.711 s + 0.8856
# ----------------------------------
# s^3 + 0.4945 s^2 + 88.99 s + 38.46

def arx_model(A, B, u, e=None):
    broj_odbiraka = len(u)
    
    A = np.flip(-A[1:])
    B = np.flip(B)

    a = len(A)
    b = len(B)

    padding = np.zeros(a)
    if e is None:
        e = np.zeros(len(u))

    y = np.zeros(broj_odbiraka + a)
    u = np.concatenate([padding, u])
    e = np.concatenate([padding, e])

    for i in range(a, broj_odbiraka + a):
        y[i] = A @ y[i - a: i] + B @ u[i - b: i] + e[i]

    return y[a:]

def inclination_angle_error_function(ulaz):
    b1, b0, a2, a1, a0 = ulaz
    new_tf = control.tf(np.array([b1, b0]), np.array([1, a2, a1, a0]))
    new_tf = control.sample_system(new_tf, dt, method='zoh')
    A_ = new_tf.den[0][0]
    B_ = new_tf.num[0][0]

    y_est = arx_model(A_, B_, system_input)

    return np.sum(np.square(inclination_angles - y_est))

x_min = -3
x_max = 3
xx = [(x_min, x_max), (x_min, x_max), (x_min, x_max), (70, 90), (30, 40)]
optimized_inclination_angle = differential_evolution(inclination_angle_error_function, bounds=xx)

print(optimized_inclination_angle.x)
b1, b0, a2, a1, a0 = optimized_inclination_angle.x
print(b1)
estimated_tf = transfer_function([b1, b0], [1, a2, a1, a0], number_of_iterations)

#plt.plot(system_t, system_input)
plt.plot(system_t, inclination_angles)
plt.plot(system_t, estimated_tf.inpute_response(system_input))
plt.show()