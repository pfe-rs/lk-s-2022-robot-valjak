# import numpy as np

# from system_identification_gray_box import model

# params = np.load('param.npy')
# params_thesis = (0.226, 0.065, 0.324, 0.0633, 0.007428, 3.294, 1.795, 0.00638, 0.00852, 0.140)
# model(params_thesis, True)

import control
import numpy as np
import matplotlib.pyplot as plt

# Constants
r   = 0.226
d   = 0.065
g   = 9.81
R   = 0.324
j_1 = 0.0633
j_2 = 0.007428
m_1 = 3.294
m_2 = 1.795
k_e = 0.00638
k_t = 0.00852
T_v = 0.140

# Motor speed to Robot Angular Velocity
A = -2*j_2 - 2*d*d*m_2 + d*r*m_2
B = -2*g*d*m_2
C = j_1 + 2*j_2 + r*r*m_1 + (2*d*d-3*d*r+r*r)*m_2
D = T_v
E = 2*g*d*m_2

num1 = np.multiply([A, 0, B], 1/C)
den1 = np.multiply([C, D, E], 1/C)

H_motorSpeedToRobotVelocity = control.tf(num1, den1)
print('H_motorSpeedToRobotVelocity(s) = ', H_motorSpeedToRobotVelocity)
(p1, z1) = control.pzmap(H_motorSpeedToRobotVelocity, False)
print('poles = ', p1)
print('zeros = ', z1, end='\n\n')

t1, f1 = control.step_response(H_motorSpeedToRobotVelocity)
plt.plot(t1, f1)
plt.grid()
plt.show()