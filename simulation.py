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


# Voltage to Robot Angular Velocity
A = -2*d*g*k_t*m_2
B = k_t*(-2*j_2 + d*(-2*d+r)*m_2)
C = -d*g*R*T_v*m_2 + 2*d*g*k_e*k_t*m_2
D = -T_v*k_e*k_t + d*g*R*j_1*m_2 + d*g*r*r*R*m_1*m_2 + d*g*r*r*R*m_2*m_2
E = -R*T_v*j_2 + 2*j_2*k_e*k_t - d*d*R*T_v*m_2 + k_e*k_t*(j_1 + r*r*m_1 
        + (2*d*d - 3*d*r + r*r)*m_2)
F = d*d*R*j_1*m_2 + d*d*r*r*R*m_1*m_2 + R*j_2*(j_1 + r*r*(m_1 + m_2))

num2 = np.multiply([B, 0, A], 1/F)
den2 = np.multiply([F, -E, D, -C], 1/F)
H_voltageToRobotVelocity = control.tf(num2, den2)


# Voltage to Pendulum Angular Velocity
A = -d*g*R*T_v*m_2 + 2*d*g*k_e*k_t*m_2
B = -T_v*k_e*k_t + d*g*R*j_1*m_2 + d*g*r*r*R*m_1*m_2 + d*g*r*r*R*m_2*m_2
C = -R*T_v*j_2 - d*d*R*T_v*m_2 + k_e*k_t*(j_1 + 2*j_2 + r*r*m_1 + 
        (2*d*d - 3*d*r + r*r)*m_2)
D = R*j_1*j_2 + d*d*R*j_1*m_2 + d*d*r*r*R*m_1*m_2 + r*r*R*j_2*(m_1 + m_2)
E = k_t*(j_1 + 2*j_2 + r*r*m_1 + (2*d*d - 3*d*r + r*r)*m_2)
F = -k_t*T_v
G = 2*d*g*m_2*k_t

num3 = np.multiply([E, -F, G], 1/D)
den3 = np.multiply([D, -C, B, -A], 1/D)
H_voltageToPendulumVelocity = control.tf(num3, den3)


# Voltage to Angle transfer functions
H_voltageToPendulumAngle = control.series(H_voltageToPendulumVelocity, control.tf([1], [1, 0]))
H_voltageToRobotAngle = control.series(H_voltageToRobotVelocity, control.tf([1], [1, 0]))
H_voltageToInclinationAngle = control.minreal(control.parallel(H_voltageToPendulumAngle, H_voltageToRobotAngle))


# Discretization of H_voltageToInclinationAngle and H_voltageToRobotAnlge
H_voltageToInclinationAngleDiscrete = control.sample_system(H_voltageToInclinationAngle, 1/200, method='zoh')
H_voltageToRobotAngleDiscrete = control.sample_system(H_voltageToRobotAngle, 1/200, method='zoh')


# Matrix solution of discretization
H_matrix = control.tf([H_voltageToInclinationAngleDiscrete.num[0],
                       H_voltageToInclinationAngleDiscrete.num[0],
                       H_voltageToRobotAngleDiscrete.num[0]],
                      [H_voltageToInclinationAngleDiscrete.den[0],
                       H_voltageToInclinationAngleDiscrete.den[0],
                       H_voltageToRobotAngleDiscrete.den[0]],
                      1/200)
sys_ss_matrix = control.tf2ss(H_matrix)

x1_matrix = np.zeros([4,1])
t_matrix = np.arange(1000)*0.005
f_matrix = []

def next_state_output(sys_input):
    variance_accel = 0.002
    variance_gyro = 0.002
    variance_encoder = 0.01
    w = np.array([np.random.normal(0, variance_accel), 
                  np.random.normal(0, variance_gyro), 
                  np.random.normal(0, variance_encoder)]).reshape([3,1])
    global x1_matrix
    x = sys_ss_matrix.A.dot(x1_matrix) + sys_ss_matrix.B
    y = sys_ss_matrix.C.dot(x) + w
    x1_matrix = x
    f_matrix.append(y)

    return y

for i in range(1000):
    next_state_output(1)
    
f_matrix = np.array(f_matrix)
plt.plot(t_matrix, f_matrix[:,0,0])
plt.show()