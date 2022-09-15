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
H_voltageToInclinationVelocity = control.minreal(control.parallel(H_voltageToPendulumVelocity, H_voltageToRobotVelocity))


# Discretization of H_voltageToInclinationAngle and H_voltageToRobotAnlge
dt = 0.005
H_voltageToInclinationAngleDiscrete = control.sample_system(H_voltageToInclinationAngle, dt, method='zoh')
H_voltageToInclinationVelocityDiscrete = control.sample_system(H_voltageToInclinationVelocity, dt, method='zoh')
H_voltageToRobotAngleDiscrete = control.sample_system(H_voltageToRobotAngle, dt, method='zoh')


# Matrix solution of discretization
H_matrix = control.tf([H_voltageToInclinationAngleDiscrete.num[0],
                       H_voltageToInclinationVelocityDiscrete.num[0],
                       H_voltageToRobotAngleDiscrete.num[0]],
                      [H_voltageToInclinationAngleDiscrete.den[0],
                       H_voltageToInclinationVelocityDiscrete.den[0],
                       H_voltageToRobotAngleDiscrete.den[0]],
                      dt)
sys_ss_matrix = control.tf2ss(H_matrix)

number_of_iterations = 1000

class Plant:
    def __init__(self):
        self.x1_matrix = np.zeros([4,1])
        self.f_matrix = [[[0],[0],[0]]]
        t_matrix = np.arange(number_of_iterations)*dt

    def next_output(self, sys_input, noise=False):
        x = sys_ss_matrix.A.dot(self.x1_matrix) + sys_ss_matrix.B*sys_input
        y = sys_ss_matrix.C.dot(x) if noise == False else sys_ss_matrix.C.dot(x) + add_noise()
        self.x1_matrix = x
        self.f_matrix.append(y)

        return y
        

#____________________________________ PID IMPLEMENTATION ____________________________________
# Controller discretization
N = 20
T_s = dt
K_p = 10  #50
K_i = 400 #400
K_d = 10  #10
b0 = K_p*(1 + N*T_s) + K_i*T_s*(1 + N*T_s) + K_d*N
b1 = -(K_p*(2 + N*T_s) + K_i*T_s + 2*K_d*N)
b2 = K_p + K_d*N
a0 = (1 + N*T_s)
a1 = -(2 + N*T_s)
a2 = 1
class Controller:
    def __init__(self):
        self.e0 = 0
        self.e1 = 0
        self.e2 = 0
        self.u0 = 0
        self.u1 = 0
        self.u2 = 0
        self.controller_discrete_tf = control.tf([b0, b1, b2], [a0, a1, a2], T_s)

    def update(self, desired_angle, current_angle):
        self.e2 = self.e1
        self.e1 = self.e0
        self.e0 = desired_angle - current_angle
        self.u2 = self.u1
        self.u1 = self.u0
        self.u0 = (-a1/a0)*self.u1 + (-a2/a0)*self.u2 + (b0/a0)*self.e0 + (b1/a0)*self.e1 + (b2/a0)*self.e2

def add_noise():
    variance_accel = 0.002

    variance_gyro = 0.01
    gyro_expectation_noise = 0.05

    variance_encoder = 0.01

    w = np.array([np.random.normal(0, variance_accel), 
                  np.random.normal(gyro_expectation_noise, variance_gyro), 
                  np.random.normal(0, variance_encoder)]).reshape([3,1])
    
    return w

def complementary_filter(current_angle, y):
    T_s = dt
    tau = 0.2
    alpha = tau/(tau + dt)

    return (1 - alpha)*(current_angle + y[1][0]*T_s) + alpha*y[0][0]


#____________________________________ GRAPHICS SIMULATION ____________________________________
if __name__ == "__main__":
    
    plant = Plant()
    controller = Controller()
    inclination_angles = []
    robot_angles = []
    current_angle = 0
    desired_angle = 0.5
    for i in range(number_of_iterations):

        # controller
        controller.update(desired_angle, current_angle)

        # plant
        y = plant.next_output(controller.u0) # If you want noise add noise=True

        # complementary filter
        current_angle = complementary_filter(current_angle, y)
        inclination_angles.append(current_angle)
        robot_angles.append(y[2][0] - current_angle)

    # plt.plot(t_matrix, angles)
    # plt.show()

    import math
    import pygame as pg
    import pygamebg

    pg.init()
    (width, height) = (700, 700)
    ground_height = 100
    pendulum_width = 5
    pendulum_distance = height//2 - ground_height - 15
    window = pygamebg.open_window(width, height, "Robot valjak simulacija")

    def draw(inclination_angle, robot_angle):
        window.fill(pg.Color("sky blue"))

        pg.draw.circle(window, pg.Color("black"), (width//2, height//2), width//2-ground_height, 8)
        pg.draw.line(window, pg.Color("black"), (width//2, height//2), 
                                                (width//2 + pendulum_distance*math.sin(inclination_angle), 
                                                height//2 + pendulum_distance*math.cos(inclination_angle)),
                                                pendulum_width)
        pg.draw.line(window, pg.Color("red"), (width//2 + pendulum_distance*math.sin(robot_angle), 
                                            height//2 + pendulum_distance*math.cos(robot_angle)),
                                            (width//2 + (pendulum_distance + 15)*math.sin(robot_angle), 
                                            height//2 + (pendulum_distance + 15)*math.cos(robot_angle)),
                                            pendulum_width)

        pg.draw.rect(window, pg.Color("brown"), (0, height-ground_height, width, height))

    i = 0
    def new_frame():
        global i
        if i < len(inclination_angles):
            draw(inclination_angles[i], robot_angles[i])
        else:
            draw(inclination_angles[-1], robot_angles[-1])
        i += 1

    pygamebg.frame_loop((1/T_s)/10, new_frame)
    pg.quit()