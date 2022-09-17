import math
import control
import numpy as np
import matplotlib.pyplot as plt
from transfer_function import transfer_function


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
T_v = 0.140 #T_v=0.140


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
H_voltageToPendulumAngleDiscrete = control.sample_system(H_voltageToPendulumAngle, dt, method='zoh')


# Matrix solution of discretization
H_matrix = control.tf([H_voltageToInclinationAngleDiscrete.num[0],
                       H_voltageToInclinationVelocityDiscrete.num[0],
                       H_voltageToPendulumAngleDiscrete.num[0]],
                      [H_voltageToInclinationAngleDiscrete.den[0],
                       H_voltageToInclinationVelocityDiscrete.den[0],
                       H_voltageToPendulumAngleDiscrete.den[0]],
                      dt)
sys_ss_matrix = control.tf2ss(H_matrix)

number_of_iterations = 10000

def sign_function(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1

def add_noise():
    variance_accel = 0.002

    variance_gyro = 0.01
    gyro_expectation_noise = 0.05

    variance_encoder = 0.01

    w = np.array([np.random.normal(0, variance_accel), 
                  np.random.normal(gyro_expectation_noise, variance_gyro), 
                  np.random.normal(0, variance_encoder)]).reshape([3,1])
    
    return w

class Plant:
    def __init__(self):
        self.x1_matrix = np.zeros([4,1])
        self.f_matrix = [[[0],[0],[0]]]
        t_matrix = np.arange(number_of_iterations)*dt

    def next_output(self, sys_input, noise=False):
        x = sys_ss_matrix.A.dot(self.x1_matrix) + sys_ss_matrix.B*sys_input
        y = sys_ss_matrix.C.dot(self.x1_matrix) if noise == False else sys_ss_matrix.C.dot(x) + add_noise()
        self.x1_matrix = x
        self.f_matrix.append(y)

        return y
        

#____________________________________ PID IMPLEMENTATION ____________________________________
# Controller discretization
class Controller:
    def __init__(self, K_p, K_i, K_d, N=20, T_s=dt):
        self.b0 = K_p*(1 + N*T_s) + K_i*T_s*(1 + N*T_s) + K_d*N
        self.b1 = -(K_p*(2 + N*T_s) + K_i*T_s + 2*K_d*N)
        self.b2 = K_p + K_d*N
        self.a0 = (1 + N*T_s)
        self.a1 = -(2 + N*T_s)
        self.a2 = 1
        self.N = N
        self.T_s = T_s

        self.e0 = 0
        self.e1 = 0
        self.e2 = 0
        self.u0 = 0
        self.u1 = 0
        self.u2 = 0
        self.controller_discrete_tf = control.tf([self.b0, self.b1, self.b2], [self.a0, self.a1, self.a2], self.T_s)

    def update(self, desired_value, current_value):
        self.e2 = self.e1
        self.e1 = self.e0
        self.e0 = desired_value - current_value
        self.u2 = self.u1
        self.u1 = self.u0
        self.u0 = ((-self.a1/self.a0)*self.u1 + (-self.a2/self.a0)*self.u2 + (self.b0/self.a0)*self.e0 + 
                   (self.b1/self.a0)*self.e1 + (self.b2/self.a0)*self.e2)

def complementary_filter(current_angle, y):
    T_s = dt
    tau = 0.2
    alpha = tau/(tau + dt)

    return (1 - alpha)*(current_angle + y[1][0]*T_s) + alpha*y[0][0]


def PID_inclination_angle():
    plant = Plant()
    K_p, K_i, K_d = 6*2, 20, 0.45*5 #T_u=0.6 i K_u=10
    controller = Controller(K_p, K_i, K_d)

    inclination_angles = []
    robot_angles = []
    current_angle = 0
    desired_angle = 0.025

    for _ in range(number_of_iterations):

        # controller
        controller.update(desired_angle, current_angle)

        # plant
        max_voltage = 12
        y = plant.next_output(min(controller.u0, max_voltage)) # If you want noise add noise=True

        # complementary filter
        current_angle = complementary_filter(current_angle, y)
        inclination_angles.append(current_angle)
        robot_angles.append(-(y[2][0] - current_angle))

    plt.plot(np.arange(number_of_iterations)*dt, inclination_angles)
    plt.show()
    return (inclination_angles, robot_angles)

def PID_robot_angular_velocity():
    plant = Plant()
    K_p, K_i, K_d = 6*2, 20, 0.45*5
    inclination_angle_controller = Controller(K_p, K_i, K_d)
    K_p, K_i, K_d = -0.5, -0.1, -0.5 #-0.5, -0.1, -0.5
    angular_velocity_controller = Controller(K_p, K_i, K_d)

    desired_velocity = -0.3
    inclination_angles = [0]
    robot_angles = [0]
    robot_angular_velocities = [0]
    omega = 1
    high_pass_filter = transfer_function([omega, 0], [1, omega])

    for _ in range(number_of_iterations):

        # Angular velocity controller
        angular_velocity_controller.update(desired_velocity, robot_angular_velocities[-1])

        # Inclination angle controller
        max_inclination_angle = 1.5
        inclination_angle_controller.update(min(angular_velocity_controller.u0, max_inclination_angle), inclination_angles[-1])

        # Plant
        max_voltage = 12
        y = plant.next_output(min(inclination_angle_controller.u0, max_voltage))

        # Complementary filter
        inclination_angle = complementary_filter(inclination_angles[-1], y)

        # Differentiator
        robot_angle = -(y[2][0] - inclination_angle) #-(pendulum_angle - inclination_angle)
        robot_angular_velocity = high_pass_filter.next_output(robot_angle)

        # Update
        inclination_angles.append(inclination_angle)
        robot_angles.append(robot_angle)
        robot_angular_velocities.append(robot_angular_velocity)
    
    t = np.arange(number_of_iterations)*dt
    robot_angular_velocities = np.array(robot_angular_velocities)
    plt.plot(t, robot_angular_velocities[1:])
    plt.show()
    return (inclination_angles, robot_angles)

def PID_robot_angle():
    plant = Plant()
    K_p, K_i, K_d = 6*2, 20, 0.45*5
    inclination_angle_controller = Controller(K_p, K_i, K_d)
    K_p, K_i, K_d = -0.5, -0.1, -0.5 #-0.5, -0.1, -0.5
    angular_velocity_controller = Controller(K_p, K_i, K_d)
    K_p, K_i, K_d = 0.5, 0, 0
    robot_angle_controller = Controller(K_p, K_i, K_d)

    desired_robot_angle = -1.56
    inclination_angles = [0]
    robot_angles = [0]
    robot_angular_velocities = [0]
    omega = 1
    high_pass_filter = transfer_function([omega, 0], [1, omega])

    for _ in range(number_of_iterations):

        # Robot angle controller
        robot_angle_controller.update(desired_robot_angle, robot_angles[-1])

        # Angular velocity controller
        max_angular_velocity = -0.9
        angular_velocity_controller.update(max(robot_angle_controller.u0, max_angular_velocity), robot_angular_velocities[-1])

        # Inclination angle controller
        max_inclination_angle = 1.5
        inclination_angle_controller.update(min(angular_velocity_controller.u0, max_inclination_angle), inclination_angles[-1])

        # Plant
        max_voltage = 12
        y = plant.next_output(min(inclination_angle_controller.u0, max_voltage))

        # Complementary filter
        inclination_angle = complementary_filter(inclination_angles[-1], y)

        # Differentiator
        robot_angle = -(y[2][0] - inclination_angle) #-(pendulum_angle - inclination_angle)
        robot_angular_velocity = high_pass_filter.next_output(robot_angle)

        # Update
        inclination_angles.append(inclination_angle)
        robot_angles.append(robot_angle)
        robot_angular_velocities.append(robot_angular_velocity)

    t = np.arange(number_of_iterations)*dt
    robot_angles = np.array(robot_angles)
    robot_angular_velocities = np.array(robot_angular_velocities)
    plt.plot(t, robot_angles[1:])
    plt.plot(t, robot_angular_velocities[1:])
    plt.show()
    return (inclination_angles, robot_angles)


#____________________________________ GRAPHICS SIMULATION ____________________________________
if __name__ == "__main__":

    import math
    import pygame as pg
    import pygamebg

    # inclination_angles, robot_angles = PID_inclination_angle()
    # inclination_angles, robot_angles = PID_robot_angular_velocity()
    inclination_angles, robot_angles = PID_robot_angle()
    
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

    T_s = dt
    pygamebg.frame_loop((1/T_s), new_frame)
    pg.quit()
    