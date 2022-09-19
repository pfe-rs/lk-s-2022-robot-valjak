from cmath import inf, nan
import csv
import pickle
import control
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution


dt = 0.005
number_of_iterations = 3000
t = np.arange(number_of_iterations)*dt

# Read input data
inclination_angles = []
gyros = []
encoders = []
system_input = []
with open("./Metrics/data.csv") as file:
    csvreader = csv.reader(file)
    for i, row in enumerate(csvreader):
        if i > 0:
            inclination_angles.append(float(row[0]))
            gyros.append(float(row[1]))
            encoders.append(float(row[2]))
            system_input.append(float(row[3]))

plt.plot(t, inclination_angles)
# plt.plot(t, gyros)
plt.plot(t, encoders)
plt.plot(t, system_input)
plt.show()
exit()

def model(ulaz, plot_data=False):
    r, d, R, j_1, j_2, m_1, m_2, k_e, k_t, T_v = ulaz
    print(ulaz)
    g = 9.81

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
    # H_voltageToInclinationAngle = control.parallel(H_voltageToPendulumAngle, H_voltageToRobotAngle)
    # H_voltageToInclinationVelocity = control.parallel(H_voltageToPendulumVelocity, H_voltageToRobotVelocity)

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

    number_of_iterations = 3000

    class Plant:
        def __init__(self):
            self.x1_matrix = np.zeros([sys_ss_matrix.A.shape[1], 1])
            self.f_matrix = [[[0],[0],[0]]]
            t_matrix = np.arange(number_of_iterations)*dt

        def next_output(self, sys_input):
            x = sys_ss_matrix.A.dot(self.x1_matrix) + sys_ss_matrix.B*sys_input
            y = sys_ss_matrix.C.dot(self.x1_matrix) + sys_ss_matrix.D*sys_input
            self.x1_matrix = x

            return y

        def input_response(self, system_input):
            for i in range(number_of_iterations):
                y = self.next_output(system_input[i])
                self.f_matrix.append(y)
            
            return np.array(self.f_matrix)
    
    plant = Plant()
    # system_output = plant.input_response(system_input)
    system_output = plant.input_response(np.ones(number_of_iterations)*3)
    
    if plot_data == True:
        plt.plot(t, inclination_angles)
        plt.plot(t, system_output[1:,0,0])
        plt.show()

    return system_output

best_loss = np.inf
def error_functions(ulaz):
    for param in ulaz:
        if param == nan:
            return 1e10

    system_output = model(ulaz)

    ret = np.mean(np.square(system_output[1:,0,0] - (inclination_angles - np.mean(inclination_angles[:300])))) #+ 
                #   np.square(system_output[1:,1,0] - gyros) + 
                #   np.square(system_output[1:,2,0] - encoders))
    print(ret)
    return ret

if __name__=="__main__":
    x_min = 1e-3
    x_max = 1
    xx = [
        (0.2, 0.2), #r
        (0.05, 0.15),   #d
        (x_min, x_max), #R
        (x_min, 0.1),   #j_1
        (x_min, 0.1),   #j_2
        (x_min, 2),     #m_1
        (x_min, 2), #m_2
        (x_min, 1e-2), #k_e
        (x_min, 1e-2), #k_t
        (x_min, x_max)] #T_v
    optimized_params = differential_evolution(error_functions, bounds=xx, maxiter=10, popsize=10)
    print(optimized_params)

    model(optimized_params.x, True)

    np.save('param.npy', optimized_params.x)