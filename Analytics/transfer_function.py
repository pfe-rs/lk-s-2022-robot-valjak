import control
import numpy as np
import matplotlib.pyplot as plt

class transfer_function:
    def __init__(self, num, den, number_of_iterations=1000, dt=0.005):
        self.num = np.array(num)
        self.den = np.array(den)
        self.sys_tf = control.tf(num, den)
        self.sys_discrete_tf = control.sample_system(self.sys_tf, dt, method='zoh')
        self.sys_ss = control.tf2ss(self.sys_discrete_tf)
        self.number_of_iterations = number_of_iterations
        self.x1 = np.zeros(self.sys_ss.B.shape)

    def next_output(self, sys_input):
        x = self.sys_ss.A.dot(self.x1) + self.sys_ss.B*sys_input
        y = self.sys_ss.C.dot(self.x1) + self.sys_ss.D*sys_input
        self.x1 = x

        return y

    def inpute_response(self, system_input):
        output = []
        for i in range(self.number_of_iterations):
            y = self.next_output(system_input[i])
            output.append(y[0][0])

        return output

if __name__=="__main__":

    dt = 0.005
    number_of_iterations = 2000
    system = transfer_function([1, 0], [1, 5], number_of_iterations)
    system_input = np.ones(number_of_iterations)
    output = system.inpute_response(system_input)

    plt.plot(np.arange(number_of_iterations)*dt, output)
    plt.show()