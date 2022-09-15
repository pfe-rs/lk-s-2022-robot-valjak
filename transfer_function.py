import control
import numpy as np
import matplotlib.pyplot as plt

class transfer_function:
    def __init__(self, num, den, number_of_iterations, dt=0.005):
        self.num = np.array(num)
        self.den = np.array(den)
        self.sys_tf = control.tf(num, den)
        self.sys_discrete_tf = control.sample_system(self.sys_tf, dt, method='zoh')
        self.sys_ss = control.tf2ss(self.sys_discrete_tf)
        self.number_of_iterations = number_of_iterations
        self.x1_matrix = np.zeros(self.sys_ss.B.shape)

    def next_output(self, sys_input, noise=False):
        x = self.sys_ss.A.dot(self.x1_matrix) + self.sys_ss.B*sys_input
        y = self.sys_ss.C.dot(x) #if noise == False else self.sys_discrete_tf.C.dot(x)
        self.x1_matrix = x

        return y

    def inpute_response(self, system_input):
        # t, f = control.step_response(self.sys_ss)
        # plt.plot(t, f)
        # plt.show()
        output = []
        for i in range(self.number_of_iterations):
            y = self.next_output(system_input[i])
            output.append(y[0][0])

        return output

if __name__=="__main__":

    dt = 0.005
    number_of_iterations = 2000
    system = transfer_function([1.720, 0.694], [1, 0.387, 89.179, 30.084], number_of_iterations)
    system_input = np.concatenate((np.concatenate((np.zeros(100), np.ones(400)*2, np.zeros(200))),
                               np.concatenate((np.ones(300)*5, np.zeros(150), np.ones(350))),
                               np.concatenate((np.zeros(300), np.ones(50)*10, np.zeros(150)))))
    output = system.inpute_response(system_input)

    plt.plot(np.arange(number_of_iterations)*dt, output)
    plt.show()