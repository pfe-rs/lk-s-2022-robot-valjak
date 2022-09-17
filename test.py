import numpy as np
import matplotlib.pyplot as plt
from transfer_function import transfer_function

# number_of_iterations = 150
# highpass = transfer_function([1, 0], [1, 10], number_of_iterations)

# plt.plot(np.arange(number_of_iterations)*0.005, highpass.inpute_response(np.arange(number_of_iterations)))
# plt.show()

import control
omega = 100
high_pass_filter = transfer_function([1, 0], [1, omega], 2001)
print(high_pass_filter.sys_ss)
t = np.linspace(0, 10, 2001)
f = high_pass_filter.inpute_response(np.sin(t))
plt.plot(t, f)
plt.show()