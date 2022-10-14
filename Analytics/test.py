import numpy as np

from system_identification_gray_box import model

params = np.load('param.npy')
params_thesis = (0.226, 0.065, 0.324, 0.0633, 0.007428, 3.294, 1.795, 0.00638, 0.00852, 0.140)
model(params_thesis, True)