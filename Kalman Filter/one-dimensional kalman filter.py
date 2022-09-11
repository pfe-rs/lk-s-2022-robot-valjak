import numpy as np
import matplotlib.pyplot as plt

which_to_plot = 4

# Example 5 - Estimating The Height Of A Building
x_0 = 60
sigma_human = 15
p_0 = sigma_human**2
sigma_measurement = 5
measurement_uncertainty = sigma_measurement**2
measurements = [48.54, 47.11, 55.01, 55.15, 49.89, 40.85, 46.72, 50.05, 51.27, 49.95]
state_estimation = []
state_prediction = [x_0]
variance_estimation = []
variance_prediction = [p_0]

for i in range(10):
    kalman_gain = variance_prediction[-1]/(variance_prediction[-1] + measurement_uncertainty)

    new_state_estimation = (1-kalman_gain)*state_prediction[-1] + kalman_gain*measurements[i]
    new_variance_estimation = (1-kalman_gain)*variance_prediction[-1]

    state_estimation.append(new_state_estimation)
    variance_estimation.append(new_variance_estimation)

    new_state_prediction = new_state_estimation
    new_variance_prediction = new_variance_estimation

    state_prediction.append(new_state_prediction)
    variance_prediction.append(new_variance_prediction)

if which_to_plot == 1:
    plt.plot(np.arange(1, 11), measurements, "-b", label="Measurements")
    plt.plot(np.arange(1, 11), state_estimation, "-r", label="Estimates")
    plt.legend(loc="upper left")
    plt.xlabel("Measurement number")
    plt.ylabel("Height")
    plt.show()


# Example 6 - Estimating The Temperature Of The Liquid In A Tank
process_noise = 0.0001
sigma_measurement = 0.1
measurement_uncertainty = sigma_measurement**2
x_0 = 10 # We do not know the temperature of the liquid so we guessed
sigma_human = 100
p_0 = sigma_human**2
label = [49.979, 50.025, 50, 50.003, 49.994, 50.002, 49.999, 50.006, 49.998, 49.991]
measurements = [49.95, 49.967, 50.1, 50.106, 49.992, 49.819, 49.933, 50.007, 50.023, 49.99]
state_estimation = []
state_prediction = [x_0]
variance_estimation = []
variance_prediction = [p_0]

for i in range(10):
    kalman_gain = variance_prediction[-1]/(variance_prediction[-1] + measurement_uncertainty)

    new_state_estimation = (1-kalman_gain)*state_prediction[-1] + kalman_gain*measurements[i]
    new_variance_estimation = (1-kalman_gain)*variance_prediction[-1]

    state_estimation.append(new_state_estimation)
    variance_estimation.append(new_variance_estimation)

    new_state_prediction = new_state_estimation
    new_variance_prediction = new_variance_estimation + process_noise

    state_prediction.append(new_state_prediction)
    variance_prediction.append(new_variance_prediction)

if which_to_plot == 2:
    plt.plot(np.arange(1, 11), measurements, "-b", label="Measurements")
    plt.plot(np.arange(1, 11), state_estimation, "-r", label="Estimates")
    plt.plot(np.arange(1, 11), label, "-g", label="True value")
    plt.legend(loc="upper left")
    plt.xlabel("Measurement number")
    plt.ylabel("Temperature")
    plt.show()


# Example 7/8 - Estimating The Temperature Of A Heating Liquid
# This example expleins the difference between high and low proces noise
# in order to reduce the Lag Error.
# When we increase the proces noise we rely more on the measurement data
# and less on our estimation
process_noise = 0.15 # process_noise = 0.0001 (low) or process_noise = 0.15 (high)
sigma_measurement = 0.1
measurement_uncertainty = sigma_measurement**2
x_0 = 10 # We do not know the temperature of the liquid so we guessed
sigma_human = 100
p_0 = sigma_human**2
label = [50.479, 51.025, 51.5, 52.003, 52.494, 53.002, 53.499, 54.006, 54.498, 54.991]
measurements = [50.45, 50.967, 51.6, 52.106, 52.492, 52.819, 53.433, 54.007, 54.523, 54.99]
state_estimation = []
state_prediction = [x_0]
variance_estimation = []
variance_prediction = [p_0]

for i in range(10):
    kalman_gain = variance_prediction[-1]/(variance_prediction[-1] + measurement_uncertainty)

    new_state_estimation = (1-kalman_gain)*state_prediction[-1] + kalman_gain*measurements[i]
    new_variance_estimation = (1-kalman_gain)*variance_prediction[-1]

    state_estimation.append(new_state_estimation)
    variance_estimation.append(new_variance_estimation)

    new_state_prediction = new_state_estimation
    new_variance_prediction = new_variance_estimation + process_noise

    state_prediction.append(new_state_prediction)
    variance_prediction.append(new_variance_prediction)

if which_to_plot == 3:
    plt.plot(np.arange(1, 11), measurements, "-b", label="Measurements")
    plt.plot(np.arange(1, 11), state_estimation, "-r", label="Estimates")
    plt.plot(np.arange(1, 11), label, "-g", label="True value")
    plt.legend(loc="upper left")
    plt.xlabel("Measurement number")
    plt.ylabel("Temperature")
    plt.show()