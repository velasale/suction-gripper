import numpy as np
import matplotlib.pyplot as plt
import random

def kalman_filter_example():
    # Example of measuring a fixed value variable:
    # e.g. Voltage sensor of a power supply set at a constant value
    # e.g. Altitude sensor of a building with the a constant height

    # --- Create a noisy signal
    size = 1000
    sigma = 0.2
    amplitude = 5
    periods = 1
    # Constant signal
    voltage = amplitude * np.ones(size)       # Real values
    # Sinusoidal signal
    x = np.linspace(-periods * np.pi, periods * np.pi, size)
    voltage = np.add(5, np.sin(x))
    # Measurement Noise
    noise = np.random.normal(0, sigma, size)    # Noisy values
    plt.hist(noise)
    measurement = np.add(voltage, noise)
    # Turn into list
    meas_list = measurement.tolist()

    # State Estimates
    xE = []
    xE.append(0)
    # Error Covariance
    P = []
    P.append(1)
    # Store K
    Ks = []

    Q = 1e-5            # Process noise covariance
    R = sigma ** 2      # Measurement noise covariance
    R = 0.001

    for i in range(1, size):
        # Predict
        error = P[i-1] + Q

        # Correct
        K = error / (error + R)
        Ks.append(K)
        prediction = xE[-1] + K*(meas_list[i]-xE[-1])
        xE.append(prediction)

        new_error = (1-K) * error
        P.append(new_error)

    fig = plt.figure()
    plt.plot(meas_list, label='Measurement', marker=".", color='r', linestyle='None', markersize = 2.0)
    plt.plot(xE, label='Filtered signal')
    plt.plot(voltage, label='Real value')
    plt.plot(Ks, label='Kalman Gain')
    plt.legend()
    # plt.ylim([3, 6])
    plt.show()

if __name__ == '__main__':
    kalman_filter_example()