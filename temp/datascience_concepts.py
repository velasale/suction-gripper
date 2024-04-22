import numpy as np
import matplotlib.pyplot as plt
import random

def kalman_filter_example():
    # Example of measuring a fixed value variable:
    # e.g. Voltage sensor of a power supply set at a constant value
    # e.g. Altitude sensor of a building with the a constant height

    # --- Create a noisy signal
    size = 100
    sigma = 0.2
    amplitude = 0.4
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
    P.append(1)         # Initial error covariance
    # Store K
    Ks = []

    # --- PARAMETERS
    Q = 1e-5            # Process noise covariance
    R = sigma ** 2      # Measurement noise covariance
    R = 0.0001
    A = 1               # The signal is a constant value
    H = 1               # Becuase our measurement is of the state itself.

    for i in range(1, size):
        # --- Time Update: PREDICTION ---
        prediction_update = xE[-1]
        error_update = P[i-1] + Q

        # ---- Measurement Update: CORRECTION ---
        K = error_update / (error_update + R)
        Ks.append(K)
        prediction = prediction_update + K*(meas_list[i]-prediction_update)
        xE.append(prediction)
        error = (1-K) * error_update
        P.append(error)

    fig = plt.figure()
    plt.plot(meas_list, label='Measurement', marker=".", color='r', linestyle='None', markersize = 2.0)
    plt.plot(xE, label='Filtered signal')
    plt.plot(voltage, label='Ground Truth', color='k', linestyle='--')
    plt.plot(Ks, label='Kalman Gain')
    plt.legend()
    # plt.ylim([3, 6])
    plt.show()

if __name__ == '__main__':
    kalman_filter_example()