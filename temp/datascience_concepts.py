import numpy as np
import matplotlib.pyplot as plt
import random

def kalman_filter_example():

    # --- Create a noisy signal
    size = 1000
    sigma = 0.2
    value = 5
    voltage = 5*np.ones(size)       # Real values
    noise = np.random.normal(value, sigma, size)    # Noisy values
    plt.hist(noise)

    # Turn into list
    noise_list = noise.tolist()

    # State Estimates
    xE = []
    xE.append(0)
    # Error Covariance
    P = []
    P.append(1)

    Q = 1e-5            # Process noise covariance
    R = sigma ** 2      # Measurement noise covariance

    for i in range(1, size):
        # Predict
        error = P[i-1] + Q

        # Correct
        K = error / (error + R)
        prediction = xE[-1] + K*(noise[i]-xE[-1])
        xE.append(prediction)

        new_error = (1-K) * error
        P.append(new_error)

    fig = plt.figure()
    plt.plot(noise, label='Measurement')
    plt.plot(xE, label='Filtered signal')
    plt.plot(voltage, label='Real value')
    plt.legend()
    plt.ylim([4.5, 5.5])
    plt.show()

if __name__ == '__main__':
    kalman_filter_example()