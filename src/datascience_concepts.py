import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.ticker import PercentFormatter
from scipy.stats import binom
# import statistics as st

from sklearn import cluster, datasets, mixture

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


def gmm_example():
    # Method to demonstrate how Gaussian Mixture Models work

    # Step 0: Generate Ground truth data
    cluster_1 = np.random.normal(40, 5, 1000)  # Noisy values
    cluster_2 = np.random.normal(50, 5, 1000)
    plt.hist(cluster_1)
    plt.hist(cluster_2)

    X = np.concatenate((cluster_1, cluster_2)).reshape(-1,1)
    fig = plt.figure()
    plt.hist(X)

    # plt.show()

    # Step 1: SciKit package
    gmm = mixture.GaussianMixture(n_components=2, random_state=0).fit(X)
    means = gmm.means_
    covariances = gmm.covariances_
    weights = gmm.weights_

    fig = plt.figure()
    # Plot individual PDFs
    x = np.linspace(np.min(X), np.max(X), 1000)[:, np.newaxis]
    for i in range(len(means)):
        pdf_values = weights[i] * gaussian_pdf(x, means[i], covariances[i])
        plt.plot(x, pdf_values, label=f'Component {i + 1}')


    plt.xlabel('X')
    plt.ylabel('Probability Density')
    plt.title('Individual PDFs of Gaussian Mixture Model')
    plt.legend()
    plt.show()


def gaussian_pdf(x, mean, cov):
    """
    Calculate the PDF of a Gaussian distribution.

    Parameters:
        x (numpy array): The input values.
        mean (numpy array): The mean of the Gaussian distribution.
        cov (numpy array): The covariance matrix of the Gaussian distribution.

    Returns:
        numpy array: The PDF values corresponding to the input values.
    """
    exponent = -0.5 * np.sum((x - mean) @ np.linalg.inv(cov) * (x - mean), axis=1)
    return np.exp(exponent) / np.sqrt((2 * np.pi) ** len(mean) * np.linalg.det(cov))


def binomial_pdf():

    # --- Parameters ---
    success_rate = 0.95
    sample_size = 10
    samples = 1000
    bins = np.arange(0, sample_size + 2) - 0.5

    plt.ion()

    fig, ax = plt.subplots()
    ax.set_yticklabels(['{:.0f}%'.format(x * 100) for x in plt.gca().get_yticks()])

    binary_results = []
    for i in range(samples):

        ax.clear()

        # Draw a sample
        sample = []
        for j in range(sample_size):

            trial = random.random()
            if trial < success_rate:
                sample.append(1)
            else:
                sample.append(0)

        binary_results.append(sum(sample))

        ax.hist(binary_results, bins=bins, density=True)
        ax.set_title('Animated Sampling of a Binomial Distribution (%i samples)' %i)
        ax.grid(True)

        # Pause to allow the plot to be updated
        plt.pause(0.001)

    plt.ioff()  # Turn off interactive mode after the loop is finished

    # --- Approach 2: Using Scipy ---
    data_binom = binom.rvs(n=sample_size, p=success_rate, size=samples)
    fig=plt.figure()
    plt.hist(data_binom, bins=bins, density=True)
    plt.title('Histogram of Binomial Distribution (using Scipy')
    plt.grid(True)

    # --- APPROACH 3: Using Numpy ---
    # Parameters for the binomial distribution
    fig = plt.figure()

    # Generate binomial distribution data
    data = np.random.binomial(sample_size, success_rate, samples)

    # Plot histogram
    plt.hist(data, bins=bins, density=True, alpha=0.75)

    # Adjust y-axis ticks to percentages
    plt.gca().set_yticklabels(['{:.0f}%'.format(x * 100) for x in plt.gca().get_yticks()])

    # Add labels and title
    plt.xlabel('Number of Successes')
    plt.ylabel('Percentage')
    plt.title('Histogram of Binomial Distribution (using Numpy')
    plt.grid(True)

    plt.show()


def central_limit_theorem():
    # How many samples do you need, before the mean of the means of the samples is the same as the mean of the population

    # Create a population distribution
    population = np.random.normal(100, 60, 1000)  # Noisy values

    # Draw a sample from the population

    means = []
    # Vary the size of the sample
    for i in range(500):

        samples = []
        # Take a few samples
        for j in range(5):
            sample = np.random.choice(population, i)
            samples.append(sample.mean())

        mean_of_samples = st.mean(samples)
        means.append(mean_of_samples)

        print('Sample No %i, Mean: %.2f' %(i, mean_of_samples))

    plt.plot(means)
    plt.grid()
    plt.show()



if __name__ == '__main__':
    # kalman_filter_example()
    gmm_example()
    # binomial_pdf()
    # central_limit_theorem()