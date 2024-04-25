import math
import statistics as st
import random

import matplotlib.pyplot as plt


def montecarlo_simulation_circle():
    # Montecarlo Simulation to obtain the value of pi()

    n = 1000000
    circle = 0

    for i in range(n):

        x = random.uniform(-1,1)
        y = random.uniform(-1,1)

        if (x ** 2 + y ** 2) ** (1/2) < 1:
            circle += 1
            # print(circle)

    print(4 * circle/n)


def rolling_die():
    # Compare Expected Value, and Average Value of rolling a die

    # --- Simulation
    dies = []
    n = 10000
    for i in range(n):
        val = random.uniform(1, 6)
        dies.append(val)
    plt.hist(dies)
    print ('Mean value after %i rolls: %.3f' %(n, st.mean(dies)))

    # --- Expected Value
    n = 6
    p_x = 1/n

    E_x = 0
    for i in range(1, n+1):
        E_x += i*p_x

    print('Expected value: %.3f' %E_x)

    # --- Average
    # Average is a special case of 'expected value' where all the probabilities are equal
    n = 1000
    sum = 0
    for i in range(n):
        val = random.uniform(1,6)
        sum += val

    average = sum / n
    print('Average: %.3f' %average)


    plt.show()


def gaussian_mixture_model():

    # Step 0: Generate the ground truth
    cluster_1 =



if __name__ == '__main__':

    # montecarlo_simulation_circle()
    rolling_die()