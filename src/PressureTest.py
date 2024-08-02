import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter
import scipy as scipy

# error function (the basis for the loop that will be created)
def olivia_test(gp1, gp2, gp3):

    def vector_math(pressure_array):
        uA = np.array([np.cos(np.pi/3),np.sin(np.pi/3),0.])  #suction cup 1
        uB = np.array([0, 1., 0])  #suction cup 2
        uC = np.array([np.cos(-np.pi/3), np.sin(-np.pi/3), 0])  #suction cup 3

        vA = pressure_array[0]*uA
        vB = pressure_array[1]*uB
        vC = pressure_array[2]*uC

        v_sum = vA + vB + vC

        return v_sum
    
    def axis_of_rotation(v_sum):
        axis = np.array([1.,1.,0.])
        axis -= axis.dot(v_sum) * v_sum / np.linalg.norm(v_sum)**2
        axis /= np.linalg.norm(axis)   #gripper frame

        return axis

    def angle_math(v_sum):
        theta = np.arctan2([v_sum[1]], [v_sum[0]])
        return theta[0]

    def speed_calc(v_sum):
        mag = np.linalg.norm(v_sum)  # units = hPa
        # print(mag)
        k = 1 # (deg/sec)/kPa    - this is so that if the error is 80 kPa the speed is 90 deg/sec
        speed = (mag / 10) * k
        # print(speed)
        return speed

    pressure = [gp1, gp2, gp3]

    neg_error1 = pressure[1] - 223 #(0.50 * pressure[1])
    neg_error2 = pressure[2] - 223 #(0.50 * pressure[2])
    pos_error1 = pressure[1] + 223 #(0.50 * pressure[1])
    pos_error2 = pressure[2] + 223 #(0.50 * pressure[2])

    if pressure[0] > 400 or pressure[1] > 400 or pressure[2] > 400:
        v_sum = vector_math(pressure)
        axis = axis_of_rotation(v_sum)
        angle = angle_math(v_sum)
        mag = np.linalg.norm(v_sum)
        speed = speed_calc(v_sum)


    elif pressure[0] <= 400 and pressure[1] <= 400 and pressure[2] <= 400:
        if neg_error1 <= pressure[0] <= pos_error1:
            if neg_error2 <= pressure[0] <= pos_error2:
                if neg_error1 <= pressure[2] <= pos_error1:
                    print(f'All 3 suction cups engaged!')
                    angle = 0
                    mag = 0
                        
                else:
                    v_sum = vector_math(pressure)
                    axis = axis_of_rotation(v_sum)
                    angle = angle_math(v_sum)
                    mag = np.linalg.norm(v_sum)
                    speed = speed_calc(v_sum)
                    

            else: 
                v_sum = vector_math(pressure)
                axis = axis_of_rotation(v_sum)
                angle = angle_math(v_sum)
                mag = np.linalg.norm(v_sum)
                speed = speed_calc(v_sum)

        else:
            v_sum = vector_math(pressure)
            axis = axis_of_rotation(v_sum)
            angle = angle_math(v_sum)
            mag = np.linalg.norm(v_sum)
            speed = speed_calc(v_sum)

    return mag, angle
    

#figure out how to finish this error loop and then youre done    