# @Time : 6/6/2023 11:15 AM
# @Author : Alejandro Velasquez

######## Standard Library Imports ########
import json
import math
import os
import re
import csv
import time
import numpy as np

import scipy
from scipy.ndimage import gaussian_filter, median_filter
import scipy.stats as st
from scipy.stats import norm


######## 3rd parties Imports ########
from operator import sub, add
# from cv_bridge import CvBridge
import cv2
import itertools

import bagpy
from matplotlib import pyplot as plt

from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from bagpy import bagreader

from sklearn.metrics import r2_score
from sklearn.cluster import KMeans
from sklearn.linear_model import LinearRegression
import pyautogui

from tqdm import tqdm

######## Self developed imports ########
# from ros_scripts import *
from plot_scripts import *

import logging
logging_format = "[%(asctime)s - %(levelname)s] %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")


# ------------------------ HANDY FUNCTIONS -----------------------------------------
def point_to_line_distance(point, origin, vector):
    """

    @param point: Point from which to measure the distance to the line
    @param origin: Origin where the direction vector
    @param vector: Direction vector
    @return: distance
    """

    P = np.array(point)
    Q = np.array(origin)
    PQ = np.subtract(P, Q)

    eef_z_vector_transpose = np.transpose(vector)
    eef_z_vector_3dim = np.array([[eef_z_vector_transpose[0, 0], eef_z_vector_transpose[0, 1], eef_z_vector_transpose[0, 2]]])

    # Distance between a line and a point:
    # Source: https://math.libretexts.org/Bookshelves/Calculus/Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.05%3A_Equations_of_Lines_and_Planes_in_Space
    num = np.cross(PQ, eef_z_vector_3dim)
    num = np.linalg.norm(num)
    den = np.linalg.norm(eef_z_vector_3dim)

    distance = 1000 * num / den  # convert to mm
    distance = round(distance, 2)

    return distance


def locate_index_of_deltas(data, width=200, change=3/10000):
    """
    Useful to locate 1st derivative changes in a time series.
    CAUTION: THIS WORKS ONLY FOR A SINGLE BACK AND FORTH MOVE WHICH LOOK
    @param data:
    @param width: distance between indexes to measure the delta
    @param change: change that wants to be located
    @return: begin and end of the deltas
    """

    POIS_plus = []
    POIS_minus = []
    POIS = []

    previous = data[0]

    for i in range(len(data) - width):
        delta = data[i+width] - previous
        previous = data[i]
        if delta > change:
            POIS_plus.append(i)     # collects all points with positive changes
        if delta < - change:
            POIS_minus.append(i)    # collects all points with negative changes

    # Only store first and last points
    POIS.append(POIS_minus[0])
    POIS.append(POIS_minus[-1] + width)
    POIS.append(POIS_plus[0])
    POIS.append(POIS_plus[-1] + width)

    logging.debug('POIs: %i %i %i %i', POIS[0], POIS[1], POIS[2], POIS[3])
    POIS.sort()
    logging.debug('2nd and 3rd POI: %i %i', POIS[2], POIS[3])
    idx_1 = POIS[2]
    idx_2 = POIS[3]

    return idx_1, idx_2


def locate_index_of_deltas_v2(data, intercept=0.5):
    """
    Useful to locate in square signals, the index where the signal intersects a certain value
    """

    POIS_plus = []
    POIS_minus = []
    POIS = []

    previous = data[0]

    for i in range(len(data)):

        if (previous < intercept) and (data[i] > intercept):
            POIS_plus.append(i)     # collects all points with positive changes
        if (previous > intercept) and (data[i] < intercept):
            POIS_minus.append(i)    # collects all points with negative changes

        previous = data[i]


    print('\nPOIs: ', POIS_plus, POIS_minus,'\n')

    return POIS_plus, POIS_minus


def list_to_hist(list, x_label, plot_norm=True):
    """Generates histogram out of a list, and provides k-means clusters
    CAUTION: Make sure your has clusters otherwise you don't need this
    @param list:
    @param x_label:
    @param plot_norm: adds a plot of the normal distribution fit
    @return: none
    """

    if len(list) > 0:

        # Step 1: Convert data into a numpy array
        values = np.array(list)
        kmeans = KMeans(n_clusters=3, random_state=0).fit(values.reshape(-1,1))
        clusters = kmeans.cluster_centers_

        # Step 2: Get some features from the data
        median = round(np.median(list), 2)

        # Step 3: PLot histogram
        fig = plt.figure()
        plt.title('Sample size: ' + str(len(list)) + ', Median: ' + str(median))
        plt.xlabel(x_label)


        # Step 4: Fit a normal distribution
        # Source: https://stackoverflow.com/questions/20011122/fitting-a-normal-distribution-to-1d-data
        if plot_norm == True:

            plt.hist(values, density=True)  # density True for normalized curves
            mu, std = norm.fit(values)
            xmin, xmax = plt.xlim()
            x = np.linspace(xmin, xmax, 100)
            p = norm.pdf(x, mu, std)
            plt.plot(x, p, 'k', linewidth=2)
        else:
            plt.hist(values)

    else:
        print('no data')


def relative_values(original_list, reference_value):
    "Subtracts a value from all the elements of a list. Handy for converting time stamps"

    relative_list = [None] * len(original_list)
    for i in range(len(original_list)):
        relative_list[i] = original_list[i] - reference_value

    return relative_list


# ------------------------ FORWARD KINEMATICS FUNCTIONS -------------------------------
def DH_T(i, thetas, alphas, a_links, d_offsets):
    """
    Forward Kinematics Denavit Hartenberg transformations
    @param i: joint
    @param thetas:
    @param alphas:
    @param a_links:
    @param d_offsets:
    @return:
    """

    cos_t = math.cos(thetas[i])
    sin_t = math.sin(thetas[i])
    cos_a = math.cos(alphas[i])
    sin_a = math.sin(alphas[i])
    a = a_links[i]
    d = d_offsets[i]

    # DH Homogeneous Transformation Matrix
    T = np.array([[cos_t, -sin_t*cos_a, sin_t*sin_a, a*cos_t],
                  [sin_t, cos_t*cos_a, - cos_t*sin_a, a*sin_t],
                  [0, sin_a, cos_a, d],
                  [0, 0, 0, 1]])

    return T


def ur5e_fk_dh(joint_angles):
    """ Forward kinematics with the Denavit-Hartemberg approach"""
    # Source: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

    # STEP1: UR5e DH parameters
    # Note: Parameters can also be found in 'default_kinematics.yaml'
    # -- Link twists [rad] --
    alphas = np.array([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0])
    # -- Link lengths [m] --
    a_links = np.array([0, -0.425, -0.3922, 0, 0, 0])
    # -- Link offsets [m] --
    d_offsets = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996])

    # STEP2: Obtain all the transformation matrices
    T = np.identity(4)
    for i in range(len(joint_angles)):
        T = np.dot(T, DH_T(i, joint_angles, alphas, a_links, d_offsets))

    # WARNING XY AXES OF 'BASE LINK' CFRAME ARE ROTATED 180 FROM 'BASE' CFRAME.
    # SINCE I DID IN ROS EVERYTHING RELATED TO 'BASELINK' AND THE 'DH' ARE W.R.T 'BASE'
    # I SIMPLY MULTIPLY T BY AN ADDITIONAL ROTATION OF 180 IN Z.

    theta = math.pi
    rot_Mat = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                      [math.sin(theta), math.cos(theta), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
    new_T = np.dot(rot_Mat, T)

    # STEP3: Get the euler angles
    beta = math.atan2(-new_T[2, 0], math.sqrt(new_T[0, 0] ** 2 + new_T[1, 0]** 2))
    alpha = math.atan2(new_T[1, 0]/math.cos(beta), new_T[0, 0]/math.cos(beta))
    gamma = math.atan2(new_T[2, 1]/math.cos(beta), new_T[2, 2]/math.cos(beta))

    # STEP4: Extract the 3d position and orientation
    eef_xyz = new_T[:, 3]
    eef_angles = [beta, alpha, gamma]

    return eef_xyz, eef_angles


# ----------------------- FILE HANDLING FUNCTIONS ------------------------------------
def read_json(file):
    """Creates a list of experiments as objects. It then reads their respective json file and adds the metadata as
    attributes to each one of them"""

    if file.endswith(".json"):
        json_file = open(file)
        json_data = json.load(json_file)

        # Create Experiment as Object
        experiment = Experiment()

        # Add metadata as attributes
        experiment.file_source = file
        try:
            experiment.exp_type = json_data["generalInfo"]["experimentType"]
        except KeyError:
            experiment.exp_type = ""

        try:
            experiment.repetition = json_data["generalInfo"]["repetition"]
        except KeyError:
            experiment.repetition = ""

        try:
            experiment.surface = json_data["surfaceInfo"]["type"]
        except KeyError:
            experiment.surface = ""


        try:
            experiment.x_noise = abs(json_data["robotInfo"]["x noise real [m]"])
        except KeyError:
            experiment.x_noise = 0

        try:
            experiment.z_noise = abs(json_data["robotInfo"]["z noise real [m]"])
        except KeyError:
            experiment.z_noise = 0

        try:
            experiment.x_noise_command = abs(json_data["robotInfo"]["x noise command [m]"])
        except KeyError:
            experiment.x_noise_command = 0

        try:
            experiment.z_noise_command = abs(json_data["robotInfo"]["z noise command [m]"])
        except KeyError:
            experiment.z_noise_command = 0

        try:
            experiment.pitch = json_data["robotInfo"]["pitch [rad]"]
        except KeyError:
            experiment.pitch = 0


        try:
            experiment.pressure = json_data["gripperInfo"]["pressureAtValve [PSI]"]
        except KeyError:
            # experiment.pressure = json_data["gripperInfo"]["pressureAtValve"]
            experiment.pressure = ""

        try:
            experiment.surface_radius = json_data["surfaceInfo"]["radius [m]"]
        except KeyError:
            # experiment.surface_radius = json_data["surfaceInfo"]["radius"]
            experiment.surface_radius = ""



        # print(experiment.exp_type)

    return experiment


def find_file(type, radius, pressure, noise, rep, pitch, surface):
    """"""

    # A. Build the name of the experiment
    location = os.path.dirname(os.getcwd())

    # location = '/media/alejo/DATA'
    # '/home/alejo/Documents/data/DATASET4'
    location = "/home/alejo/Documents"
    # location = '/home/alejo/gripper_ws/src/suction-experiment'
    # location = '/media/alejo/042ba298-5d73-45b6-a7ec-e4419f0e790b/home/avl'
    location = '/home/alejo/Documents'

    if type == "horizontal":
        # folder = "/data/DATASET2/x_noise/rep" + str(rep + 1) + "/"
        folder = "/data/DATASET4AND5/"
        filename = "horizontal_#" + str(noise) + \
                   "_pres_" + str(pressure) + \
                   "_surface_3DPrintedPrimer" + \
                   "_radius_" + str(round(radius, 4))
    elif type == "vertical":
        folder = "/data/DATASET2/z_noise/rep" + str(rep + 1) + "/"
        filename = "vertical_#" + str(noise) + \
                   "_pres_" + str(pressure) + \
                   "_surface_3DPrintedPrimer85" + \
                   "_radius_" + str(radius)
    elif type == "simple_suction":
        folder = "/data/pressure_check/"
        filename = "simple_suction_#" + str(rep) +\
                    "_pres_" + str(pressure) +\
                    "_surface_" + surface +\
                    "_radius_0.0375"

    file_path = location + folder
    # print("\nSearching for:", filename)

    # B. Look for the file
    if type == "simple_suction":
        for f in os.listdir(file_path):
            # if re.match(filename, f) and f.endswith(".bag"):
            if re.match(filename, f) and f.endswith(".json"):
                only_filename = f.split(".json")[0]
                break
            else:
                only_filename = "no_match"
    else:
        for f in os.listdir(file_path):
            # if re.match(filename, f) and f.endswith(".bag"):
            if re.match(filename, f) and f.endswith("pitch_" + str(pitch) + "_rep_" + str(rep) + ".json"):
                only_filename = f.split(".json")[0]
                break
            else:
                only_filename = "no_match"
    # print(only_filename)

    if only_filename == "no_match":
        file = "no_match"
        print("\n", (rep+1))
        print("Couldn't find :", (filename + "  " + str(pitch) + "_rep_" + str(rep) + ".json"))
    else:
        file = location + folder + only_filename

    return file, only_filename


def read_csvs(experiment, folder):
    """Opens the csvs associated to each experiment and saves it as lists"""

    # Sweep the csvs of each experiment
    for file in os.listdir(folder):
        if file.endswith('.csv'):
            data_list = pd.read_csv(folder + "/" + file)

            if file == "gripper-pressure.csv":
                experiment.pressure_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.pressure_values = data_list.iloc[:, 1].tolist()
                # Convert to kPa (1000 Pa) which is more standard than hPa (100 Pa)
                experiment.pressure_values = np.divide(experiment.pressure_values, 10)

            if file == "gripper-pressure-sc1.csv":
                experiment.pressure_sc1_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.pressure_sc1_values = data_list.iloc[:, 1].tolist()
                # Convert to kPa (1000 Pa) which is more standard than hPa (100 Pa)
                experiment.pressure_sc1_values = np.divide(experiment.pressure_sc1_values, 10)

            if file == "gripper-pressure-sc2.csv":
                experiment.pressure_sc2_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.pressure_sc2_values = data_list.iloc[:, 1].tolist()
                # Convert to kPa (1000 Pa) which is more standard than hPa (100 Pa)
                experiment.pressure_sc2_values = np.divide(experiment.pressure_sc2_values, 10)

            if file == "gripper-pressure-sc3.csv":
                experiment.pressure_sc3_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.pressure_sc3_values = data_list.iloc[:, 1].tolist()
                # Convert to kPa (1000 Pa) which is more standard than hPa (100 Pa)
                experiment.pressure_sc3_values = np.divide(experiment.pressure_sc3_values, 10)

            if file == "rench.csv":
                experiment.wrench_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.wrench_xforce_values = data_list.iloc[:, 5].tolist()
                experiment.wrench_yforce_values = data_list.iloc[:, 6].tolist()
                experiment.wrench_zforce_values = data_list.iloc[:, 7].tolist()
                experiment.wrench_xtorque_values = data_list.iloc[:, 8].tolist()
                experiment.wrench_ytorque_values = data_list.iloc[:, 9].tolist()
                experiment.wrench_ztorque_values = data_list.iloc[:, 10].tolist()

            if file == "xperiment_steps.csv":
                experiment.event_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.event_values = data_list.iloc[:, 1].tolist()

            if file == "oint_states.csv":
                experiment.joint_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.j0_shoulder_pan_values = data_list.iloc[:, 8].tolist()
                experiment.j1_shoulder_lift_values = data_list.iloc[:, 7].tolist()
                experiment.j2_elbow_values = data_list.iloc[:, 6].tolist()
                experiment.j3_wrist1_values = data_list.iloc[:, 9].tolist()
                experiment.j4_wrist2_values = data_list.iloc[:, 10].tolist()
                experiment.j5_wrist3_values = data_list.iloc[:, 11].tolist()

            if file == "gripper-distance.csv":
                experiment.tof_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.tof_values = data_list.iloc[:, 1].tolist()

    return experiment


# ----------------------- PLOT RELATED FUNCTIONS -------------------------------------
def suction_plots(var, type, x_noises, z_noises, mean_values, std_values, pressure, pitch, radius, trends='false'):

    FONTSIZE = 24
    TICKSIZE = 24

    # convert x and z noises from m to mm
    for i in range(len(x_noises)):
        x_noises[i] = x_noises[i] * 1000

    if type == "horizontal":
        # print("x_noises: ", x_noises)
        # print("y_values: ", mean_values)

        # Comment next line to remove the std dev lines from the
        # std_values = 0

        # plt.errorbar(x_noises, mean_values, std_values, label=(str(pressure) + " PSI, " + str(pitch) + '$^\circ$'))
        plt.errorbar(x_noises, mean_values, std_values, label=(str(pitch) + '$^\circ$'))

        # Trendline
        if trends == 'true':
            z = np.polyfit(x_noises, mean_values, 4)
            p = np.poly1d(z)
            plt.plot(x_noises, p(x_noises), linestyle='dashed', color='black')
            print(r2_score(mean_values, p(x_noises)))
            plt.annotate("r-squared = {:.3f}".format(r2_score(mean_values, p(x_noises))), (0.01, -200))

        # Trial of fill between
        # plt.plot(noises_xnoises, noises_vacuum_means)
        # plt.fill_between(noises_xnoises, list(map(sub, noises_vacuum_means, noises_vacuum_stds)), list(map(add,noises_vacuum_means, noises_vacuum_stds)), alpha=.5)

        title = "Cartesian noise in x, for %.2f mm diameter" % (2000*radius)
        plt.xlabel("x-noise [mm]", fontsize=FONTSIZE)

    elif type == "vertical":
        plt.errorbar(z_noises, mean_values, std_values, label=(str(pressure) + " PSI"))

        # Trendline
        if trends == 'true':
            z = np.polyfit(z_noises, mean_values, 4)
            p = np.poly1d(z)
            plt.plot(z_noises, p(z_noises), linestyle='dashed', color='black')
            plt.annotate("r-squared = {:.3f}".format(r2_score(mean_values, p(z_noises))),(0.01, -200))

        title = "Cartesian noise in z, for %.2f mm diameter" % (2000*radius)
        plt.xlabel("z-noise [m]", fontsize=FONTSIZE)

    if var == 'pressure':
        plt.ylabel("Vacuum [kPa]", fontsize=FONTSIZE)
        plt.ylim([0, 110])
        # plt.ylim([-100, 0])
    elif var == 'force' or var == 'zforce':
        plt.ylabel("Force z [N]", fontsize=FONTSIZE)
        plt.ylim([0, 6.5])
    elif var =='torque':
        plt.ylabel("Torque [Nm]")
        plt.ylim([0, 0.5])

    # plt.xlim([0, 35])
    plt.xlim([0, 45])

    plt.xticks(size=TICKSIZE, fontsize=FONTSIZE)
    plt.yticks(size=TICKSIZE, fontsize=FONTSIZE)

    plt.legend(fontsize=FONTSIZE)
    # plt.grid()
    # plt.title(title)


def circle_plots(x_noises, z_noises, radius, x_forces, z_forces, pressure):

    # x_noises = [0,5,10,15,20,25,30, 35]
    z_noises = []

    # --- Plot semi-circle
    # radius = 75 / 2
    x_axis = []
    z_axis = []
    x = 0
    dpi = 1000
    while x < radius:
        z = (radius ** 2 - x ** 2) ** 0.5
        x_axis.append(x)
        z_axis.append(z)
        x += 0.001
    plt.plot(x_axis, z_axis, color='red')

    # --- Plot dots at each sampling point
    max_zforce = max(z_forces)
    max_xforce = max(x_forces)
    for x, fx, fz in zip(x_noises, x_forces, z_forces):
        z = (radius ** 2 - x ** 2) ** 0.5

        dx = 0.005 * fx / max_xforce
        dz = 0.005 * fz / max_zforce

        # --- Plot arrows in the direction of Z and X, have the length scaled with the magnitude
        plt.arrow(x, z, dx=0, dy=dz, width=0.00025, color='blue')
        plt.arrow(x, z, dx=dx, dy=0, width=0.00025, color='red')

        plt.plot(x, z, marker="o", markersize=5, markeredgecolor="black", markerfacecolor="black")

    # # Plot arrows in the direction of N and Tan, with length proportional to the magnitude
    # for x, fn, ft in zip(x_noises, n:
    #     z = (radius ** 2 - x ** 2) ** 0.5
    #     theta = math.acos(x/radius)
    #
    #     # --- Plot arrows in the direction of Z and X, have the length scaled with the magnitude
    #     d = 2/1000
    #     dx = d * math.cos(theta)
    #     dy = d * math.sin(theta)
    #     plt.arrow(x, z, dx, dy, width=0.00025, color='orange')
    #     plt.arrow(x, z, dy, -dx, width=0.00025, color='gray')
    #
    #     plt.plot(x, z, marker="o", markersize=5, markeredgecolor="black", markerfacecolor="black")

    plt.axis('equal')
    plt.title('Diameter: %.2f mm and Feeding Pressure: %.2f PSI' % (2000 * radius, pressure))
    plt.grid()
    plt.xlabel('x-axis [mm]')
    plt.ylabel('z-axis [mm]')
    plt.show()


def plot_and_video():
    """Method to run a vertical line on a plot and a video"""

    # --- Provide File ---

    # --- Default Hard Drive ---
    # folder = '/home/alejo/gripper_ws/src/suction-gripper/data/'
    # --- Hard Drive B ---
    folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"
    # --- Hard Drive C ---
    folder = '/media/alejo/042ba298-5d73-45b6-a7ec-e4419f0e790b/home/avl/data/REAL_APPLE_PICKS/'

    location = '/media/alejo/042ba298-5d73-45b6-a7ec-e4419f0e790b/home/avl/data/DATASET5/'
    filename = 'horizontal_#7_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_26.46_pitch_45.0_rep_1'
    # Sample with 0deg tilt and 0 mm offset:
    filename = 'horizontal_#0_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_0.0_pitch_0.0_rep_1'
    # Sample with 0deg tilt and 18.9mm offset:
    filename = 'horizontal_#5_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_18.9_pitch_0.0_rep_1'
    # Sample with 15deg tilt and 18.9mm offset
    filename = 'horizontal_#5_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_18.9_pitch_15.0_rep_3'
    # Sample with 30deg tilt and 18.9mm offset
    filename = 'horizontal_#5_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_18.9_pitch_45.0_rep_1'
    # Sample with 45deg tilt and 26.5mm offsrt
    # filename = 'horizontal_#7_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_26.46_pitch_45.0_rep_1'

    # --- ICRA24 accompanying video
    # subfolder = "LOW_STIFFNESS/"
    subfolder = ''
    location = folder + subfolder
    filename = '20230731_proxy_sample_6_yaw_45_rep_0_stiff_low_force_low'
    filename = '20230922_realapple3_attempt_1_orientation_0_yaw_0'

    filename = '20230922_realapple1_attempt_1_orientation_0_yaw_0'
    filename = '20230922_realapple2_attempt_1_orientation_0_yaw_0'

    # --- Prosser2
    location = '/media/alejo/Elements/Prosser_Data/Dataset - apple picks/'
    filename = '2023111_realapple24_mode_dual_attempt_1_orientation_0_yaw_0'

    # --- 3. Create Experiment Object
    experiment = Experiment()

    # --- 4. Assign json dictionary as property of the experiment
    json_file = open(location + filename + '.json')
    json_data = json.load(json_file)
    experiment.metadata = json_data
    print(experiment.metadata['general'])

    # --- 4. Read values from 'csv'
    read_csvs(experiment, location + filename)

    # --- 5. Get different features for the experiment
    experiment.get_features()

    # 6. Plot each experiment if needed
    running_plot_and_video(location, filename, experiment)
    plt.show()


def mark10_plots(location, tags, gripper_modes, variable_list, reps, xlabel):

    # Axes limits
    xmin = min(variable_list)
    xmax = max(variable_list)
    colors = ['blue', 'green', 'red']

    # Experiment number
    exp_number = location.split('/experiment')[1]
    exp_number = int(exp_number.split('_')[0])
    print('Experiment number: ', exp_number)

    # Lists to keep track of stuff
    stepses = []
    good_picks = []
    suction_picks = []

    # Create figure
    fig = plt.figure()
    exp_prefix = 'loremipsum'

    for mode, tag, color in zip(gripper_modes, tags, colors):
        print('\nMode: ', mode)
        stepses = []
        mean_max_forces = []
        sdv_max_forces = []

        for steps in variable_list:

            max_forces = []

            # Gather data from all repetitions of the same trial combination
            for rep in range(1, reps+1):

                # exp_prefix = 'delta_' + str(steps) + '_' + (tag) + '_rep' + str(rep)
                # exp_prefix = tag + '_dist_' + str(steps) + '_rep' + str(rep)

                if exp_number == 8:
                    exp_prefix = 'exp(pullBack)_mode(' + tag + ')_dist(58)_speed(' + str(steps) + ')_rep' + str(rep)

                if exp_number == 7:
                    exp_prefix = 'exp(pullBack)_mode(' + tag + ')_distance(' + str(steps) + ')_rep' + str(rep)

                if exp_number == 9:
                    if steps < 10:
                        exp_prefix = 'exp(pullBack)_mode(' + tag + ')_offset(0' + str(steps) + ')_rep' + str(rep)
                    else:
                        exp_prefix = 'exp(pullBack)_mode(' + tag + ')_offset(' + str(steps) + ')_rep' + str(rep)

                if exp_number == 10:
                    exp_prefix = 'exp(pullBack)_mode(' + tag + ')_angle(' + str(steps) + ')_rep' + str(rep)

                max_pull = 'Nan'

                for file in sorted(os.listdir(location)):

                    if file.startswith(exp_prefix):
                        print('\nFile: ', file)

                        # Step 3: Open file and turn into dataframe
                        trial_df = pd.read_excel(location + file, index_col=0)

                        # # Plot time series
                        # plt.plot(trial_df['Travel [mm]'], trial_df['Load [N]'])
                        # plt.title(file)
                        # plt.grid()
                        # plt.show()

                        max_pull = abs(min(trial_df['Load [N]']))
                        # print('Max pull force: ', max_pull)
                        max_forces.append(max_pull)

                        # Keep track of successful pick values
                        if file.endswith('apple_picked).xlsx'):
                            good_picks.append(max_pull)

                        if tag == 'V' or tag == 'suction':
                            suction_picks.append(max_pull)

            print(max_forces)
            stepses.append(steps)
            mean_max_forces.append(abs(np.mean(max_forces)))
            sdv_max_forces.append(abs(np.std(max_forces)))

        print('\n', mode)
        print(stepses)
        print(mean_max_forces)

        # Plot each mode series with a band gap
        if tag == 'suction' or tag == 'V':

            print('Suction Pick Forces:', suction_picks)
            mean_suction_force = np.mean(suction_picks)
            sdv_suction_force = np.std(suction_picks)
            lows = np.subtract(mean_suction_force, sdv_suction_force)
            highs = np.add(mean_suction_force, sdv_suction_force)
            plt.hlines(y=mean_suction_force, xmin=xmin, xmax=xmax, linestyles='-', lw=1,
                       label='Suction cup force [N]')
            plt.fill_between(variable_list, lows, highs, alpha=.2)

        else:
            if len(max_forces) > 0:
                lows = np.add(mean_max_forces, sdv_max_forces)
                highs = np.subtract(mean_max_forces, sdv_max_forces)
                plt.plot(stepses, mean_max_forces, 'o-', label=mode, color=color)
                plt.fill_between(stepses, lows, highs, alpha=.2, color=color)

    # Note: Just for Experiment 2 -- Plot the force at which the magnet releases
    if len(good_picks) > 0:
        mean_pick_force = np.mean(good_picks)
        sdv_pick_force = np.std(good_picks)
        lows = np.subtract(mean_pick_force, sdv_pick_force)
        highs = np.add(mean_pick_force, sdv_pick_force)
        plt.hlines(y=mean_pick_force, xmin=xmin, xmax=xmax, linestyles='--', lw=1,
                   label='Magnet release force [N]', color='red')
        plt.fill_between(stepses, lows, highs, color='red', alpha=.2)

    # Plot median detachment force
    plt.hlines(y=16, xmin=xmin, xmax=xmax, linestyles='--', lw=2, label='Median Detachment Force', color='k')
    # Plot suction force
    plt.hlines(y=12, xmin=xmin, xmax=xmax, linestyles='--', lw=2, label='Average Suction Force')

    plt.grid()
    plt.ylim([10, 15])
    plt.legend()
    plt.xlabel(xlabel)
    plt.ylabel('Force [N]')
    plt.title('Gripper pulling force [N] vs ' + xlabel)


# ----------------------- MAIN CLASS FOR EXPERIMENTS ------------------------------------
class Experiment:
    """Class to define an Experiment as an Object. Each experiment has properties from its json file.
    """
    def __init__(self, metadata,
                 id=0, apple_id=0,
                 pressure=60,
                 surface="3DPrinted_with_Primer",
                 radius=37.5,
                 z_noise=0,
                 x_noise=0,
                 file_source="",
                 vacuum_type='absolute'):

        self.id = id
        self.vacuum_type = vacuum_type

        # ------------------------- Data from jsons ---------------------------
        self.exp_type = metadata['general']['experiment type']
        self.pressure = pressure
        self.surface = surface
        self.surface_radius = radius
        self.z_noise_command = 0
        self.x_noise_command = 0
        self.z_noise = z_noise
        self.x_noise = x_noise
        self.time_stamp = []
        self.pressure_values = []
        self.file_source = file_source
        self.filename = ""
        self.pitch = 0
        self.repetition = 0

        self.metadata = metadata

        # ------------------------ Data from csvs -----------------------------
        # Topic: Gripper's Pressure Sensors
        self.pressure_time_stamp = []
        self.pressure_elapsed_time = []
        self.pressure_values = []

        self.pressure_sc1_time_stamp = []
        self.pressure_sc1_elapsed_time = []
        self.pressure_sc1_values = []

        self.pressure_sc2_time_stamp = []
        self.pressure_sc2_elapsed_time = []
        self.pressure_sc2_values = []

        self.pressure_sc3_time_stamp = []
        self.pressure_sc3_elapsed_time = []
        self.pressure_sc3_values = []

        self.sc1_value_at_engagement = 0.00
        self.sc2_value_at_engagement = 0.00
        self.sc3_value_at_engagement = 0.00

        # Topic: Gripper's ToF Sensor
        self.tof_time_stamp = []
        self.tof_elapsed_time = []
        self.tof_values = []

        # Topic: UR5e's Wrench
        self.wrench_time_stamp = []
        self.wrench_elapsed_time = []

        self.wrench_xforce_values = []
        self.wrench_xforce_relative_values = []
        self.wrench_yforce_values = []
        self.wrench_yforce_relative_values = []
        self.wrench_zforce_values = []
        self.wrench_zforce_relative_values = []
        self.wrench_netforce_relative_values = []

        self.wrench_xtorque_values = []
        self.wrench_xtorque_relative_values = []
        self.wrench_ytorque_values = []
        self.wrench_ytorque_relative_values = []
        self.wrench_ztorque_values = []
        self.wrench_ztorque_relative_values = []

        # Topic: UR5e's Joint States
        self.joint_time_stamp = []
        self.joint_elapsed_time = []
        self.j0_shoulder_pan_values = []
        self.j1_shoulder_lift_values = []
        self.j2_elbow_values = []
        self.j3_wrist1_values = []
        self.j4_wrist2_values = []
        self.j5_wrist3_values = []

        # Topic: Experiment Steps
        self.event_time_stamp = []
        self.event_elapsed_time = []
        self.event_values = []

        self.first_time_stamp = 0
        self.atmospheric_pressure = 0
        self.errors = []

        # ---------------- Features from Data (Statistics and Math) ----------------
        self.steady_pressure_values = []
        self.steady_vacuum_mean = 0
        self.steady_vacuum_std = 0

        self.max_detach_xforce = 0
        self.max_detach_xforce_time = 0
        self.max_detach_yforce = 0
        self.max_detach_yforce_time = 0
        self.max_detach_zforce = 0
        self.max_detach_zforce_time = 0
        self.max_detach_sumforce = 0
        self.max_detach_sumforce_time = 0

        self.max_detach_ytorque = 0
        self.max_detach_ytorque_time = 0

        self.max_detach_nforce = 0          # normal force
        self.max_detach_nforce_time = 0
        self.max_detach_tforce = 0          # tangential force
        self.max_detach_tforce_time = 0

        self.normal_angle = 0
        self.normal_force_values = []
        self.tangent_force_values = []

        self.eef_x = []
        self.eef_y = []
        self.eef_z = []
        self.eef_beta = []
        self.eef_alpha = []
        self.eef_gamma = []
        self.eef_travel = []
        self.offset_eef_apple = []
        self.stiffness = 0

        self.wrench_idx_start_pick = 0
        self.wrench_idx_end_pick = 0
        self.wrench_idx_maxForce_at_pick = 0
        self.max_normalForce_at_pick = 0
        self.max_tangentialForce_at_pick = 0
        self.max_netForce_at_pick = 0
        self.theta_at_pick = 0
        self.travel_at_pick = 0

        # --------------- Apple properties
        self.apple_id = apple_id
        self.apple_diameter = self.metadata['proxy']['apple diameter']
        self.apple_height = self.metadata['proxy']['apple height']
        self.apple_center_loc = [0, 0, 0]

    # --- Functions and methods to use in different experiments ---
    def initial_stamp(self):
        """ Takes the initial stamp from all the topics. This is useful to subtract it from all Time stamps and get a readable time"""
        try:
            # self.first_time_stamp = min(min(self.pressure_time_stamp), min(self.wrench_time_stamp),
            #                             min(self.event_time_stamp))

            self.first_time_stamp = min(min(self.wrench_time_stamp),
                                        min(self.joint_time_stamp),
                                        min(self.event_time_stamp),
                                        min(self.pressure_sc1_time_stamp),
                                        min(self.pressure_sc2_time_stamp),
                                        min(self.pressure_sc3_time_stamp),
                                        min(self.tof_time_stamp)
                                        )

        except ValueError:
            print('Value Error')
            self.first_time_stamp = 0

    def elapsed_times(self):
        """Subtracts the initial stamp from all topics' time-stamps to improve readability"""

        # STEP1: Obtain the initial time stamp of the experiment as a reference to the rest
        self.initial_stamp()

        # STEP2: Elapsed times for all the topics
        # UR5e topics
        self.wrench_elapsed_time = relative_values(self.wrench_time_stamp, self.first_time_stamp)
        self.joint_elapsed_time = relative_values(self.joint_time_stamp, self.first_time_stamp)
        # Experiment topics
        self.event_elapsed_time = relative_values(self.event_time_stamp, self.first_time_stamp)
        # Gripper topics
        self.pressure_elapsed_time = relative_values(self.pressure_time_stamp, self.first_time_stamp)
        self.pressure_sc1_elapsed_time = relative_values(self.pressure_sc1_time_stamp, self.first_time_stamp)
        self.pressure_sc2_elapsed_time = relative_values(self.pressure_sc2_time_stamp, self.first_time_stamp)
        self.pressure_sc3_elapsed_time = relative_values(self.pressure_sc3_time_stamp, self.first_time_stamp)
        self.tof_elapsed_time = relative_values(self.tof_time_stamp, self.first_time_stamp)

    def filter_wrench(self, filter_param):
        self.wrench_xforce_values = median_filter(self.wrench_xforce_values, filter_param)
        self.wrench_yforce_values = median_filter(self.wrench_yforce_values, filter_param)
        self.wrench_zforce_values = median_filter(self.wrench_zforce_values, filter_param)
        self.wrench_xtorque_values = median_filter(self.wrench_xtorque_values, filter_param)
        self.wrench_ytorque_values = median_filter(self.wrench_ytorque_values, filter_param)
        self.wrench_ztorque_values = median_filter(self.wrench_ztorque_values, filter_param)

    def get_relative_values(self):
        """ Subtracts the first reading of the F/T -- when nothing is attached -- to get rid of
            miscalibration or zero offsets
        """

        for i in range(len(self.wrench_time_stamp)):
            relative_zforce = self.wrench_zforce_values[i] - self.wrench_zforce_values[0]
            relative_yforce = self.wrench_yforce_values[i] - self.wrench_yforce_values[0]
            relative_xforce = self.wrench_xforce_values[i] - self.wrench_xforce_values[0]
            relative_ztorque = self.wrench_ztorque_values[i] - self.wrench_ztorque_values[0]
            relative_ytorque = self.wrench_ytorque_values[i] - self.wrench_ytorque_values[0]
            relative_xtorque = self.wrench_xtorque_values[i] - self.wrench_xtorque_values[0]

            relative_netforce = math.sqrt(relative_zforce ** 2 + relative_xforce ** 2 + relative_yforce ** 2)

            self.wrench_zforce_relative_values.append(relative_zforce)
            self.wrench_yforce_relative_values.append(relative_yforce)
            self.wrench_xforce_relative_values.append(relative_xforce)

            self.wrench_netforce_relative_values.append(relative_netforce)

            self.wrench_ztorque_relative_values.append(relative_ztorque)
            self.wrench_ytorque_relative_values.append(relative_ytorque)
            self.wrench_xtorque_relative_values.append(relative_xtorque)

    def get_features(self):
        """Basically run all the methods"""

        self.elapsed_times()

        # -------- Pressure Features ----------
        # self.get_atmospheric_pressure()
        # self.get_steady_vacuum()

        # -------- Wrench Features ------------
        self.filter_wrench(50)
        self.get_relative_values()

        # Normal and Tangential Forces
        # self.get_normal_angle()
        self.normal_and_tangent_forces()

        if self.metadata['labels']['apple pick result'] != 'c':
            self.get_detach_values()

        # self.check_errors()

    # ---------------- METHODS FOR BRANCH STIFFNESS --------------
    def eef_location(self, plots='no'):
        """Obtain the 3d position of the End Effector"""

        x = []
        y = []
        z = []

        beta = []
        alpha = []
        gamma = []

        # STEP 1: Forward Kinematics
        counter = []
        for i in range(len(self.j0_shoulder_pan_values)):
            joints = np.array([self.j0_shoulder_pan_values[i],
                               self.j1_shoulder_lift_values[i],
                               self.j2_elbow_values[i],
                               self.j3_wrist1_values[i],
                               self.j4_wrist2_values[i],
                               self.j5_wrist3_values[i]])
            position, orientation = ur5e_fk_dh(joints)

            x.append(position[0])
            y.append(position[1])
            z.append(position[2])

            beta.append(orientation[0])
            alpha.append(orientation[1])
            gamma.append(orientation[2])

            counter.append(i)

        self.eef_x = x
        self.eef_y = y
        self.eef_z = z

        self.beta = beta
        self.alpha = alpha
        self.gamma = gamma

        # STEP 2: Orientation of the eef cframe
        # note: I don't know the exact reason why the eef aligns correctly with implicit rotation,
        # which is the reason why I swap alpha and gamma
        # https://en.wikipedia.org/wiki/Rotation_matrix
        alpha = self.gamma[-1]
        beta = self.beta[-1]
        gamma = self.alpha[-1]
        logging.debug('EEF angles alpha: %i.2, beta: %i.2, gamma: %i.2' %(alpha, beta, gamma))

        sin_a = math.sin(alpha)
        sin_b = math.sin(beta)
        sin_g = math.sin(gamma)
        cos_a = math.cos(alpha)
        cos_b = math.cos(beta)
        cos_g = math.cos(gamma)

        Rot_M = np.array(
            [[cos_b * cos_g, sin_a * sin_b * cos_g - cos_a * sin_g, cos_a * sin_b * cos_g + sin_a * sin_g, 0],
             [cos_b * sin_g, sin_a * sin_b * sin_g + cos_a * cos_g, cos_a * sin_b * sin_g - sin_a * cos_g, 0],
             [-sin_b, sin_a * cos_b, cos_a * cos_b, 0],
             [0, 0, 0, 1]])

        z_axis = np.array([[0, 0, 1, 1]]).transpose()
        y_axis = np.array([[0, 1, 0, 1]]).transpose()
        x_axis = np.array([[1, 0, 0, 1]]).transpose()

        plot_factor = 0.1  # to plot vectors with about 10cm length
        eef_x_vector = np.dot(Rot_M, x_axis)
        eef_x_vector = plot_factor * eef_x_vector / np.linalg.norm(eef_x_vector)
        eef_y_vector = np.dot(Rot_M, y_axis)
        eef_y_vector = plot_factor * eef_y_vector / np.linalg.norm(eef_y_vector)
        eef_z_vector = np.dot(Rot_M, z_axis)
        eef_z_vector = plot_factor * eef_z_vector / np.linalg.norm(eef_z_vector)

        # STEP 3: Location of the apple
        stem_vector = 0.01 * np.ones(3)
        calix_coord, stem_coord, branch_coord = self.apple_pose()

        compare_coords = 'same'
        for  i,j in zip(branch_coord, stem_coord):
            if i != j:
                compare_coords = 'different'
                break

        if compare_coords != 'same':          #avoid divisions by zero
            stem_vector = np.subtract(branch_coord, stem_coord)
            stem_vector = plot_factor * stem_vector / np.linalg.norm(stem_vector)  # Normalize its magnitude
        apple_vector = np.subtract(stem_coord, calix_coord)

        x_last = self.eef_x[-1]
        y_last = self.eef_y[-1]
        z_last = self.eef_z[-1]
        self.offset_eef_apple = point_to_line_distance(self.apple_center_loc, [x_last, y_last, z_last], eef_z_vector)
        logging.debug('EEF offset: %i.2 ' % self.offset_eef_apple)

        # plots
        if plots == 'yes':

            # 3D Plot of eef trajectory
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            ax.set_title('End Effector Trajectory\n' + self.filename)
            ax.set_xlabel('x[m]')
            ax.set_ylabel('y[m]')
            ax.set_zlabel('z[m]')
            ax.plot3D(x, y, z, 'black', linewidth='0.5')

            # Z axis over time
            fig = plt.figure()
            plt.plot(counter, z)
            plt.title('EEF distance in z axis [m] \n' + self.filename)

            # 3D plot of the apple
            a = self.apple_center_loc[0]
            b = self.apple_center_loc[1]
            c = self.apple_center_loc[2]

            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            apple_radius = self.apple_diameter / 2000

            x_a = (apple_radius * np.outer(np.cos(u), np.sin(v))) + a
            y_a = (apple_radius * np.outer(np.sin(u), np.sin(v))) + b
            z_a = (apple_radius * np.outer(np.ones(np.size(u)), np.cos(v))) + c

            ax.plot_surface(x_a, y_a, z_a, rstride=4, cstride=4, color='r', linewidth=0, alpha=0.2)

            # Draw Stem Vector
            ax.quiver(stem_coord[0], stem_coord[1], stem_coord[2], stem_vector[0], stem_vector[1], stem_vector[2], length=1, color='green')
            ax.text(stem_coord[0], stem_coord[1], stem_coord[2], "stem", color='green', size=10, zorder=1)

            # Draw Apple axis
            ax.quiver(calix_coord[0], calix_coord[1], calix_coord[2], apple_vector[0], apple_vector[1], apple_vector[2], length=1, color='red')
            ax.text(calix_coord[0], calix_coord[1], calix_coord[2], "apple main axis", color='red', size=10, zorder=1)

            # Make 3dplot have same aspect ratio
            x_maxes = max([max(x), x_a.max()])
            x_mins = min([min(x), x_a.min()])
            y_maxes = max([max(y), y_a.max()])
            y_mins = min([min(y), y_a.min()])
            z_maxes = max([max(z), z_a.max()])
            z_mins = min([min(z), z_a.min()])
            max_range = np.array([x_maxes - x_mins, y_maxes - y_mins, z_maxes - z_mins]).max() * 0.75
            mid_x = (x_maxes + x_mins) * 0.5
            mid_y = (y_maxes + y_mins) * 0.5
            mid_z = (z_maxes + z_mins) * 0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

            # Plot eef cframe
            x = self.eef_x[-1]
            y = self.eef_y[-1]
            z = self.eef_z[-1]

            u = eef_x_vector[0]
            v = eef_x_vector[1]
            w = eef_x_vector[2]
            ax.quiver(x, y, z, u, v, w, length=1, color='red')

            u = eef_y_vector[0]
            v = eef_y_vector[1]
            w = eef_y_vector[2]
            ax.quiver(x, y, z, u, v, w, length=1, color='green')

            u = eef_z_vector[0]
            v = eef_z_vector[1]
            w = eef_z_vector[2]
            ax.quiver(x, y, z, u, v, w, length=1, color='blue')

            ax.text(x, y, z, "eef", color='black', size=10, zorder=1)

            # Draw a line from eef towards the apple in z'direction
            eef_location_vector = np.array([[x, y, z]]).transpose()
            scaled_vector = np.dot(5, eef_z_vector[:3])
            new_point = np.add(scaled_vector, eef_location_vector )
            x1 = new_point.item(0)
            y1 = new_point.item(1)
            z1 = new_point.item(2)
            ax.plot([x, x1], [y, y1], [z, z1], linestyle='dotted')

            text = 'offset: ' + str(self.offset_eef_apple) + 'mm'
            ax.text((a+x)/2, (b+y)/2, (c+z)/2, text, color='black', size=10, zorder=1)

    def pick_points(self, plots='no'):
        """Find the EEF location during the Start and Finish of the pick
        """

        # EEF location from the FK
        x = self.eef_x
        y = self.eef_y
        z = self.eef_z

        # Step1: Locate starting and final points of the pick
        idx_1, idx_2 = locate_index_of_deltas(z)
        self.wrench_idx_start_pick = idx_1
        self.wrench_idx_end_pick = idx_2

        # Step2: Measure relative distance travelled by the eef
        distances = []
        counter = []
        for i in range(idx_2 - idx_1):
            delta_x = x[idx_1 + i] - x[idx_1]
            delta_y = y[idx_1 + i] - y[idx_1]
            delta_z = z[idx_1 + i] - z[idx_1]
            # print('\Deltas:', delta_x, delta_y, delta_z)
            distance = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
            distances.append(distance)
            counter.append(i)
        self.eef_travel = distances

        if plots == 'yes':
            fig = plt.figure()
            plt.plot(counter, distances)
            plt.title('Net distance travelled [m]')

        # Step3: Find Fmax between the POI's
        new_force_list = self.wrench_zforce_values[idx_1:idx_2]
        new_time_list = self.wrench_elapsed_time[idx_1:idx_2]
        idx_max = np.argmax(new_force_list)

        max_force = new_force_list[idx_max]

        max_force_time = new_time_list[idx_max]
        self.wrench_idx_maxForce_at_pick = self.wrench_elapsed_time.index(max_force_time)
        logging.debug('Pick MaxForce occurs at %i.2s (index %i)' % (max_force_time, self.wrench_idx_maxForce_at_pick))

    def pick_forces(self):
        """ APPROACH 1: USING POIS """

        # Print all values during the detachment
        max_netForce = max(self.wrench_netforce_relative_values)

        # index = self.wrench_netforce_relative_values.index(max_netForce)
        index = self.wrench_idx_maxForce_at_pick

        max_xForce = self.wrench_xforce_relative_values[index]
        max_yForce = self.wrench_yforce_relative_values[index]
        max_zForce = self.wrench_zforce_relative_values[index]

        max_tanForce = math.sqrt(max_xForce ** 2 + max_yForce ** 2)
        theta_rad = math.atan((max_zForce / max_tanForce))
        theta = math.degrees(theta_rad)

        # Store values
        self.max_normalForce_at_pick = max_zForce
        self.max_tangentialForce_at_pick = max_tanForce
        self.max_netForce_at_pick = max_netForce
        self.theta_at_pick = theta

        logging.debug('Max xForce: %i.2N, yForce: %i.2N, zForce: %i.2N' % (max_xForce, max_yForce, max_zForce))
        logging.debug('Max Normal: %i.2N, Tangential: %i.2N, Net: %i.2N, Theta: %i.0deg' % (
        max_zForce, max_tanForce, max_netForce, theta))

    def pick_stiffness(self, plots='no'):

        idx_1 = self.wrench_idx_start_pick
        idx_2 = self.wrench_idx_end_pick
        idx_max = self.wrench_idx_maxForce_at_pick

        # Stiffness w.r.t zForce(@eef) because the displacement is measured in zaxis (@eef)
        new_force_list = self.wrench_zforce_values[idx_1:idx_2]

        min_force = self.wrench_zforce_values[idx_1]
        max_force = self.wrench_zforce_values[idx_max]

        min_force_loc = self.eef_travel[0]
        max_force_loc = self.eef_travel[idx_max - idx_1]

        delta_force = max_force - min_force
        delta_travel = max_force_loc - min_force_loc

        # Delta Approach
        delta_stiffness = abs(delta_force) / abs(delta_travel)
        logging.debug('Stiffness - simple approach: %i' %delta_stiffness)

        # Linear Regression Approach
        lr_xtrain = np.array(self.eef_travel[0:idx_max - idx_1])
        lr_ytrain = np.array(self.wrench_zforce_values[idx_1:idx_max])
        lr_stiffness = st.linregress(lr_xtrain, lr_ytrain)
        logging.debug('Stiffness - lr approach: %i (Rvalue: %.2f)' %(lr_stiffness.slope, lr_stiffness.rvalue))

        stiffness = lr_stiffness.slope
        # TODO Stiffness in the direction of the net force
        # stiffness = abs(stiffness / math.sin(math.radians(self.theta_at_pick)))

        if plots == 'yes':
            fig = plt.figure()
            plt.plot(self.eef_travel, new_force_list)
            plt.title('z-Force vs Net distance travelled\n' + self.filename)
            plt.plot([min_force_loc,max_force_loc], [min_force, max_force], linestyle='dotted')
            text = 'LR Stiffness: ' + str(int(lr_stiffness.slope)) + 'N/m' + ' (Rvalue: ' + str(round(lr_stiffness.rvalue, 2)) +')'
            plt.text(delta_travel/2, min_force + delta_force/2, text)
            plt.ylim([-15, 40])
            plt.xlabel('EEF displacement [m]')
            plt.ylabel('zForce @ EEF [m]')

        self.stiffness = stiffness
        self.travel_at_pick = delta_travel

    # ----------- METHODS FOR EEF POSE W.R.T APPLE ---------
    def apple_pose(self):
        """Returns as numpy arrays the coordinates of calix, north pole and abcission layer"""

        # STEP1: Locate csv with the ground truth of the apples
        folder = '/media/alejo/Elements/Prosser_Data/Probe/20231101_apples_coords.csv'
        # folder = 'D:/Prosser_Data/Probe/20231101_apples_coords.csv'
        data_list = pd.read_csv(folder)

        array_sp = np.ones(3)
        array_np = np.ones(3)
        array_ab = np.ones(3)

        try:
            idx = data_list['Label'].tolist().index('apple' + str(self.apple_id))

            # STEP2: Read coordinates from csv and convert into a numpy array
            apple_south_pole = data_list['South Pole coords'].iloc[idx]
            apple_north_pole = data_list['North Pole coords'].iloc[idx]
            abcission_coord = data_list['Abcision coords'].iloc[idx]

            # Convert string of list into list
            s_pole = apple_south_pole.strip('][').split(', ')
            n_pole = apple_north_pole.strip('][').split(', ')
            ab = abcission_coord.strip('][').split(', ')
            # print('Apple coords - list of strings', s_pole, n_pole, ab)

            # Convert list of strings into list of floats
            float_sp = [round(float(x), 4) for x in s_pole]
            float_np = [round(float(x), 4) for x in n_pole]
            float_ab = [round(float(x), 4) for x in ab]
            # print('Apple coords - list of floats', float_sp, float_np, float_ab)

            # Convert list of floats into an array
            array_sp = np.array(float_sp)
            array_np = np.array(float_np)
            array_ab = np.array(float_ab)
            logging.debug('Apple calix: %s, north pole: %s, abcission: %s' %(array_sp, array_np, array_ab))

            # Apple center
            self.apple_center_loc = np.mean([array_sp, array_np], axis=0)
            logging.debug("Apple id: %i, index: %i, calix location: %s" %(self.apple_id, idx, array_sp))

        except ValueError:
            logging.debug("Label not found in the csv file")

        logging.debug("Apple center: %s" %self.apple_center_loc)

        return array_sp, array_np, array_ab

    # --------------- METHODS FOR AIR PRESSURE -------------
    def get_atmospheric_pressure(self):
        """Takes initial and last reading as the atmospheric pressure.
        Both are taken because in some cases the valve was already on, hence the last one (after valve is off) is also checked
        """
        first_reading = self.pressure_values[0]
        last_reading = self.pressure_values[1]

        self.atmospheric_pressure = max(first_reading, last_reading)

    def get_normal_angle(self):
        try:
            self.normal_angle = math.acos(self.x_noise / self.surface_radius)
        except ValueError:
            ...

    def normal_and_tangent_forces(self):
        """Method to transform the Forces at the XZ cframe into a Normal-Tangential Cframe"""

        for fx, fz in zip(self.wrench_xforce_relative_values, self.wrench_zforce_relative_values):
            fn = fz * math.sin(self.normal_angle) + fx * math.cos(self.normal_angle)
            ft = -fz * math.cos(self.normal_angle) + fx * math.sin(self.normal_angle)
            self.normal_force_values.append(fn)
            self.tangent_force_values.append(ft)

    def get_steady_vacuum(self, start_label='Steady', end_label='Retrieve'):
        """Method to obtain the mean and std deviation of the vacuum during steady state"""

        # Get the index at which the steady state starts and ends
        start_index = self.event_values.index(start_label)
        end_index = self.event_values.index(end_label)

        # Get the time at which the steady state starts and ends
        steady_vacuum_start = self.event_elapsed_time[start_index]
        steady_vacuum_end = self.event_elapsed_time[end_index]
        # print("\n %.0d Starts at %.2f and ends at %.2f" %(self.id, steady_vacuum_start, steady_vacuum_end))

        # Get the steady state mean and std values
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if (time > steady_vacuum_start) and (time < steady_vacuum_end):
                if self.vacuum_type == 'barometric':
                    self.steady_pressure_values.append(value - self.atmospheric_pressure)
                elif self.vacuum_type == 'absolute':
                    self.steady_pressure_values.append(value)

        self.steady_vacuum_mean = np.mean(self.steady_pressure_values)
        self.steady_vacuum_std = np.std(self.steady_pressure_values)

        # print("\n\nMean: %.2f and Std: %.2f" % (self.steady_vacuum_mean, self.steady_vacuum_std))

        return self.steady_vacuum_mean, self.steady_vacuum_std

    def get_detach_values(self):
        """Method to obtain the max force during the retrieval
        """
        if self.exp_type == 'simple_suction':
            start_label = 'Steady'
        else:
            start_label = 'Retrieve'

        # Get indexes where detachment occurs
        start_index = self.event_values.index(start_label)
        end_index = self.event_values.index("Vacuum Off")

        # Get the time at which the steady state starts and ends
        retrieve_start = self.event_elapsed_time[start_index]
        vacuum_stops = self.event_elapsed_time[end_index]

        xforce_detach_values = []
        ytorque_detach_values = []
        zforce_detach_values = []
        nforce_detach_values = []
        tforce_detach_values = []
        sumforce_detach_values = []

        list_of_detach_values = [xforce_detach_values, ytorque_detach_values, zforce_detach_values, nforce_detach_values, tforce_detach_values, sumforce_detach_values]
        list_of_values = [self.wrench_xforce_relative_values, self.wrench_ytorque_relative_values, self.wrench_zforce_relative_values, self.normal_force_values, self.tangent_force_values, self.wrench_netforce_relative_values]

        list_of_max_values = []
        list_of_max_times = []
        list_of_min_values = []
        list_of_min_times = []

        for detach_value, values in zip(list_of_detach_values, list_of_values):
            for time, value in zip(self.wrench_elapsed_time, values):
                if (time > retrieve_start) and (time < vacuum_stops):
                    detach_value.append(value)
            try:
                max_value = max(detach_value)
                list_of_max_values.append(max_value)
                index = values.index(max_value)
                max_time = self.wrench_elapsed_time[index]
                list_of_max_times.append(max_time)

                min_value = min(detach_value)
                list_of_min_values.append(min_value)
                index_min = values.index(min_value)
                min_time = self.wrench_elapsed_time[index_min]
                list_of_min_times.append(min_time)

            except ValueError:
                max_value = "error"

        self.max_detach_xforce = list_of_max_values[0]
        self.max_detach_xforce_time = list_of_max_times[0]
        self.max_detach_ytorque = list_of_max_values[1]
        self.max_detach_ytorque_time = list_of_max_times[1]
        self.max_detach_zforce = list_of_max_values[2]
        self.max_detach_zforce_time = list_of_max_times[2]
        self.max_detach_nforce = list_of_max_values[3]
        self.max_detach_nforce_time = list_of_max_times[3]
        self.max_detach_tforce = list_of_min_values[4]
        self.max_detach_tforce_time = list_of_min_times[4]
        self.max_detach_sumforce = list_of_max_values[5]
        self.max_detach_sumforce_time = list_of_max_times[5]

        return self.max_detach_zforce, self.max_detach_xforce

    def check_errors(self):
        """ Method to check possible errors that may invalidate the data. For instance:
        - arm didn't move and remain touching the surface after retrieve, hence no force is present.
        - suction cup collapsed in the air, and therefore showed some vacuum
        @note: This is meant for the SUCTION CUP CHARACTERIZATION experiments
        """

        # 1. Cases due to the arm movement solver:

        # 1.1. When arm didnt retrieve, force and vacuum remain constant before and after retrieve event. Hence you dont
        # see much change in the zforce
        force_range = 1
        p_threshold = 80
        p_range = 5
        retrieve_index = self.event_values.index("Retrieve")
        time_at_index = self.event_elapsed_time[retrieve_index]
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_relative_values):
            if time > time_at_index:
                force_at_retrieve = value
                break
            else:
                force_at_retrieve = 0
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_retrieve = value
                break

        vacuum_off_index = self.event_values.index("Vacuum Off")
        time_at_index = self.event_elapsed_time[vacuum_off_index]
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_relative_values):
            if time > time_at_index:
                force_at_vacuum_off = value
                break
            else:
                force_at_vacuum_off = 0
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_vacuum_off = value
                break

        if (abs(force_at_retrieve - force_at_vacuum_off) < force_range) and pressure_at_retrieve < p_threshold and abs(pressure_at_vacuum_off - pressure_at_retrieve) < p_range:
            print("Error ", force_at_retrieve, force_at_vacuum_off)
            self.errors.append("Arm didn't move after Retrieve")

        if self.exp_type == "vertical" and self.z_noise < 0.01 and pressure_at_retrieve > 90:
            self.errors.append("Arm didn't move after Retrieve")

        if self.exp_type == "horizontal" and self.x_noise < 0.02 and pressure_at_retrieve > 90:
            self.errors.append("Arm didn't move after Retrieve")

        # 1.2.When for some reason, one topic stopped from being recorded. Hence, the total elapsed time is different
        time_range = 1
        force_time = self.wrench_elapsed_time[-1] - self.wrench_elapsed_time[0]
        pressure_time = self.pressure_elapsed_time[-1] - self.pressure_elapsed_time[0]

        if abs(force_time - pressure_time) > time_range:
            self.errors.append("One of the topics wasn't recorded properly")

        # 1.3. When for some reason the arm didn't move to the starting position, hence the x_noise was doubled
        # this is checked with the x_noise vs radius
        if abs(self.x_noise - self.x_noise_command) > 0.002 or abs(self.z_noise - self.z_noise_command) > 0.002:
            self.errors.append("The noise was't addede properly")

        # 2. Cases due to the suction cup:
        # 2.1. When suction cup collapses. This shows an increase in vacuum after retrieving
        pressure_range = 5
        retrieve_index = self.event_values.index("Retrieve")
        time_at_index = self.event_elapsed_time[retrieve_index]
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_retrieve = value
                break

        vacuum_off_index = self.event_values.index("Vacuum Off")
        time_at_index = self.event_elapsed_time[vacuum_off_index]
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_vacuum_off = value
                break

        if (pressure_at_retrieve - pressure_at_vacuum_off) > pressure_range:
            self.errors.append("Cup collapsed after retrieve")

    def suction_engagement(self, vacuum_thr=60):
        # Check sensor values and see whether the cup was engaged or not

        # STEP 1: Find index when cups allegedly engaged with apple
        idx=0
        for count, event in enumerate(self.event_values):
            if event == 'Labeling cups':
                idx = count
                break

        time_of_index = self.event_elapsed_time[idx]

        # STEP 2: Find the index but in the pressure signals, having time as ref
        pressure_index = 0
        for count, value in enumerate(self.pressure_sc3_elapsed_time):
            if value > time_of_index:
                pressure_index = count
                break

        pressure_index += 10       # safety margin
        # STEP 2: Check sensor values at the index
        self.sc1_value_at_engagement = self.pressure_sc1_values[pressure_index]
        self.sc2_value_at_engagement = self.pressure_sc2_values[pressure_index]
        self.sc3_value_at_engagement = self.pressure_sc3_values[pressure_index]

        logging.debug('Elapsed time @ engagement: %.2f sec (index %i)' % (time_of_index, pressure_index))
        logging.debug('Engagement Values: %.1f, %.1f, %.1f' % (self.sc1_value_at_engagement, self.sc2_value_at_engagement, self.sc3_value_at_engagement))

        # STEP 3: Set labels accodring to the vacuum threshold
        sc_a_sensor_label = 'no'
        sc_b_sensor_label = 'no'
        sc_c_sensor_label = 'no'
        if self.sc1_value_at_engagement < vacuum_thr:
            sc_a_sensor_label = 'yes'
        if self.sc2_value_at_engagement < vacuum_thr:
            sc_b_sensor_label = 'yes'
        if self.sc3_value_at_engagement < vacuum_thr:
            sc_c_sensor_label = 'yes'

        sc_a_manual_label = self.metadata['labels']['suction cup a']
        sc_b_manual_label = self.metadata['labels']['suction cup b']
        sc_c_manual_label = self.metadata['labels']['suction cup c']
        logging.debug('Manual labels: %s, %s, %s' %(sc_a_manual_label, sc_b_manual_label, sc_c_manual_label))

        if sc_a_sensor_label != sc_a_manual_label or sc_b_sensor_label != sc_b_manual_label or sc_c_sensor_label != sc_c_manual_label:
            print('warning, correct labels')
            print(self.filename)
            print('Engagement Values: %.1f, %.1f, %.1f' % (self.sc1_value_at_engagement, self.sc2_value_at_engagement, self.sc3_value_at_engagement))
            print('Manual labels: %s, %s, %s' %(sc_a_manual_label, sc_b_manual_label, sc_c_manual_label))

    # ------------ METHODS FOR PLOTS ----------------
    def plots_stuff(self):
        """Plots wrench (forces and moments) and pressure readings"""

        force_time = self.wrench_elapsed_time
        xforce_values = self.wrench_xforce_relative_values
        yforce_values = self.wrench_yforce_relative_values
        zforce_values = self.wrench_zforce_relative_values
        sumforce_values = self.wrench_netforce_relative_values

        nforce_values = self.normal_force_values
        tforce_values = self.tangent_force_values

        xtorque_values = self.wrench_xtorque_relative_values
        ytorque_values = self.wrench_ytorque_relative_values
        ztorque_values = self.wrench_ztorque_relative_values

        pressure_time = self.pressure_elapsed_time
        pressure_values = self.pressure_values

        event_x = self.event_elapsed_time
        event_y = self.event_values

        max_xforce_time = self.max_detach_xforce_time
        max_xforce_val = self.max_detach_xforce
        max_zforce_time = self.max_detach_zforce_time
        max_zforce_val = self.max_detach_zforce
        max_ytorque_time = self.max_detach_ytorque_time
        max_ytorque_val = self.max_detach_ytorque
        max_nforce_val = self.max_detach_nforce
        max_nforce_time = self.max_detach_nforce_time
        max_tforce_val = self.max_detach_tforce
        max_tforce_time = self.max_detach_tforce_time
        max_sumforce_val = self.max_detach_sumforce
        max_sumforce_time = self.max_detach_sumforce_time

        # --- Labels, limits and other annotations ---
        figure, axis = plt.subplots(4, 3, figsize=(16, 9))
        yvalues = [zforce_values, yforce_values, xforce_values, pressure_values]
        xvalues = [force_time, force_time, force_time, pressure_time]
        ylabels = ["zForce [N]", "yForce [N]", "xForce [N]", "Pressure [hPa]"]
        ylims = [[-22, 15], [-22, 15], [-22, 15], [0, 1100]]
        colors = ['blue', 'green', 'red', 'black']
        for i in range(len(axis)):
            axis[i, 0].plot(xvalues[i], yvalues[i], colors[i])
            axis[i, 0].axvline(x=max_xforce_time, color='red', linestyle='dashed', linewidth=1)
            axis[i, 0].axvline(x=max_zforce_time, color='blue', linestyle='dashed', linewidth=1)
            axis[i, 0].axvline(x=max_nforce_time, color='orange', linestyle='dashed', linewidth=1)
            axis[i, 0].axvline(x=max_tforce_time, color='gray', linestyle='dashed', linewidth=1)
            axis[i, 0].grid()
            axis[i, 0].set_ylabel(ylabels[i])
            axis[i, 0].set_ylim(ylims[i])
            # Add vertical lines at the events
            for event, label in zip(event_x, event_y):
                axis[i, 0].axvline(x=event, color='black', linestyle='dotted', linewidth=1)
                if i == (len(axis)-1):
                    axis[i, 0].text(event, 100, label, rotation=90, color='black')
                    axis[i, 0].set_xlabel("Elapsed Time [sec]")

        # ---- Max Force Annotations ---
        axis[2, 0].annotate('Max xForce:' + str(round(max_xforce_val, 3)), xy=(max_xforce_time, max_xforce_val),
                         xycoords='data', xytext=(max_xforce_time - 0.5, max_xforce_val + 10),
                         va='top', ha='right', arrowprops=dict(facecolor='red', shrink=0))
        axis[0, 0].annotate('Max zForce:' + str(round(max_zforce_val, 3)), xy=(max_zforce_time, max_zforce_val),
                         xycoords='data', xytext=(max_zforce_time - 0.5, max_zforce_val + 10),
                         va='top', ha='right', arrowprops=dict(facecolor='blue', shrink=0))
        axis[1, 1].annotate('Max yTorque:' + str(round(max_ytorque_val, 4)), xy=(max_ytorque_time, max_ytorque_val),
                            xycoords='data', xytext=(max_ytorque_time - 0.5, max_ytorque_val + 0.15),
                            va='top', ha='right', arrowprops=dict(facecolor='green', shrink=0))
        axis[1, 2].annotate('Max nForce:' + str(round(max_nforce_val, 3)), xy=(max_nforce_time, max_nforce_val),
                            xycoords='data', xytext=(max_nforce_time - 0.5, max_nforce_val + 10),
                            va='top', ha='right', arrowprops=dict(facecolor='orange', shrink=0))
        axis[2, 2].annotate('Max tForce:' + str(round(max_tforce_val, 3)), xy=(max_tforce_time, max_tforce_val),
                            xycoords='data', xytext=(max_tforce_time - 0.5, max_tforce_val -3),
                            va='top', ha='right', arrowprops=dict(facecolor='gray', shrink=0))
        axis[0, 2].annotate('Max sumForce:' + str(round(max_sumforce_val, 3)), xy=(max_sumforce_time, max_sumforce_val),
                            xycoords='data', xytext=(max_sumforce_time - 0.5, max_sumforce_val + 10),
                            va='top', ha='right', arrowprops=dict(facecolor='gray', shrink=0))

        # --- Add error in the title if there was any ---
        try:
            error_type = self.errors[0]
        except IndexError:
            error_type = "data looks good"
        figure.suptitle(self.filename + "\n\n" + error_type)
        print(self.filename)

        # --- Labels, limits and other annotations ---
        yvalues = [ztorque_values, ytorque_values, xtorque_values, pressure_values]
        xvalues = [force_time, force_time, force_time, pressure_time]
        ylabels = ["zTorque [Nm]", "yTorque [Nm]", "xTorque [Nm]", "Pressure [hPa]"]
        ylims = [[-0.35, 0.35], [-0.35, 0.35], [-0.35, 0.35], [0, 1100]]
        colors = ['blue', 'green', 'red', 'black']

        for i in range(len(axis)):
            axis[i, 1].plot(xvalues[i], yvalues[i], colors[i])
            axis[i, 1].axvline(x=max_xforce_time, color='red', linestyle='dashed', linewidth=1)
            axis[i, 1].axvline(x=max_zforce_time, color='blue', linestyle='dashed', linewidth=1)
            axis[i, 1].axvline(x=max_nforce_time, color='orange', linestyle='dashed', linewidth=1)
            axis[i, 1].axvline(x=max_tforce_time, color='gray', linestyle='dashed', linewidth=1)
            axis[i, 1].grid()
            axis[i, 1].set_ylabel(ylabels[i])

            axis[i, 1].yaxis.set_label_position("left")
            axis[i, 1].yaxis.tick_left()

            axis[i, 1].set_ylim(ylims[i])
            # Add vertical lines at the events
            for event, label in zip(event_x, event_y):
                axis[i, 1].axvline(x=event, color='black', linestyle='dotted', linewidth=1)
                if i == (len(axis) - 1):
                    axis[i, 1].text(event, 0, label, rotation=90, color='black')
                    axis[i, 1].set_xlabel("Elapsed Time [sec]")

        # --- Labels, limits and other annotations ---
        yvalues = [sumforce_values, nforce_values, tforce_values, pressure_values]
        xvalues = [force_time, force_time, force_time, pressure_time]
        ylabels = ["sum Force [N]", "normal Force [N]", "Tangential Force [N]", "Pressure [hPa]"]
        ylims = [[-22, 15], [-22, 15], [-5, 5], [0, 1100]]
        colors = ['blue', 'orange', 'gray', 'black']

        for i in range(len(yvalues)):
            axis[i, 2].plot(xvalues[i], yvalues[i], colors[i])
            axis[i, 2].axvline(x=max_xforce_time, color='red', linestyle='dashed', linewidth=1)
            axis[i, 2].axvline(x=max_zforce_time, color='blue', linestyle='dashed', linewidth=1)
            axis[i, 2].axvline(x=max_nforce_time, color='orange', linestyle='dashed', linewidth=1)
            axis[i, 2].axvline(x=max_tforce_time, color='gray', linestyle='dashed', linewidth=1)
            axis[i, 2].grid()
            axis[i, 2].set_ylabel(ylabels[i])

            axis[i, 2].yaxis.set_label_position("left")
            axis[i, 2].yaxis.tick_left()

            axis[i, 2].set_ylim(ylims[i])
            # Add vertical lines at the events
            for event, label in zip(event_x, event_y):
                axis[i, 2].axvline(x=event, color='black', linestyle='dotted', linewidth=1)
                if i == (len(axis) - 1):
                    axis[i, 2].text(event, 0, label, rotation=90, color='black')
                    axis[i, 2].set_xlabel("Elapsed Time [sec]")

    def plot_only_pressure(self, type='single_plot'):
        """
        Plots only pressure readings
        @param type: 'single' to have the three suction cup plots in a single figure
                     'many' to have a figure per plot
        @return:
        """

        icra24_figure5_fonts = 24

        FONTSIZE = icra24_figure5_fonts
        TICKSIZE = icra24_figure5_fonts
        # FIGURESIZE = (8, 6)
        FIGURESIZE = (13, 5)  # Fig5 ICRA24


        lines = itertools.cycle(('dotted', 'dashed', 'dashdot'))
        colors = itertools.cycle(('orange', 'blue', 'green'))
        # text_locations = itertools.cycle((0, 6, 12, 0, 6, 12))
        text_locations = itertools.cycle((105, 95, 85, 105, 95, 85))

        # pressure_time = self.pressure_elapsed_time
        # pressure_values = self.pressure_values

        # --- Zoom In the a-xis if needed ---
        threshold = 0
        suctionCup1_times = []
        suctionCup2_times = []
        suctionCup3_times = []
        suctionCup1_values = []
        suctionCup2_values = []
        suctionCup3_values = []

        for i, j in zip(self.pressure_sc1_elapsed_time, self.pressure_sc1_values):
            if i > threshold:
                suctionCup1_times.append(i)
                suctionCup1_values.append(j)
        for i, j in zip(self.pressure_sc2_elapsed_time, self.pressure_sc2_values):
            if i > threshold:
                suctionCup2_times.append(i)
                suctionCup2_values.append(j)
        for i, j in zip(self.pressure_sc3_elapsed_time, self.pressure_sc3_values):
            if i > threshold:
                suctionCup3_times.append(i)
                suctionCup3_values.append(j)

        pressure_times = [suctionCup1_times, suctionCup2_times, suctionCup3_times]
        pressure_values = [suctionCup1_values, suctionCup2_values, suctionCup3_values]
        pressure_labels = ["cup A", "cup B", "cup C"]

        event_x = self.event_elapsed_time
        event_y = self.event_values

        # --- Condition the Event's labels a bit for the paper purposes
        picking_time = 10.597
        # picking_time = 22.9
        cnt = 0
        flag = 0
        for i, j in zip(event_x, event_y):
            if j == 'Labeling cups':                # Renames label
                event_y[cnt] = 'Cup engagement'
            if j == 'Labeling apple pick': #or j == 'Vacuum On':          # Deletes label
                event_x.pop(cnt)
                event_y.pop(cnt)
            if (i > picking_time) and (flag == 0):
                event_y.insert(cnt, 'Apple Picked')
                event_x.insert(cnt, picking_time)
                flag = 1
            if j == 'Retrieve':
                event_y[cnt] = 'pull back'
            cnt += 1

        if type == 'many':
            print(self.filename, '\n')
            fig, axes = plt.subplots(3, 1, figsize=FIGURESIZE)
            cnt = 0
            for pressure_time, pressure_value, pressure_label in zip(pressure_times, pressure_values, pressure_labels):

                axes[cnt].plot(pressure_time, pressure_value, linewidth=2)

                for event, label in zip(event_x, event_y):
                    axes[cnt].axvline(x=event, color='black', linestyle='dotted', linewidth=2)
                    axes[cnt].text(event, 50, label, rotation=90, color='black', fontsize=FONTSIZE)
                    axes[cnt].set_xlabel("Elapsed Time [sec]", fontsize=FONTSIZE)
                    axes[cnt].set_ylabel("Pressure [kPa]", fontsize=FONTSIZE)
                    axes[cnt].set_ylim([0, 110])

                # --- Add error in the title if there was any ---
                try:
                    error_type = self.errors[0]
                except IndexError:
                    error_type = "data looks good"
                # axes[cnt].suptitle(self.filename + "\n\n" + error_type)

                title_text = "Experiment Type: " + str(self.exp_type) + \
                             ", F.P.: " + str(self.pressure) + "PSI" \
                             ", Diameter: " + str(self.surface_radius * 2000) + "mm" \
                             "\n,Pitch: " + str(int(round(math.degrees(self.pitch), 0))) + "deg" \
                             ", xNoise Command: " + str(round(self.x_noise_command * 1000, 2)) + "mm" \
                             ", Repetition No: " + str(self.repetition)

                axes[cnt].grid()
                # axes[cnt].set_xticks(size=TICKSIZE)
                # axes[cnt].set_yticks(size=TICKSIZE)

                axes[cnt].set_title(pressure_label)

                plt.title(self.filename + "\n" + error_type, fontsize=8)
                plt.suptitle(title_text)
                cnt += 1

        elif type == 'single_plot':

            plt.figure(figsize=FIGURESIZE)
            plt.rc('font', family='serif')
            cnt = 0

            # Plot pressure signals
            for pressure_time, pressure_value, pressure_label in zip(pressure_times, pressure_values, pressure_labels):
                plt.plot(pressure_time, pressure_value, linewidth=2, label=pressure_label, linestyle=next(lines), color=next(colors))
                cnt += 1

            # Plot experiment events for reference
            for event, label in zip(event_x, event_y):
                plt.axvline(x=event, color='black', linestyle='--', linewidth=1.5)
                plt.text(event, 5, label.lower(), rotation=45, color='black', fontsize=TICKSIZE)

            plt.xlabel("Elapsed Time [sec]", fontsize=FONTSIZE)
            plt.ylabel("Pressure [kPa]", fontsize=FONTSIZE)
            plt.ylim([0, 110])
            # plt.xlim([0, 25])
            plt.xlim([0, 16])       # Fig.5 ICRA24
            # plt.xlim([0,30])        # Fig.8 ICRA24
            plt.yticks(fontsize=TICKSIZE)
            plt.xticks(fontsize=TICKSIZE)
            plt.grid()
            plt.legend(fontsize=FONTSIZE)
            plt.tight_layout()

    def plot_only_total_force(self):
        """Plots only force readings (forces and moments)"""

        icra24_figure5_fonts = 24

        FONTSIZE = icra24_figure5_fonts           # Use 24 for papers
        TICKSIZE = icra24_figure5_fonts
        FIGURESIZE = (8, 6)
        FIGURESIZE = (13, 5)  # Fig5 ICRA24

        plt.figure(figsize=FIGURESIZE)

        # ------- Zoom In the a-xis if needed -------
        threshold = 0
        force_time = []
        sumforce_values = []

        force_values = self.wrench_netforce_relative_values
        # force_values = self.wrench_zforce_values

        for i, j in zip(self.wrench_elapsed_time, force_values):
            if i > threshold:
                force_time.append(i)
                sumforce_values.append(j)
        # --------------------------------------------

        event_x = self.event_elapsed_time
        event_y = self.event_values

        # --- Condition the Event's labels a bit for the paper purposes
        picking_time = 10.597       # Fig5. ICRA24
        # picking_time = 22.9
        cnt = 0
        flag = 0
        for i, j in zip(event_x, event_y):
            if j == 'Labeling cups':                # Renames label for ICRA'24 paper
                event_y[cnt] = 'Cup engagement'
            if j == 'Labeling apple pick': # or j =='Vacuum On':          # Delete this label
                event_x.pop(cnt)
                event_y.pop(cnt)
            if (i > picking_time) and (flag == 0):  # Adds label for ICRA'24 paper
                event_y.insert(cnt, 'Apple Picked')
                event_x.insert(cnt, picking_time)
                flag = 1
            if j == 'Retrieve':
                event_y[cnt] = 'pull back'
            cnt += 1

        max_sumforce_val = self.max_detach_sumforce
        max_sumforce_time = self.max_detach_sumforce_time

        plt.plot(force_time, sumforce_values, linewidth=2, color='red')
        plt.annotate('Max Force: ' + str(round(max_sumforce_val, 1)), xy=(max_sumforce_time, max_sumforce_val),
                            xycoords='data', xytext=(max_sumforce_time + 0, max_sumforce_val + 1.5),
                            va='top', ha='right', arrowprops=dict(facecolor='orange', shrink=0), fontsize=FONTSIZE)

        for event, label in zip(event_x, event_y):
            plt.axvline(x=event, color='black', linestyle='--', linewidth=1.5)
            # plt.text(event, 0.5, label.lower(), rotation=45, color='black', fontsize=FONTSIZE)
            plt.text(event, 2, label.lower(), rotation=45, color='black', fontsize=FONTSIZE)   # Fig.5 ICRA24
            plt.xlabel("Elapsed Time [sec]", fontsize=FONTSIZE)
            plt.ylabel("Force [N]", fontsize=FONTSIZE)
            # plt.ylim([-5, 40])
            plt.ylim([0, 10])       # Limits for Fig.5 ICRA24


        # --- Add error in the title if there was any ---
        try:
            error_type = self.errors[0]
        except IndexError:
            error_type = "data looks good"
        # plt.suptitle(self.filename + "\n\n" + error_type)
        print(self.filename)

        title_text = "Experiment Type: " + str(self.exp_type) + \
                     ", F.P.: " + str(self.pressure) + "PSI" \
                     ", Diameter: " + str(self.surface_radius * 2000) + "mm" \
                     "\n,Pitch: " + str(int(round(math.degrees(self.pitch), 0))) + "deg" \
                     ", xNoise Command: " + str(round(self.x_noise_command * 1000, 2)) + "mm" \
                     ", Repetition No: " + str(self.repetition)

        plt.grid()
        plt.xticks(size=TICKSIZE)
        plt.yticks(size=TICKSIZE)
        # plt.title(self.filename + "\n" + error_type, fontsize=8)
        # plt.suptitle(title_text)
        # plt.xlim([0, 100])
        plt.xlim([0, 16])       # Fig.5 ICRA24
        # plt.xlim([0,30])        # Fig.8 ICRA24
        plt.tight_layout()

    def plot_only_pressure_animated(self, location, filename):
        """Plots wrench (forces and moments) and pressure readings
        Refs:https://stackoverflow.com/questions/61808191/is-there-an-easy-way-to-animate-a-scrolling-vertical-line-in-matplotlib
        """

        pressure_time = self.pressure_elapsed_time
        pressure_values = self.pressure_values

        event_x = self.event_elapsed_time
        event_y = self.event_values

        # Sort png files in a list
        lst = os.listdir(location + filename + '/pngs')
        listop = []
        for i in lst:
            if i.endswith('.png'):
                x = i.split('.png')[0]
                listop.append(int(x))
        listop.sort()

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(12, 4.8))
        ax[0].plot(pressure_time, pressure_values, 'k-', linewidth=2)
        ax[0].set_xlim(0, max(pressure_time))
        ax[0].set_ylim(0, 120)
        ax[0].grid()
        ax[0].set_xlabel('Elapsed time [sec]')
        ax[0].set_ylabel('Pressure [kPa]')
        plt.ion()
        plt.show()
        plt.title(filename, loc='right')

        # Remove details from ax[1] because we are displaying only the image
        ax[1].xaxis.set_visible(False)
        ax[1].yaxis.set_visible(False)
        for spine in ['top', 'right', 'left', 'bottom']:
            ax[1].spines[spine].set_visible(False)

        # out = None
        counter = 0
        for i in listop:
            # Vertical Line moving along the x axis
            x = i/1000
            line = ax[0].axvline(x=x, color='red', linestyle='dotted', linewidth=2)

            # Picture from the rosbag file
            img = plt.imread(location + filename + '/pngs/' + str(i) + '.png', 0)
            im = OffsetImage(img, zoom=0.55)
            ab = AnnotationBbox(im, (0, 0), xycoords='axes fraction', box_alignment=(0, 0))
            ax[1].add_artist(ab)
            plt.pause(0.0001)

            # # Save the figure window into an avi file
            # img = pyautogui.screenshot()
            # frame = np.array(img)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # out,write(frame)

            # if out is None:
            #     out = cv2.VideoWriter(location + filename + '/trial.avi', cv2.VideoWriter_fourcc(*'MP4V'), 40, (640, 480))
            # img_for_video = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR)
            # out.write(img_for_video)

            # Remove annotations to avoid RAM memory consumption
            ab.remove()
            line.remove()

            if counter == 0:
                time.sleep(10)

            counter += 1

        # out.release()


# ----------------------- FUNCTIONS FOR EACH TYPE OF EXPERIMENT ANALYSIS -----------------
def noise_experiments(exp_type="vertical"):

    plt.figure()

    # exp_type = "vertical"
    # exp_type = "horizontal"

    # --- Controlled variables ---
    radius = 0.0425
    # radius = 0.0375
    # pressures = [50, 60, 70, 80]    #only in dataset1 we did @80psi
    pressures = [40, 50, 60, 70]

    # Number of noise steps implemented for each direction
    if exp_type == 'vertical':
        n_noises = 12
    else:
        n_noises = 10

    n_reps = 4

    # --- Sweep all the pressures ---
    for pressure in pressures:

        noises_vacuum_means = []
        noises_vacuum_stds = []
        noises_xnoises = []
        noises_znoises = []
        noises_zforce_means = []
        noises_zforce_stds = []
        noises_xforce_means = []
        noises_xforce_stds = []
        noises_ytorque_means = []
        noises_ytorque_stds = []

        # --- Sweep all the noises ---
        for noise in range(n_noises):

            reps_xnoises = []
            reps_znoises = []
            reps_vacuum_means = []
            reps_vacuum_stds = []
            reps_zforce_max = []
            reps_xforce_max = []
            reps_ytorque_max = []
            reps_ytorque_max = []

            # --- Sweep all repetitions ---
            for rep in range(n_reps):

                # 1. Find file
                file, only_filename = find_file(exp_type, radius, pressure, noise, rep, 'surface')
                if file == "no_match":
                    continue

                # 2. Turn Bag into csvs if needed
                # Comment if it is already done
                # bag_to_csvs(file + ".bag")

                # 3. Read attributes from 'json' files
                metadata = read_json(file + ".json")

                # 4. Read values from 'csv' files for each 'json' file
                experiment = read_csvs(metadata, file)
                experiment.filename = only_filename

                # 5. Get different properties for each experiment
                experiment.get_features()
                # plt.close('all')
                # experiment.plots_stuff()
                # plt.show()

                # 6. Check if there were any errors during the experiment
                if len(experiment.errors) > 0:
                    continue

                # 7. Gather features from all the repetitions of the experiment
                reps_xnoises.append(experiment.x_noise)
                reps_znoises.append(experiment.z_noise)
                reps_vacuum_means.append(round(experiment.steady_vacuum_mean, 2))
                reps_vacuum_stds.append(round(experiment.steady_vacuum_std, 4))
                reps_zforce_max.append(experiment.max_detach_zforce)
                reps_xforce_max.append(experiment.max_detach_xforce)
                reps_ytorque_max.append(experiment.max_detach_ytorque)

            # --- Once all values are gathered for all repetitions, obtain the mean values
            if len(reps_vacuum_means) == 0:
                continue
            final_x_noise = np.mean(reps_xnoises)
            final_z_noise = np.mean(reps_znoises)
            final_vacuum_mean = np.mean(reps_vacuum_means)
            final_zforce_mean = np.mean(reps_zforce_max)
            final_zforce_std = np.std(reps_zforce_max)
            final_xforce_mean = np.mean(reps_xforce_max)
            final_xforce_std = np.std(reps_xforce_max)
            final_ytorque_mean = np.mean(reps_ytorque_max)
            final_ytorque_std = np.std(reps_ytorque_max)

            mean_stds = 0
            for i in range(len(reps_vacuum_stds)):
                mean_stds += reps_vacuum_stds[i] ** 2
            final_vacuum_std = (mean_stds / len(reps_vacuum_stds)) ** 0.5

            noises_vacuum_means.append(round(final_vacuum_mean, 2))
            noises_vacuum_stds.append(round(final_vacuum_std, 2))
            noises_xnoises.append(round(final_x_noise, 4))
            noises_znoises.append(round(final_z_noise, 4))
            noises_zforce_means.append(round(final_zforce_mean, 2))
            noises_zforce_stds.append(round(final_zforce_std, 2))
            noises_xforce_means.append(round(final_xforce_mean, 2))
            noises_xforce_stds.append(round(final_xforce_std, 2))
            noises_ytorque_means.append(round(final_ytorque_mean, 2))
            noises_ytorque_stds.append(round(final_ytorque_std, 2))

        # --- Once all values are collected for all noises, print and plot
        # suction_plots(exp_type, noises_xnoises, noises_znoises, noises_vacuum_means,
        #               noises_vacuum_stds, pressure, radius, 'false')
        suction_plots(exp_type, noises_xnoises, noises_znoises, noises_zforce_means,
                      noises_zforce_stds, pressure, radius, 'false')

        print('\nFeed In Pressure: ', pressure)
        print('zForce means: ', noises_zforce_means)
        if exp_type == 'vertical':
            print('zNoises: ', noises_znoises)
        else:
            print('xForce means: ', noises_xforce_means)
            print('xNoises: ', noises_xnoises)


        # # Save lists into csvs
        # filename = str(pressure) + "PSI_xnoises"
        # with open(filename, 'wb') as f:
        #     write = csv.writer(f, delimiter=',')
        #     for item in noises_xnoises:
        #         write.writerow(item)

        # circle_plots(noises_xnoises, 1, radius, noises_xforce_means, noises_zforce_means, pressure)

        # plt.show()

    plt.grid()
    plt.show()


def noise_experiments_pitch(exp_type="vertical", radius=75/1000, variable='pressure'):
    """

    @type radius: float
    """
    FIGURESIZE = (9, 7.2)
    plt.figure(figsize=FIGURESIZE)

    # --- Controlled variables ---

    # The pitch was varied only @60PSI
    pressure = 60

    pitches = [0.0, 15.0, 30.0, 45.0]
    # pitches = [30.0]

    # Number of noise steps implemented for each direction
    if exp_type == 'vertical':
        n_noises = 12
    else:
        n_noises = 10

    n_reps = 8

    errors = 0

    # --- Sweep all the pressures ---
    for pitch in pitches:

        noises_vacuum_means = []
        noises_vacuum_stds = []
        noises_xnoises = []
        noises_znoises = []
        noises_zforce_means = []
        noises_zforce_stds = []
        noises_xforce_means = []
        noises_xforce_stds = []
        noises_ytorque_means = []
        noises_ytorque_stds = []
        noises_sumforce_means = []
        noises_sumforce_stds = []

        # --- Sweep all the noises ---
        for noise in range(n_noises):

            reps_xnoises = []
            reps_znoises = []
            reps_vacuum_means = []
            reps_vacuum_stds = []
            reps_zforce_max = []
            reps_xforce_max = []
            reps_ytorque_max = []
            reps_ytorque_max = []
            reps_sumforce_max = []

            # --- Sweep all repetitions ---
            for rep in range(n_reps):

                # 1. Find file
                file, only_filename = find_file(exp_type, radius, pressure, noise, (rep+1), pitch, 'surface')
                if file == "no_match":
                    continue

                # 2. Turn Bag into csvs if needed
                if os.path.isdir(file):
                    # print("csvs already created")
                    pass
                else:
                    bag_to_csvs(file + ".bag")

                # 3. Read attributes from 'json' files
                metadata = read_json(file + ".json")

                # 4. Read values from 'csv' files for each 'json' file
                experiment = read_csvs(metadata, file)
                experiment.filename = only_filename

                # 5. Get different properties for each experiment
                experiment.get_features()
                # plt.close('all')

                # if only_filename == 'horizontal_#2_pres_60_surface_3DPrintedPrimer_radius_0.0375_noise_7.56_pitch_15.0_rep_7':
                    # experiment.plots_stuff()
                    # experiment.plot_only_total_force()

                    # experiment.plot_only_pressure()
                    # plt.show()

                # 6. Check if there were any errors during the experiment and
                # gather features from all the repetitions of the experiment

                if len(experiment.errors) > 0:
                    errors += 1
                    print(errors)
                    continue
                    # Uncomment following lines if you want to make
                    reps_xnoises.append(experiment.x_noise_command)
                    reps_znoises.append(experiment.z_noise_command)
                    reps_vacuum_means.append(experiment.atmospheric_pressure)
                    reps_vacuum_stds.append('Nan')
                    reps_zforce_max.append('Nan')
                    reps_xforce_max.append('Nan')
                    reps_ytorque_max.append('Nan')
                    reps_sumforce_max.append('Nan')

                else:
                    reps_xnoises.append(experiment.x_noise)
                    reps_znoises.append(experiment.z_noise)
                    reps_vacuum_means.append(round(experiment.steady_vacuum_mean, 2))
                    reps_vacuum_stds.append(round(experiment.steady_vacuum_std, 4))
                    reps_zforce_max.append(experiment.max_detach_zforce)
                    reps_xforce_max.append(experiment.max_detach_xforce)
                    reps_ytorque_max.append(experiment.max_detach_ytorque)
                    reps_sumforce_max.append(experiment.max_detach_sumforce)

            # --- Once all values are gathered for all repetitions, obtain the mean values
            if len(reps_vacuum_means) == 0:
                continue
            final_x_noise = np.mean(reps_xnoises)
            final_z_noise = np.mean(reps_znoises)
            final_vacuum_mean = np.mean(reps_vacuum_means)
            final_zforce_mean = np.mean(reps_zforce_max)
            final_zforce_std = np.std(reps_zforce_max)
            final_xforce_mean = np.mean(reps_xforce_max)
            final_xforce_std = np.std(reps_xforce_max)
            final_ytorque_mean = np.mean(reps_ytorque_max)
            final_ytorque_std = np.std(reps_ytorque_max)
            final_sumforce_mean = np.mean(reps_sumforce_max)
            final_sumforce_std = np.std(reps_sumforce_max)

            mean_stds = 0
            for i in range(len(reps_vacuum_stds)):
                mean_stds += reps_vacuum_stds[i] ** 2
            final_vacuum_std = (mean_stds / len(reps_vacuum_stds)) ** 0.5

            noises_vacuum_means.append(round(final_vacuum_mean, 2))
            noises_vacuum_stds.append(round(final_vacuum_std, 2))
            noises_xnoises.append(round(final_x_noise, 4))
            noises_znoises.append(round(final_z_noise, 4))
            noises_zforce_means.append(round(final_zforce_mean, 2))
            noises_zforce_stds.append(round(final_zforce_std, 2))
            noises_xforce_means.append(round(final_xforce_mean, 2))
            noises_xforce_stds.append(round(final_xforce_std, 2))
            noises_ytorque_means.append(round(final_ytorque_mean, 2))
            noises_ytorque_stds.append(round(final_ytorque_std, 2))
            noises_sumforce_means.append(round(final_sumforce_mean, 2))
            noises_sumforce_stds.append(round(final_sumforce_std, 2))

        # --- Once all values are collected for all noises, print and plot
        if variable == 'pressure':
            suction_plots(variable, exp_type, noises_xnoises, noises_znoises, noises_vacuum_means,
                          noises_vacuum_stds, pressure, pitch, radius, 'false')
        elif variable == 'zforce':
            suction_plots(variable, exp_type, noises_xnoises, noises_znoises, noises_zforce_means,
                          noises_zforce_stds, pressure, pitch, radius, 'false')
        elif variable == 'force':
            suction_plots(variable, exp_type, noises_xnoises, noises_znoises, noises_sumforce_means,
                          noises_sumforce_stds, pressure, pitch, radius, 'false')

        print('\nFeed In Pressure: ', pressure)
        print('Vacuum means: ', noises_vacuum_means)
        print('Vacuum stds: ', noises_vacuum_stds)

        print('sumForce means: ', noises_sumforce_means)
        if exp_type == 'vertical':
            print('zNoises: ', noises_znoises)
        else:
            print('xForce means: ', noises_xforce_means)
            print('xNoises: ', noises_xnoises)

        # # Save lists into csvs
        # filename = str(pressure) + "PSI_xnoises"
        # with open(filename, 'wb') as f:
        #     write = csv.writer(f, delimiter=',')
        #     for item in noises_xnoises:
        #         write.writerow(item)

        # circle_plots(noises_xnoises, 1, radius, noises_xforce_means, noises_zforce_means, pressure)

        # plt.show()

    plt.grid()
    plt.show()


def simple_suction_experiment():

    # --- Controlled variables ---
    # pressures = [50, 60, 70, 80]
    pressures = [60]
    surfaces = ['Real_Apple', 'Gloss_Fake_Apple', '3DPrinted_with_Primer', '3DPrinted_with_Primer_85mm']
    # surfaces = ['Gloss_Fake_Apple_suctionA', 'Gloss_Fake_Apple_suctionB']
    # surfaces = ['3DPrintedPrimer85_suctionA', '3DPrintedPrimer85_suctionB']
    surfaces = ['Gloss_Fake_Apple', '3DPrinted_with_Primer_85mm']
    n_reps = 5

    vacuum_type = 'absolute'    # The one given by the sensor
    # vacuum_type = 'barometric'  # The one given by the sensor minus the atmospheric pressure

    figure, axis = plt.subplots(1, 4, figsize=(10, 5))

    # --- Sweep all pressures ---
    for pressure, ctr in zip(pressures, range(len(pressures))):

        surfaces_min_vacuums = []

        # --- Sweep all surfaces ---
        for surface in surfaces:

            reps_min_vacuums = []

            # --- Sweep all reps ---
            for rep in range(n_reps):

                # 1. Find file
                file, only_filename = find_file("simple_suction", 10000, pressure, 1000, rep, 0.0, surface)
                if file == "no_match":
                    continue

                # 2. Turn Bag into csvs if needed
                if os.path.isdir(file):
                    pass
                    # print("csvs already created")
                else:
                    bag_to_csvs(file + ".bag")

                # 3. Read Attributes from 'json' files
                metadata = read_json(file + ".json")

                # 4. Read Values from 'csv' files
                experiment = read_csvs(metadata, file)
                experiment.filename = only_filename

                # 5. Get some features
                experiment.elapsed_times()
                experiment.get_atmospheric_pressure()
                experiment.vacuum_type = vacuum_type
                experiment.get_steady_vacuum('Steady', "Vacuum Off")

                # 6. Plot each experiment if needed
                # experiment.plot_only_pressure()
                # plt.show()

                # 7. Gather features from all reps.
                # reps_min_vacuums.append(min(experiment.steady_pressure_values))
                reps_min_vacuums.append(np.mean(experiment.steady_pressure_values))

            # Gather features from all surfaces
            surfaces_min_vacuums.append(reps_min_vacuums)

        # Finally plot values for the current pressure
        axis[ctr].boxplot(surfaces_min_vacuums)
        if vacuum_type == 'absolute':
            axis[ctr].set_ylim([100, 500])
            # axis[ctr].set_ylim([200, 400])
        elif vacuum_type == 'barometric':
            axis[ctr].set_ylim([-1000, -600])

        axis[ctr].set_xticklabels(['Real', 'Fake', '3DwithPrimer1', '3DwithPrimer2'], rotation=30, fontsize=8)
        axis[ctr].set_title('FP: %.0d [PSI]' % pressure)
        axis[ctr].set_xlabel('Surface')
        axis[ctr].grid()

        # Create a plot for each pressure
        # print(plot_title, surfaces_min_vacuums)
    axis[0].set_ylabel('Pressure [hPa]')
    plt.suptitle('Min Vacuum with different Feeding Pressures (FP)')
    plt.ylim([250, 300])
    plt.show()


def proxy_experiments():

    # STEP A: Data Location
    # folder = '/home/alejo/gripper_ws/src/suction-gripper/data/'   # Default Hard Drive

    folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"     # Fig.5 ICRA24

    # folder = '/media/alejo/042ba298-5d73-45b6-a7ec-e4419f0e790b/home/avl/data/REAL_APPLE_PICKS/'  # Hard Drive C
    # folder = '/media/alejo/Elements/Prosser_Data/'  # External Hard Drive

    # subfolder = 'MEDIUM_STIFFNESS/'
    subfolder = 'HIGH_STIFFNESS/'       # Fig5 ICRA24
    # subfolder = 'LOW_STIFFNESS/'
    # subfolder = 'Dataset - apple picks/'
    # subfolder = ''

    location = folder + subfolder

    # file = '2023083_proxy_sample_6_yaw_45_rep_0_stiff_medium_force_low'
    # file ='2023083_proxy_sample_0_yaw_-15_rep_1_stiff_medium_force_medium'
    # file = '2023083_proxy_sample_5_yaw_45_rep_0_stiff_medium_force_low'
    # file = '2023082_proxy_sample_5_yaw_45_rep_0_stiff_high_force_low'

    # --- ICRA24 paper plots
    file = '2023083_proxy_sample_5_yaw_45_rep_1_stiff_high_force_low'   # Fig5. ICRA24
    # file = '2023096_realapple5_attempt3'
    # file = '2023096_realapple6_attempt5'            # Fig8. ICRA24
    # file = '2023096_realapple3_attempt1'
    # file = '2023083_proxy_sample_0_yaw_45_rep_1_stiff_low_force_low'
    # file = '20230922_realapple3_attempt_1_orientation_0_yaw_0'
    # file = '20230920_sim_sample_0_yaw_-15_offset_0.005_rep_0_stiff_low_force_low'

    # --- ICRA24 accompanying video
    # file = '20230731_proxy_sample_6_yaw_45_rep_0_stiff_low_force_low'
    # file = '20230922_realapple3_attempt_1_orientation_0_yaw_0'
    # file = '20230922_realapple2_attempt_1_orientation_0_yaw_0'



    # --- 2. Turn bag into csvs if needed
    if os.path.isdir(location + file):
        # print("csvs already created")
        pass
    else:
        bag_to_csvs(location + file + ".bag")

    # --- 3. Create Experiment Object
    json_file = open(location + file + '.json')
    json_data = json.load(json_file)
    metadata = json_data
    experiment = Experiment(metadata=metadata)

    print(experiment.metadata['general'])

    # --- 4. Read values from 'csv'
    read_csvs(experiment, location + file)

    # --- 5. Get different features for the experiment
    experiment.get_features()

    # --- 6. Plot
    experiment.plot_only_pressure()
    experiment.plot_only_total_force()
    plt.show()


def real_experiments():

    # STEP A: Data Location
    # TODO: detect machine
    # folder = '/home/alejo/gripper_ws/src/suction-gripper/data/'   # ArmFarm laptop - Default Hard Drive
    # folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"     # ArmFarm laptop - Hard Drive B
    # folder = '/media/alejo/042ba298-5d73-45b6-a7ec-e4419f0e790b/home/avl/data/REAL_APPLE_PICKS/'  # ArmFarm laptop - Hard Drive C
    folder = '/media/alejo/Elements/Prosser_Data/'      # External Hard Drive
    # folder = 'D:/Prosser_Data/'
    # subfolder = 'Dataset - apple grasps/'
    # subfolder = 'Dataset - apple picks/'

    # --- ICRA24 accompanying video
    # file = '20230731_proxy_sample_6_yaw_45_rep_0_stiff_low_force_low'
    # file = '20230922_realapple3_attempt_1_orientation_0_yaw_0'
    # file = '20230922_realapple2_attempt_1_orientation_0_yaw_0'
    # file = '2023111_realapple1_mode_dual_attempt_1_orientation_0_yaw_0'

    # STEP B: Define variables to gather from all the experiments
    stiffnesses = []
    max_normalForces = []
    max_tangentialForces = []
    max_netForces = []
    thetas = []
    deltas = []
    apple_ids = []
    offsets = []        # distances between eef center and the apple center
    sc1_values_at_eng = []
    sc2_values_at_eng = []
    sc3_values_at_eng = []

    # STEP C: Sweep all bag files, and for each one do next
    subfolders= ['Dataset - apple grasps/', 'Dataset - apple picks/']
    subfolders = ['Dataset - apple grasps/']
    subfolders = ['Dataset - apple picks/']

    for subfolder in subfolders:

        location = folder + subfolder
        for file in tqdm(os.listdir(location)):
            if file.endswith('.bag') and file != 'vacuum_test_.bag':
                logging.debug('------------- Filename: %s ------------' %file)
                only_name = file.split('.')[0]  # remove extension
                file = only_name

                # Extract apple's id
                apple_id = file.split('apple')[1]
                apple_id = int(apple_id.split('_')[0])
                logging.debug('Apple %i' %apple_id)

                # STEP 1: Turn bag into csvs if needed
                if os.path.isdir(location + file):
                    logging.debug('csvs were already generated')
                    pass
                else:
                    logging.debug('make sure to import ros_scripts.py')
                    bag_to_csvs(location + file + ".bag")

                # STEP 2: Generate an experiment object for each file, and read its json file
                json_file = open(location + file + '.json')
                json_data = json.load(json_file)
                experiment = Experiment(apple_id=apple_id, metadata=json_data)
                experiment.filename = file
                logging.debug('metadata: %s' %experiment.metadata['general'])

                # STEP 3: Check conditions from metadata to continue with the analysis
                mode = experiment.metadata['robot']['actuation mode']
                pick = experiment.metadata['labels']['apple pick result']

                # if pick != 'c':
                # if pick == 'c':
                if True:
                # if mode == 'suction':
                # if file == '2023111_realapple2_mode_dual_attempt_1_orientation_0_yaw_0':

                    # STEP 4: Read values from 'csv'
                    read_csvs(experiment, location + file)

                    # STEP 5: Get features for the experiment
                    experiment.get_features()
                    experiment.eef_location(plots='no')
                    # experiment.pick_points(plots='no')
                    # experiment.pick_forces()
                    # experiment.pick_stiffness(plots='no')
                    experiment.suction_engagement()

                    # STEP 6: Append variables of interest
                    # stiffnesses.append(experiment.stiffness)
                    # max_normalForces.append(experiment.max_normalForce_at_pick)
                    # max_tangentialForces.append(experiment.max_tangentialForce_at_pick)
                    # max_netForces.append(experiment.max_netForce_at_pick)
                    # thetas.append(experiment.theta_at_pick)
                    # deltas.append(experiment.travel_at_pick)
                    apple_ids.append(experiment.apple_id)

                    air_pres_thr = 120       # @ Corvallis, atmospheric air pressure ~115kPa
                    if experiment.sc1_value_at_engagement < air_pres_thr:
                        sc1_values_at_eng.append(experiment.sc1_value_at_engagement)
                    if experiment.sc2_value_at_engagement < air_pres_thr:
                        sc2_values_at_eng.append(experiment.sc2_value_at_engagement)
                    if experiment.sc3_value_at_engagement < air_pres_thr:
                        sc3_values_at_eng.append(experiment.sc3_value_at_engagement)

                    if experiment.offset_eef_apple < 100:       # Record of the offset between eef and apple
                        offsets.append(experiment.offset_eef_apple)

                    # STEP 7: Single experiment plots
                    # experiment.plot_only_pressure(type='single')
                    # experiment.plot_only_total_force()

    # STEP 8: Grouped experiments plots
    if len(stiffnesses) > 1:

        print('Stiffness MEAN, MEDIAN: ', round(np.mean(stiffnesses), 2), round(np.median(stiffnesses), 2))
        print('NetForce MEAN, MEDIAN: ', round(np.mean(max_netForces), 2), round(np.median(max_netForces), 2))
        # Confidence Intervals
        confidence = 0.95
        a = stiffnesses
        c_int = st.t.interval(confidence, len(a)-1, loc=np.mean(a), scale=st.sem(a))
        print('Stiffness Confidence Interval 95%: ', round(c_int[0],2), round(c_int[1],2))
        a = max_netForces
        c_int = st.t.interval(confidence, len(a)-1, loc=np.mean(a), scale=st.sem(a))
        print('Net Force Confidence Interval 95%: ', round(c_int[0],2), round(c_int[1],2))

        # Boxplots
        fig = plt.figure()
        plt.boxplot(stiffnesses, notch=True, meanline=True)
        plt.title('Branch Stiffness (NormalForce / Travelled Distance) [N/m]')
        plt.grid()

        fig = plt.figure()
        plt.boxplot(max_netForces, notch=True, meanline=True)
        plt.title('Max Net Forces [N]')
        plt.grid()

        # Violinplots
        fig = plt.figure()
        plt.violinplot(stiffnesses, showmedians=True)
        plt.title('Branch Stiffness (NormalForce / Travelled Distance) [N/m]')
        plt.grid()

        fig = plt.figure()
        plt.violinplot(max_netForces, showmedians=True)
        plt.title('Max Net Forces [N]')
        plt.grid()

        # Scatter Plots
        fig = plt.figure()
        plt.scatter(deltas, max_netForces)
        plt.title('Travelled Distance Vs Net Force')
        plt.xlabel('EEF travelled distance [m]')
        plt.ylabel('Net Force [N]')
        plt.grid()
        for i, txt in enumerate(apple_ids):
            plt.annotate(txt, (deltas[i], max_netForces[i]))

        # Histograms
        list_to_hist(stiffnesses, 'Branch stiffness [N/m]')
        list_to_hist(max_normalForces, 'Max Normal Force [N]')
        list_to_hist(max_tangentialForces, 'Max Tangential Force [N]')
        list_to_hist(max_netForces, 'Max Net Force [N]')
        print('Apple IDs: ', apple_ids)
        print('Offsets: ', offsets)

    if len(apple_ids) > 1:
        list_to_hist(offsets, 'Offset from apple [mm]')
        list_to_hist(sc1_values_at_eng, 'ScA - Pressure [kPa]')
        list_to_hist(sc2_values_at_eng, 'ScB - Pressure [kPa]')
        list_to_hist(sc3_values_at_eng, 'ScC - Pressure [kPa]')

        fig = plt.figure()
        df = pd.DataFrame(np.array(([sc1_values_at_eng, sc2_values_at_eng, sc3_values_at_eng])).transpose(), columns=['scA', 'scB', 'scC'])
        df.boxplot(column=['scA', 'scB', 'scC'])
        plt.xlabel('Suction cup')
        plt.ylabel('Air pressure [kPa]')

    plt.show(block=False)
    plt.ion()
    # plt.show()
    plt.pause(0.001)  # Pause for interval seconds.
    input("\nhit[enter] to end.")
    plt.close('all')  # all open plots are correctly closed after each run


def orthogonal_load_cell_experiments():
    # Step 1 - Location
    # folder = '/home/alejo/Downloads/Mark10_experiments-20240227T171746Z-001/Mark10_experiments/experiment1_orthogonalLoad/'
    folder = 'C:/Users/avela/Dropbox/03 Temporal/03 Research/data/Mark10_experiments/'      # Personal Laptop

    # subfolder = 'experiment1_orthogonalLoad/'
    subfolder = 'experiment6_orthogonalLoad_accelStepper/'

    folder = folder + subfolder

    # Step 2 - Sweep all diameters
    diameters = [70, 75, 80]
    # steps_list = [1285, 1300, 1325, 1350, 1375, 1400, 1425]
    nut_travel_list = [52, 54, 56, 58, 60]

    for diameter in diameters:
        location = folder
        prefix = 'finger_load_apple_' + str(diameter) + 'mm'

        nut_travel_distances = []
        mean_max_forces = []
        std_max_forces = []

        for nut_travel in nut_travel_list:
            # sufix = str(steps) + 'steps.xlsx'

            sufix = str(nut_travel) + 'mm.xlsx'

            max_ortho = 'Nan'

            for file in sorted(os.listdir(location)):
                if file.startswith(prefix) and file.endswith(sufix):
                    print(file)

                    # Step 3: Open file and turn into dataframe
                    trial_df = pd.read_excel(location + file, index_col=0)

                    # Step 4: Check for POI's
                    max_ortho = max(trial_df['Load [N]'])
                    pois_pos, pois_neg = locate_index_of_deltas_v2(trial_df['Load [N]'].tolist(), max_ortho*0.8)
                    cycles = len(pois_pos)
                    print('Number of cycles: ', cycles)

                    # Step 5: Take the max values from each cycle
                    max_vals = []
                    n_points = trial_df.shape[0]
                    for i in range(cycles):
                        if i == cycles:
                            max_val = max(trial_df['Load [N]'][pois_pos[i]:pois_neg[i]])
                        else:
                            max_val = max(trial_df['Load [N]'][pois_pos[i]:n_points])
                        max_vals.append(max_val)

                    print('Max values', max_vals)

                    mean_max = np.mean(max_vals)
                    std_max = np.std(max_vals)

                    nut_travel_distances.append(nut_travel)
                    mean_max_forces.append(mean_max)
                    std_max_forces.append(std_max)

        print(nut_travel_distances)
        print(mean_max_forces)
        lows = np.subtract(mean_max_forces, std_max_forces)
        highs = np.add(mean_max_forces, std_max_forces)
        plt.plot(nut_travel_distances, mean_max_forces, 'o-', label=('diameter ' + str(diameter)+'mm'))
        plt.fill_between(nut_travel_distances, lows, highs, alpha=.2)
    plt.grid()

    plt.xlabel('Nut travel distance [mm]')
    plt.ylabel('Force [N]')
    plt.title('Normal Force from each finger [N]')

    # Plot the apple bruising threshold
    thr_press = 0.29e6    # Pa (Li et al. 2016)
    finger_width = 20   # mm
    thr_force = thr_press * (10 ** 2)/1e6
    print(thr_force)
    # plt.hlines(y=thr_force, xmin=1285, xmax=1425, linestyles='--', lw=2, label='Apple Bruising threshold')
    plt.hlines(y=thr_force, xmin=52, xmax=60, linestyles='--', lw=2, label='Apple Bruising threshold')
    plt.legend()
    plt.ylim([0, 35])

    plt.show()


def mark10_pullback_experiments():

    # Step 1 - Location
    # folder = '/home/alejo/Downloads/Mark10_experiments-20240309T010320Z-001/Mark10_experiments/'    # ArmFarm laptop
    folder = 'C:/Users/avela/Dropbox/03 Temporal/03 Research/data/Mark10_experiments/'  # Personal Laptop


    # --- Fake Apple / Pull-back trials at 0 degrees ---
    # subfolder = 'experiment2_pullingLoad_medSpring_medMagnet/'
    # exp_prefix = 'pull_load_fakeApple_' + str(steps) + 'steps_rep' + str(rep) + '_' + tag + '_'
    # tags = ['V', 'F', 'VF']
    # steps_list = [1285, 1300, 1325, 1350, 1375, 1400]

    # --- Fixed Apple / Pull-back trials at 0 degrees ----
    # subfolder = 'experiment3_pullingLoad_fixedApple/'
    # exp_prefix = 'pull_load_fixedApple_' + str(steps) + 'steps_rep' + str(rep) + '_' + tag + '.'
    # tags = ['V', 'F', 'VF']
    # steps_list = [1325, 1350, 1375, 1400]


    # ---- EQUATOR OFFSET ----
    # subfolder = 'experiment5_pullingLoad_fixedApple_distanced/'
    # exp_prefix = tag + '_dist_' + str(steps) + '_rep' + str(rep)
    # tags = ['suction', 'fingers', 'dual']
    # angles = [0, 2, 4, 6, 8, 10, 12, 15, 18, 21, 24, 27, 30]
    # steps_list = angles

    # ---- CLAMPING SPEED ----
    mark10_plots(folder + 'experiment8_pullBack_fixedApple_fingerSpeed/',
                 ['suction', 'fingers', 'dual'],
                 ['Suction cups', 'Fingers', 'Dual'],
                 [140, 190, 240, 290, 340],
                 10,
                 'Nut-Travel speed [rpm]'
                 )

    # ---- NUT TRAVEL DISTANCE ----
    mark10_plots(folder + 'experiment7_pullBack_fixedApple_fingerDistance/',
                 ['suction', 'fingers', 'dual'],
                 ['Suction cups', 'Fingers', 'Dual'],
                 [52, 54, 56, 58],
                 10,
                 'Nut-Travel distance [mm]'
                 )

    # ---- EQUATOR OFFSET ----
    mark10_plots(folder + 'experiment9_pullBack_fixedApple_equatorOffset/',
                 ['suction', 'fingers', 'dual'],
                 ['Suction cups', 'Fingers', 'Dual'],
                 [0, 5, 10, 15, 20],
                 10,
                 'Equator Offset [mm]'
                 )

    # ---- ANGLES ----
    mark10_plots(folder + 'experiment10_pullBack_angled/',
                 ['suction', 'fingers', 'dual'],
                 ['Suction cups', 'Fingers', 'Dual'],
                 [0, 15, 30, 45],
                 10,
                 'Angle between pulling force and gripper [deg]'
                 )

    plt.show()


def main():

    logging.getLogger('matplotlib.font_manager').disabled = True
    # Comment out this line for all DEBUG-level messages to be suppressed
    # logging.getLogger().setLevel(logging.DEBUG)

    # --- Simply choose the desired experiment by un-commenting it ---
    # circle_plots(1,1,1)
    # noise_experiments('horizontal')
    # noise_experiments('vertical')
    radius = 85 / 2000  # in meters! switch between 75mm and 85mm
    variable = 'pressure'  # switch between force, pressure and zforce
    # noise_experiments_pitch(exp_type='horizontal', radius=radius, variable=variable)
    # simple_suction_experiment()
    proxy_experiments()

    # real_experiments()

    # --- Build video from pngs and a plot beside of it with a vertical line running ---
    # plot_and_video()

    # TODO: Compare results between  get_detach_values() and get_forces_at_pick()
    # TODO: Interpret moments (lever = height of the rig)
    # TODO: Script ideas
    # TODO: Apple Pose w.r.t gripper analysis.


if __name__ == '__main__':
    # main()
    mark10_pullback_experiments()
    # orthogonal_load_cell_experiments()

