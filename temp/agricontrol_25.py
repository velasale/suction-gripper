import pandas as pd
import numpy as np
import os
from src.data_analysis_robot import ur5e_fk_dh
from matplotlib import pyplot as plt
import math


# Function to search for the subfolder recursively and open joint_states.csv
def find_subfolder_and_open_file(base_dir, target_folder_name):
    # Walk through the directory tree
    for root, dirs, files in os.walk(base_dir):
        # Check if the target folder is in the current directory
        if target_folder_name in dirs:
            subfolder_path = os.path.join(root, target_folder_name)  # Full path to the found subfolder
            joint_states_file = os.path.join(subfolder_path, 'joint_states.csv')

            # Check if 'joint_states.csv' exists in that subfolder
            if os.path.isfile(joint_states_file):
                # print(f"Found 'joint_states.csv' in {subfolder_path}")
                # Open the 'joint_states.csv' file (you can load it into a pandas DataFrame or just print it)
                joint_states_df = pd.read_csv(joint_states_file)
                # print(joint_states_df)  # Print the contents of the 'joint_states.csv' file
                return joint_states_df
    return False


def rotation_matrix(alpha, beta, gamma):
    """ Obtain the rotation matrix given, the alpha, beta and gamma"""

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

    return Rot_M


# def compute_jacobian()


def main():

    # --- Step 1: Only consider Useful Data ---
    # Note: Useful data is in the csv file 'pressure_servo_folders_labels'
    base_directory ="C:/Users/avela/OneDrive/Documents/01 Research/Data/"
    file_path = base_directory + "pressure_servo_folders_labels.csv"
    df = pd.read_csv(file_path)

    # Filter 1.1: Use only useful data
    # Filter rows where 'Data Useful' column is 'yes'. Some data is not useful for different reasons
    useful_df = df[df['Data Useful?'] == 'yes']

    # Filter 1.2: Other filters
    useful_df = useful_df[useful_df['gripper pose success'] == 'yes']
    # useful_df = useful_df[useful_df['Cause / Effect'] == 'SW - Pressure Servoing: Singularity reached']

    # --- Step 2: Find the initial pose angle
    swept_angles = []       # Book keeping of the angle swept while pressure servoing
    for bagfile_name in useful_df['bagfile']:

        # Open 'joint_states' topic
        print(bagfile_name)
        joint_states_df = find_subfolder_and_open_file(base_directory, bagfile_name)

        time_stamp = joint_states_df['header_stamp_secs'] + 1e-9 * joint_states_df['header_stamp_nsecs']
        elapsed_time_stamp = time_stamp - time_stamp.iloc[0]
        elapsed_time_stamp = elapsed_time_stamp.tolist()
        j0_shoulder_pan_values = joint_states_df['position_5'].tolist()
        j1_shoulder_lift_values = joint_states_df['position_0'].tolist()
        j2_elbow_values = joint_states_df['position_1'].tolist()
        j3_wrist1_values = joint_states_df['position_2'].tolist()
        j4_wrist2_values = joint_states_df['position_3'].tolist()
        j5_wrist3_values = joint_states_df['position_4'].tolist()

        # --- Forward Kinematics ----
        x = []
        y = []
        z = []
        beta = []
        alpha = []
        gamma = []
        for i in range(len(elapsed_time_stamp)):
            joints = np.array([j0_shoulder_pan_values[i],
                               j1_shoulder_lift_values[i],
                               j2_elbow_values[i],
                               j3_wrist1_values[i],
                               j4_wrist2_values[i],
                               j5_wrist3_values[i]])

            eef_position, eef_orientation = ur5e_fk_dh(joints)

            # Store eef_positions and orientations for each timestamp
            x.append(eef_position[0])
            y.append(eef_position[1])
            z.append(eef_position[2])

            beta.append(eef_orientation[0])
            alpha.append(eef_orientation[1])
            gamma.append(eef_orientation[2])

        # --- Angular delta: Dot Product of the z-axis at the beginning and at the end
        plot_factor = 0.1  # to plot vectors with about 10cm length
        z_axis = np.array([[0, 0, 1, 1]]).transpose()
        y_axis = np.array([[0, 1, 0, 1]]).transpose()
        x_axis = np.array([[1, 0, 0, 1]]).transpose()

        # Pose before Pressure Servoing
        alpha_angle = gamma[0]
        beta_angle = beta[0]
        gamma_angle = alpha[0]

        Rot_M = rotation_matrix(alpha_angle, beta_angle, gamma_angle)
        eef_z_vector_begin = np.dot(Rot_M, z_axis)
        eef_z_vector_begin = plot_factor * eef_z_vector_begin / np.linalg.norm(eef_z_vector_begin)
        eef_y_vector_begin = np.dot(Rot_M, y_axis)
        eef_y_vector_begin = plot_factor * eef_y_vector_begin / np.linalg.norm(eef_y_vector_begin)
        eef_x_vector_begin = np.dot(Rot_M, x_axis)
        eef_x_vector_begin = plot_factor * eef_x_vector_begin / np.linalg.norm(eef_x_vector_begin)

        # Pose after Pressure Servoing
        alpha_angle = gamma[-1]
        beta_angle = beta[-1]
        gamma_angle = alpha[-1]

        Rot_M = rotation_matrix(alpha_angle, beta_angle, gamma_angle)
        eef_z_vector_end = np.dot(Rot_M, z_axis)
        eef_z_vector_end = plot_factor * eef_z_vector_end / np.linalg.norm(eef_z_vector_end)
        eef_y_vector_end = np.dot(Rot_M, y_axis)
        eef_y_vector_end = plot_factor * eef_y_vector_end / np.linalg.norm(eef_y_vector_end)
        eef_x_vector_end = np.dot(Rot_M, x_axis)
        eef_x_vector_end = plot_factor * eef_x_vector_end / np.linalg.norm(eef_x_vector_end)

        # Swept angle while pressure servoing
        magnitude_begin = np.linalg.norm(eef_z_vector_begin)
        magnitude_end = np.linalg.norm(eef_z_vector_end)
        cos_swept_angle = np.dot(eef_z_vector_begin.flatten(), eef_z_vector_end.flatten()) / (magnitude_end * magnitude_begin)
        swept_angle_radians = np.arccos(cos_swept_angle)
        swept_angle_degrees = math.degrees(swept_angle_radians)
        print(f'Swept angle [deg]: {swept_angle_degrees}\n')

        swept_angles.append(swept_angle_degrees)

        # --- Plots ---
        # 3D Plot of eef trajectory
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_title('End Effector Trajectory\n' + bagfile_name)
        ax.set_xlabel('x[m]')
        ax.set_ylabel('y[m]')
        ax.set_zlabel('z[m]')
        ax.text(x[0], y[0], z[0], "Start", color='green')
        ax.text(x[-1], y[-1], z[-1], "End", color='red')
        ax.plot3D(x, y, z, 'black', linewidth='2', linestyle='dotted')

        # Plot eef frame
        ax.quiver(x[0], y[0], z[0], eef_z_vector_begin[0], eef_z_vector_begin[1], eef_z_vector_begin[2], length=1, color='blue')
        ax.quiver(x[0], y[0], z[0], eef_y_vector_begin[0], eef_y_vector_begin[1], eef_y_vector_begin[2], length=1, color='green')
        ax.quiver(x[0], y[0], z[0], eef_x_vector_begin[0], eef_x_vector_begin[1], eef_x_vector_begin[2], length=1, color='red')
        ax.quiver(x[-1], y[-1], z[-1], eef_z_vector_end[0], eef_z_vector_end[1], eef_z_vector_end[2], length=1, color='blue')
        ax.quiver(x[-1], y[-1], z[-1], eef_y_vector_end[0], eef_y_vector_end[1], eef_y_vector_end[2], length=1, color='green')
        ax.quiver(x[-1], y[-1], z[-1], eef_x_vector_end[0], eef_x_vector_end[1], eef_x_vector_end[2], length=1, color='red')

        # Make 3dplot have same aspect ratio
        x_maxes = max(x)
        x_mins = min(x)
        y_maxes = max(y)
        y_mins = min(y)
        z_maxes = max(z)
        z_mins = min(z)
        max_range = np.array([x_maxes - x_mins, y_maxes - y_mins, z_maxes - z_mins]).max() * 0.75
        mid_x = (x_maxes + x_mins) * 0.5
        mid_y = (y_maxes + y_mins) * 0.5
        mid_z = (z_maxes + z_mins) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Boxplot of swept angles
    fig = plt.figure()
    plt.hist(swept_angles)
    plt.ylabel('Swept angle [deg]')
    plt.title('Unsuccessful pose servoing')

    plt.show()



if __name__ == '__main__':

    main()