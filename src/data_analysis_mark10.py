# Created by alejo at 7/8/24

import os
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter, median_filter


################################ HANDY FUNCTIONS ##################################################
def mark10_plots(location, tags, gripper_modes, variable_list, reps, xlabel, plot_type='bandgap'):


    # Axes limits
    xmin = min(variable_list)
    xmax = max(variable_list)
    colors = ['green', 'red', 'blue', 'black']
    markers = ['o', 'v', '^', '<', '>']
    linestyles = ['dotted', 'dashdot', '--', 'solid']


    # Experiment number
    exp_number = location.split('/experiment')[1]
    exp_number = int(exp_number.split('_')[0])
    print('Experiment number: ', exp_number)

    # Lists to keep track of stuff
    stepses = []
    good_picks = []
    suction_picks = []
    max_forces_data = []

    # Create figure
    fig = plt.figure(figsize=(8, 6))
    fig = plt.figure(figsize=(6, 6))
    exp_prefix = 'loremipsum'

    for mode, tag, color, marker, style in zip(gripper_modes, tags, colors, markers, linestyles):
        print('\n########################## Mode: %s #########################' %(mode))
        stepses = []
        mean_max_forces = []
        sdv_max_forces = []

        for steps in variable_list:

            max_forces = []

            # Gather data from all repetitions of the same trial combination
            for rep in range(1, reps+1):

                # exp_prefix = 'delta_' + str(steps) + '_' + (tag) + '_rep' + str(rep)
                # exp_prefix = tag + '_dist_' + str(steps) + '_rep' + str(rep)

                # Concatenate filename depending on each experiment

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

                if exp_number == 11:
                    exp_prefix = 'exp(pullBackMoment)_mode(' + tag + ')_rep' + str(rep)

                max_pull = 'Nan'

                for file in sorted(os.listdir(location)):

                    if file.startswith(exp_prefix):

                        # Step 3: Open file and turn into dataframe
                        trial_df = pd.read_excel(location + file, index_col=0)

                        # Plot time series
                        # plt.plot(trial_df['Travel [mm]'], trial_df['Load [N]'])
                        # plt.title(file)
                        # plt.xlabel('Time [sec]')
                        # plt.ylabel('Force [N]')
                        # plt.grid()
                        # plt.show()

                        # Read minimum value and subtract initial one
                        max_pull = round(abs(trial_df['Load [N]'].iat[0] - min(trial_df['Load [N]'])),2)

                        # print('File: ', file)
                        # print('Max pull force: ', max_pull)
                        max_forces.append(max_pull)

                        # Keep track of successful pick values
                        if file.endswith('apple_picked).xlsx'):
                            good_picks.append(max_pull)

                        if tag == 'V' or tag == 'suction':
                            suction_picks.append(max_pull)

            print("\nList of Max forces", max_forces)

            stepses.append(steps)
            # stepses.append(steps/max(variable_list))

            mean_max_forces.append(round(abs(np.mean(max_forces)),2))
            sdv_max_forces.append(round(abs(np.std(max_forces)),2))

            max_forces_data.append(max_forces)

        print('--- Results summary of the actuation mode:---')
        print('Steps:', stepses)
        print('Mean Value:', mean_max_forces)
        print('SD:', sdv_max_forces)

        if plot_type == 'bandgap':
            # Plot each mode series with a band gap
            # if tag == 'suction' or tag == 'V':
            #     print('Suction Pick Forces:', suction_picks)
            #     mean_suction_force = np.mean(suction_picks)
            #     sdv_suction_force = np.std(suction_picks)
            #     lows = np.subtract(mean_suction_force, sdv_suction_force)
            #     highs = np.add(mean_suction_force, sdv_suction_force)
            #     plt.hlines(y=mean_suction_force, xmin=xmin, xmax=xmax, linestyles=style, lw=1, color=color,
            #                label='Suction cup force [N]')
            #     plt.fill_between(variable_list, lows, highs, alpha=.2)
            #
            # else:
            # if len(max_forces) > 0:
            lows = np.add(mean_max_forces, sdv_max_forces)
            highs = np.subtract(mean_max_forces, sdv_max_forces)
            plt.plot(stepses, mean_max_forces, linestyle=style, label=mode, color=color, marker=marker, lw=1)
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

    if plot_type == 'bandgap':
        # Plot median detachment force
        plt.hlines(y=16, xmin=xmin, xmax=xmax, linestyle='--', lw=2, label='Median FDF', color='k')

    # Read suction reference values
    if 'suction' not in tags:

        ### Step 1 - Data Location ###
        if os.name == 'nt':  # Windows OS
            storage = 'D:/'
        else:  # Ubuntu OS
            storage = '/media/alejo/Elements/'

        suction_folder = 'Alejo - Mark10 Gripper Tests/Mark10_experiments/suction_reference_values/'
        suction_folder = storage + suction_folder

        max_forces = []
        for file in sorted(os.listdir(suction_folder)):
            trial_df = pd.read_excel(suction_folder + file, index_col=0)
            # # Plot time series
            # plt.plot(trial_df['Travel [mm]'], trial_df['Load [N]'])
            # plt.plot(trial_df['Time'], trial_df['Load [N]'])
            # plt.title(file)
            # plt.grid()
            # plt.show()

            max_pull = abs(min(trial_df['Load [N]']))
            # print('Max pull force: ', max_pull)
            max_forces.append(max_pull)
        print('Suction Max Forces: ', max_forces)
        print('Suction Mean Max Forces: ', abs(np.mean(max_forces)))
        print('Suction Sdv Max Forces: ', abs(np.std(max_forces)))
        mean_pick_force = np.mean(max_forces)
        sdv_pick_force = np.std(max_forces)
        lows = np.subtract(mean_pick_force, sdv_pick_force)
        highs = np.add(mean_pick_force, sdv_pick_force)
        plt.hlines(y=mean_pick_force, xmin=xmin, xmax=xmax, linestyles='--', lw=1,
                   label='Suction-cups', color='blue')
        plt.fill_between(stepses, lows, highs, color='blue', alpha=.2)

        if 'dual' not in tags:
            # Combined Plots
            # This is just to make the plot more clear, and see both modes adding each other
            both_pick_force = np.add(mean_pick_force, mean_max_forces)
            both_sdv_force = np.add(sdv_pick_force, sdv_max_forces)
            lows = np.subtract(both_pick_force, both_sdv_force)
            highs = np.add(both_pick_force, both_sdv_force)
            plt.plot(stepses, both_pick_force, linestyle='dashdot', label='Dual', color='red', marker='v', lw=1)
            plt.fill_between(stepses, lows, highs, color='red', alpha=.2)


    if plot_type == 'barplot':
        array = np.array(max_forces_data, dtype=object)
        plt.boxplot(array, labels=gripper_modes)
    else:
        plt.legend()

    plt.grid()
    plt.ylim([0, 50])
    plt.xlabel(xlabel)
    plt.ylabel('Force [N]')
    # plt.title('Gripper pulling force [N] vs ' + xlabel)


def locate_index_of_deltas_v2(data, intercept=0.5):
    """
    Useful to locate in square signals, the index where the signal intersects a certain value
    """

    POIS_plus = []
    POIS_minus = []

    previous = data[0]

    for i in range(len(data)):

        if (previous < intercept) and (data[i] > intercept):
            POIS_plus.append(i)     # collects all points with positive gradient
        if (previous > intercept) and (data[i] < intercept):
            POIS_minus.append(i)    # collects all points with negative gradient

        previous = data[i]

    print('For the intercept: ', intercept)
    print('POIs of positive deltas: ', POIS_plus)
    print('POIs of negative deltas: ', POIS_minus)

    return POIS_plus, POIS_minus


############################### SCRIPTS FOR EACH EXPERIMENT ########################################
def orthogonal_load_cell_experiments(folder):

    # subfolder = 'experiment1_orthogonalLoad/'
    subfolder = 'experiment6_orthogonalLoad_accelStepper/'
    # subfolder = 'experiment12_orthogonalLoad/'

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


def push_load_cell_experiments(folder):

    # subfolder = 'experiment1_orthogonalLoad/'
    # subfolder = 'experiment6_orthogonalLoad_accelStepper/'
    subfolder = 'experiment12_orthogonalLoad/'

    location = folder + subfolder

    mean_max_forces = []
    std_max_forces = []
    all_fingers_max_vals = []
    max_ortho = 'Nan'

    fingers_data = []

    for file in sorted(os.listdir(location)):

        finger_max_vals = []

        if file.startswith('f'):
            print('\n\n' + file)

            # Step 3: Open file and turn into dataframe
            trial_df = pd.read_excel(location + file, index_col=0)
            load_list = trial_df['Load [N]'].tolist()

            fig = plt.figure(figsize=(8, 6))
            plt.plot(load_list, label='original data')
            filtered_data = median_filter(load_list, 20)
            plt.plot(filtered_data, label='filtered data')
            plt.legend()

            # Step 4: Check for POI's
            max_ortho = max(filtered_data)
            pois_pos, pois_neg = locate_index_of_deltas_v2(filtered_data, max_ortho*0.25)
            cycles = len(pois_pos)
            print('Number of cycles: ', cycles)

            # Step 5: Take the max values from each cycle
            n_points = trial_df.shape[0]

            for i in range(cycles):
                start = pois_pos[i]
                try:
                    end = pois_neg[i]
                except IndexError:
                    # Because some times the trials were cut in the middle
                    end = n_points

                max_val = max(filtered_data[start: end])
                finger_max_vals.append(max_val)
                all_fingers_max_vals.append(max_val)

            print('Finger Max-values', finger_max_vals)
            print('All fingers accumulated Max-values', all_fingers_max_vals)


        fingers_data.append(finger_max_vals)

        fig = plt.figure(figsize=(6, 6))
        plt.boxplot(finger_max_vals)

    print('Mean and Deviation of all fingers:', round(np.mean(all_fingers_max_vals),1), round(np.std(all_fingers_max_vals),1))

    fingers_data.append(all_fingers_max_vals)

    # Convert into array to circumvent the list size misalignment
    array = np.array(fingers_data, dtype=object)

    fig = plt.figure(figsize=(6, 6))
    plt.boxplot(array, labels=['A', 'B', 'C', 'All'])
    plt.xlabel('Finger')
    plt.ylabel('Force [N]')
    # plt.title('Normal Force from each finger [N]')

    # Plot the apple bruising threshold
    thr_press = 0.29e6    # Pa (Li et al. 2016)
    finger_width = 20   # mm
    thr_force = thr_press * (10 ** 2)/1e6
    print(thr_force)
    # plt.hlines(y=thr_force, xmin=1285, xmax=1425, linestyles='--', lw=2, label='Apple Bruising threshold')

    # plt.hlines(y=thr_force, xmin=52, xmax=60, linestyles='--', lw=2, label='Apple Bruising threshold')
    plt.ylim([0, 50])
    plt.grid()

    # plt.show()


def mark10_pullback_experiments(folder):


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

    # # ---- CLAMPING SPEED ----
    # mark10_plots(folder + 'experiment8_pullBack_fixedApple_fingerSpeed/',
    #              ['fingers', 'dual'],
    #              ['Fingers', 'Dual'],
    #              [140, 190, 240, 290, 340],
    #              10,
    #              'Nut-Travel speed [rpm]'
    #              )
    #
    # ---- NUT TRAVEL DISTANCE ----
    mark10_plots(folder + 'experiment7_pullBack_fixedApple_fingerDistance/',
                 ['fingers', 'dual'],
                 ['Fingers', 'Dual'],
                 [52, 54, 56, 58],
                 10,
                 'Nut-travel distance [mm]'
                 )

    # ---- EQUATOR OFFSET ----
    mark10_plots(folder + 'experiment9_pullBack_fixedApple_equatorOffset/',
                 # ['fingers', 'dual'],
                 ['fingers'],
                 # ['Fingers', 'Dual'],
                     ['Fingers'],
                 [0, 5, 10, 15, 20],
                 10,
                 'Fruit equator offset [mm]'
                 )

    # ---- ANGLES ----
    mark10_plots(folder + 'experiment10_pullBack_angled/',
                 ['fingers', 'dual', 'suction'],
                 ['Fingers', 'Dual', 'Suction cups'],
                 [0, 15, 30, 45],
                 10,
                 '$\omega$ [deg]'
                 )

    # ---- MOMENTS ----
    mark10_plots(folder + 'experiment11_pullBack_moment/',
                 ['suction', 'fingers', 'dual'],
                 ['Suction cups', 'Fingers', 'Dual'],
                 [0],
                 10,
                 'Actuation mode',
                 plot_type='barplot'
                 )

    # plt.show()


if __name__ == '__main__':

    ### Adjust plot parameters (for papers) ###
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]
    plt.rcParams["font.size"] = 16
    plt.rcParams["font.weight"] = 'light'
    plt.rc('legend', fontsize=14)  # using a size in points

    ### Step 1 - Data Location ###
    if os.name == 'nt':     # Windows OS
        storage = 'D:/'
    else:                   # Ubuntu OS
        storage = '/media/alejo/Elements/'

    folder = 'Alejo - Mark10 Gripper Tests/Mark10_experiments/'
    folder = storage + folder

    ### Step 2 - Subfunctions ###
    # orthogonal_load_cell_experiments(folder)
    push_load_cell_experiments(folder)
    mark10_pullback_experiments(folder)

    plt.show()