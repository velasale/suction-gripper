# Created by alejo at 7/27/23

import os
import json
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import matplotlib.image as image
from matplotlib import rc
import numpy as np
import pandas as pd
import seaborn as sns
import itertools


def rename_level(level):

    if level == 'low':
        level = '1_low'
    elif level == 'medium':
        level = '2_medium'
    elif level == 'high':
        level = '3_high'

    return level


def df_from_jsons(folder, dataset):

    # https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
    CRED = '\033[91m'
    CGREEN = '\033[92m'
    CEND = '\033[0m'

    # --- Create DataFrame ---
    COLUMN_NAMES = ['file', 'sampling point', 'stiffness', 'strength', 'yaw', 'x_offset', 'cup_a', 'cup_b', 'cup_c',
                    'cups engaged', 'result', 'cup engagement']
    df = pd.DataFrame(columns=COLUMN_NAMES)

    exp = 0
    for filename in os.listdir(folder + dataset):

        if filename.endswith(".json") and not filename.startswith("vacuum_test"):
            print('\n' + CGREEN + "Experiment # %i" %exp)
            print(filename, CEND)
            exp += 1

            with open(folder + dataset + filename, 'r') as json_file:

                # Extract dictionaries
                json_dict = json.load(json_file)
                general_info = json_dict['general']
                robot_info = json_dict['robot']
                proxy_info = json_dict['proxy']
                labels = json_dict['labels']

                # Extract values
                sampling_point = general_info['sampling point']
                x_offset = robot_info['position noise command [m]'][0]
                yaw = general_info['yaw']
                stiffness = rename_level(proxy_info['branch stiffness']) + '_stiffness'
                strength = rename_level(proxy_info['stem force']) + '_strength'
                cup_a = labels['suction cup a']
                cup_b = labels['suction cup b']
                cup_c = labels['suction cup c']
                result = labels['apple pick result']

                # Rename result
                if result == 'a':
                     result = "Good Pick"
                elif result == 'b':
                    result = "Bad Pick"
                elif result == 'c':
                    result = "Bad Pick"
                elif result == 'd':
                    result = "Good Pick"

                # Count number of cups engaged
                cnt = 0
                if cup_a == 'yes':
                    cnt += 1
                if cup_b == 'yes':
                    cnt += 1
                if cup_c == 'yes':
                    cnt += 1

                if cnt > 1:
                    cup_engagement = "2 or 3"
                else:
                    cup_engagement = "0 or 1"

                # Build row and add it to DataFrame
                new_row = {'file': filename,
                           'stiffness': stiffness,
                           'strength': strength,
                           'yaw': yaw,
                           'x_offset': x_offset,
                           'cup_a': cup_a,
                           'cup_b': cup_b,
                           'cup_c': cup_c,
                           'result': result,
                           'sampling point': sampling_point,
                           'cups engaged': cnt,
                           'cup engagement': cup_engagement}
                df.loc[len(df)] = new_row

    return(df)


def bubble_chart(df):

    x = ["l", "l", "l",
         "m", "m", "m",
         "h", "h", "h"]

    y = ["l", "m", "h",
         "l", "m", "h",
         "l", "m", "h"]

    z = [74,7,0,67,2,2,71,9,2]

    # create pandas dataframe
    data_list = pd.DataFrame(
        {'x_axis': df['stiffness'],
         'y_axis': df['strength'],
         'category': df['result']
         })

    data_list = pd.DataFrame(
        {'x_axis': x,
         'y_axis': y,
         'category': z
         })

    # change size of data points
    minsize = min(data_list['category'])
    maxsize = max(data_list['category'])

    # use the scatterplot function to build the bubble map
    sns.scatterplot(data=data_list, x="x_axis", y="y_axis", size="category", legend=False, sizes=(0, 500))

    # show the graph
    plt.show()


def df_numerical_stats(df, num_value, x_filter_name, x_filter_values, series_name, series_values):
    """

    @param df: Dataframe
    @param num_value: numerical value of which statistics are obtained
    @param x_filter_name:
    @param x_filter_values:
    @param series_name:
    @param series_values:
    @return:
    """

    plt.figure()
    plt.grid()
    plt.ylim([0, 3])

    print(df.sum())

    for serie in series_values:
        means = []
        stds = []
        for x_filter in x_filter_values:
            filter = (df[series_name] == serie) & (df[x_filter_name] == x_filter)
            filtered_df = df[filter]
            mean = filtered_df[num_value].mean()
            std = filtered_df[num_value].std()
            means.append(mean)
            stds.append(std)

        print(means, stds)
        # plt.errorbar(x_filter_values, means, stds)
        plt.plot(x_filter_values, means, 'o-', linestyle='dashed', label=series_name + " = " + str(serie))
        plt.ylabel('Mean of Number of Cups engaged')
        plt.xticks(x_filter_values)
        plt.xlabel(x_filter_name)
        plt.legend()


def df_categorical_stats(df, cat_name, cat_value, x_filter_name, x_filter_values, x_filter_ticks, series_name, series_values, series_labels):
    """

    @param df: Dataframe
    @param cat_name: Name of the categorical variable
    @param cat_value: Desired value of categorical value
    @param x_filter_name:
    @param x_filter_values:
    @param x_filter_ticks:
    @param series_name:
    @param series_values:
    @param series_labels:
    @return:
    """

    plt.figure(figsize=(6, 4), dpi=80)
    plt.rc('font', family='serif')      # Font similar to Latex
    plt.grid()
    plt.ylim([0, 105])
    marker = itertools.cycle(('x', '+', '.', 'o', '*'))

    if series_name == 'none':
        series_values = ['']
        series_labels = ['']

    for serie, label in zip(series_values, series_labels):
        values = []

        for x_filter in x_filter_values:

            if series_name == 'none':
                filter = df[x_filter_name] == x_filter

            else:
                filter = (df[series_name] == serie) & (df[x_filter_name] == x_filter)

            filtered_df = df[filter]

            # --- Count ---
            cnt = 0
            for j in filtered_df[cat_name]:
                if j == cat_value:
                    cnt += 1

            try:
                value = cnt / len(filtered_df) * 100
            except ZeroDivisionError:
                value = 0

            values.append(value)

        if series_name == 'yaw':
            file = "../art/delta.png"
            logo = image.imread(file)

        plt.plot(x_filter_values, values, marker=next(marker), linestyle='dashed', label=(series_name + " = " + str(label)))
        plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(xmax=100.0))
        plt.ylabel('Grasps with 2 or 3 cups engaged [%]', fontsize=14)
        # plt.ylabel('Good Apple picks [%]', fontsize=14)
        plt.xticks(x_filter_values, x_filter_ticks)
        plt.xlabel(x_filter_name, fontsize=14)
        # plt.legend(bbox_to_anchor = (1, 1), loc = "upper left")
        plt.legend()


def main():

    # ------ Dataset Location ------
    # folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"
    # folder = '/home/alejo/Documents/research/data/suction_gripper/'
    # --- Lab's laptop ----
    folder = '/home/alejo/Dropbox/03 Temporal/data/suction_gripper/'
    # dataset = "MEDIUM_STIFFNESS/"
    dataset = ''

    # ------ Create or Read DataFrame with all the json files ------

    # df = df_from_jsons(folder, dataset)
    # df.to_csv('suction_gripper_df.csv')
    df = pd.read_csv('../data/suction_gripper_df.csv', index_col=0)
    print(df.dtypes)

    # ------ Plot results
    # df_numerical_stats(df, 'cups engaged',
    #                    'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02],
    #                    'yaw', [-15, 45])
    #
    # yaw_filter = df['yaw'] == 45
    # df_numerical_stats(df[yaw_filter], 'cups engaged',
    #                    'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02],
    #                    'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'])
    #
    # yaw_filter = df['yaw'] == -15
    # df_numerical_stats(df[yaw_filter], 'cups engaged',
    #                    'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02],
    #                    'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'])
    #
    # df_numerical_stats(df, 'cups engaged',
    #                    'sampling point', [0, 1, 2, 3, 4, 5, 6, 7, 8],
    #                    'yaw', [-15, 45])
    #
    # df_numerical_stats(df, 'cups engaged',
    #                    'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'],
    #                    'yaw', [-15, 45])
    #
    # df_numerical_stats(df, 'cups engaged',
    #                    'sampling point', [0, 1, 2, 3, 4, 5, 6, 7, 8],
    #                    'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02])
    #

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    yaw_filter = df['yaw'] == 45
    df_categorical_stats(df, 'cup engagement', '2 or 3',
                         'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02], [0, 5, 10, 15, 20],
                         'yaw', [-15, 45], ['nabla', 'delta'])

    df_categorical_stats(df[yaw_filter], 'cup engagement', '2 or 3',
                         'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02], [0, 5, 10, 15, 20],
                         'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'], ['Low', 'Medium', 'High'])

    yaw_filter = df['yaw'] == -15
    df_categorical_stats(df[yaw_filter], 'cup engagement', '2 or 3',
                         'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02], [0, 5, 10, 15, 20],
                         'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'],
                         ['Low', 'Medium', 'High'])

    df_categorical_stats(df, 'cup engagement', '2 or 3',
                       'sampling point', [0, 1, 2, 3, 4, 5, 6, 7, 8], [0, 15, 30, 45, 60, 75, 90, 105, 120],
                       'yaw', [-15, 45], ['nabla', 'delta'])

    df_categorical_stats(df, 'cup engagement', '2 or 3',
                         'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'], ['Low Stiffness', 'Medium Stiffness', 'High Stiffness'],
                         'yaw', [-15, 45], ['nabla', 'delta'])

    yaw_filter = df['yaw'] == 45
    df_categorical_stats(df[yaw_filter], 'cup engagement', '2 or 3',
                         'sampling point', [0, 1, 2, 3, 4, 5, 6, 7, 8], [0, 15, 30, 45, 60, 75, 90, 105, 120],
                         'x_offset', [0.00, 0.005, 0.010, 0.015, 0.02], ['0mm', '5mm', '10mm', '15mm', '20mm'])

    strength_filter = df['strength'] == '1_low_strength'
    df_categorical_stats(df[strength_filter], 'result', 'Good Pick',
                         'cups engaged', [0, 1, 2, 3], [0, 1, 2, 3],
                         'stiffness', ['1_low_stiffness', '2_medium_stiffness', '3_high_stiffness'],
                         ['Low', 'Medium', 'High'])
                        # 'none', [''], [''])
    plt.show()


if __name__ == '__main__':
    main()
    # bubble_chart()