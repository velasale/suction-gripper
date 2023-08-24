# Created by alejo at 7/27/23

import os
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns


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
    df = pd.read_csv('suction_gripper_df.csv', index_col=0)

    # # ---- Apply Filters
    # variable = 'cups engaged'
    # x_offsets = [0.00, 0.005, 0.010, 0.015, 0.02]
    # x_offsets_mm = [0, 5, 10, 15, 20]
    #
    # yaws = [-15, 45]
    #
    # for yaw in yaws:
    #     means = []
    #     stds = []
    #     for i in x_offsets:
    #         filter = (df['yaw'] == yaw) & (df['x_offset'] == i)
    #         filtered_df = df[filter]
    #         mean = filtered_df[variable].mean()
    #         std = filtered_df[variable].std()
    #         means.append(mean)
    #         stds.append(std)
    #
    #     print(means, stds)
    #     # plt.errorbar(x_offsets_mm, means, stds, linestyle='None', marker='^')
    #     plt.plot(x_offsets_mm, means, 'o-', linestyle='dashed')
    # plt.show()

    # ---- Apply Filters
    variable = 'cup engagement'
    series_name = 'x_offset'
    series = [0.00, 0.005, 0.010, 0.015, 0.02]
    series_x_ticks = [0, 5, 10, 15, 20]

    series_name = 'sampling point'
    series = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    series_x_ticks = [0, 1, 2, 3, 4, 5, 6, 7, 8]

    yaws = [-15, 45]

    for yaw in yaws:
        values = []

        for i in series:
            filter = (df['yaw'] == yaw) & (df[series_name] == i)
            filtered_df = df[filter]
            # --- Count
            cnt = 0
            for j in filtered_df[variable]:
                if j == '2 or 3':
                    cnt += 1

            try:
                value = cnt / len(filtered_df) * 100
            except ZeroDivisionError:
                value = 0

            values.append(value)

        print(values)
        # plt.errorbar(x_offsets_mm, means, stds, linestyle='None', marker='^')
        plt.plot(series_x_ticks, values, 'o-', linestyle='dashed')
    plt.show()





if __name__ == '__main__':
    main()
    # bubble_chart()