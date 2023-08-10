# Created by alejo at 7/27/23

import os
import json
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


def dict_from_jsons(folder, dataset):

    # https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
    CRED = '\033[91m'
    CGREEN = '\033[92m'
    CEND = '\033[0m'


    experiment_counter = {'low_stiffness': {'low_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0},
                                         'medium_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0},
                                         'high_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0}},
                       'medium_stiffness': {'low_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0},
                                         'medium_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0},
                                         'high_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0}},
                       'high_stiffness': {'low_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0},
                                         'medium_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0},
                                         'high_force': {0: 0, 0.005: 0, 0.01: 0, 0.015: 0, 0.02: 0}}}

    cups_engagement_pose =    {0: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               1: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               2: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               3: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               4: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               5: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               6: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               7: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               8: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               9: {'low_stiffness': 0,
                                   'medium_stiffness': 0,
                                   'high_stiffness': 0},
                               }

    # Step 2 - Things to plot
    cups_engagement = {'low_stiffness': {'low_force': {'0': 0, '1': 0, '2': 0, '3': 0},
                                         'medium_force': {'0': 0, '1': 0, '2': 0, '3': 0},
                                         'high_force': {'0': 0, '1': 0, '2': 0, '3': 0}},
                       'medium_stiffness': {'low_force': {'0': 0, '1': 0, '2': 0, '3': 0},
                                            'medium_force': {'0': 0, '1': 0, '2': 0, '3': 0},
                                            'high_force': {'0': 0, '1': 0, '2': 0, '3': 0}},
                       'high_stiffness': {'low_force': {'0': 0, '1': 0, '2': 0, '3': 0},
                                          'medium_force': {'0': 0, '1': 0, '2': 0, '3': 0},
                                          'high_force': {'0': 0, '1': 0, '2': 0, '3': 0}}}

    pick_result     = {'low_stiffness': {'low_force':       {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0},
                                         'medium_force':    {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0},
                                         'high_force':      {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0}},
                       'medium_stiffness': {'low_force':    {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0},
                                         'medium_force':    {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0},
                                         'high_force':      {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0}},
                       'high_stiffness': {'low_force':      {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0},
                                         'medium_force':    {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0},
                                         'high_force':      {'Successful pick: after pick pattern': 0,
                                                             "Successful pick: before pick pattern": 0,
                                                             "Un-successful: apple picked but fell afterwards": 0,
                                                             "Un-successful: apple not picked": 0}}}

    # Step 3 - Gather info
    exp = 1
    for filename in os.listdir(folder + dataset):

        if filename.endswith(".json") and not filename.startswith("vacuum_test"):
            print('\n' + CGREEN + "Experiment # %i" %exp)
            print(filename, CEND)
            exp += 1

            with open(folder + dataset + filename, 'r') as json_file:
                json_dict = json.load(json_file)

                # Extract dictionaries
                general_info = json_dict['general']
                robot_info = json_dict['robot']
                proxy_info = json_dict['proxy']
                labels = json_dict['labels']

                # Count number of cups engaged
                cups_engaged = (labels['suction cup a'], labels['suction cup b'], labels['suction cup c'])
                cnt = 0
                for cup in cups_engaged:
                    if cup == 'yes':
                        cnt += 1
                print('Cups engaged: ', cnt)

                # Pick result
                pick_label = labels['apple pick result']
                success = 0
                if pick_label == 'a':
                     pick_label = "Successful pick: after pick pattern"
                     success = 1
                elif pick_label == 'b':
                    pick_label = "Un-successful: apple picked but fell afterwards"
                    success = 0
                elif pick_label == 'c':
                    pick_label = "Un-successful: apple not picked"
                    success = 0
                elif pick_label == 'd':
                    pick_label = "Successful pick: before pick pattern"
                    success = 1
                    # input('Hit enter to continue')

                # Pose
                pose = general_info['sampling point']
                stiffness = proxy_info['branch stiffness'] + "_stiffness"
                force = proxy_info['stem force'] + "_force"
                x_noise = robot_info['position noise command [m]'][0]
                print("x_noise = %f, Pose = %i, Stiffness = %s, Force = %s and Pick_Label = %s" % (x_noise, pose, stiffness, force, pick_label))

                cups_engagement[stiffness][force][str(cnt)] += 1
                pick_result[stiffness][force][pick_label] += 1

                experiment_counter[stiffness][force][x_noise] += 1

                # if general_info['yaw'] == 0 or general_info['yaw'] == -15:
                # if general_info['yaw'] == 60 or general_info['yaw'] == 45:
                # cups_engagement_pose[pose][stiffness] += cnt

                cups_engagement_pose[pose][stiffness] += success

                # print(cups_engagement[stiffness])
                # print(pick_result[stiffness])
                print(experiment_counter[stiffness])


    # Step 4 - Plot results
    pd.DataFrame(cups_engagement[stiffness]).T.plot(kind='bar')
    plt.xticks(rotation=0)
    plt.xlabel("Stem Force")
    plt.ylabel("Count")
    plt.ylim([0, 25])
    plt.title("Number of Suction Cups engaged")

    pd.DataFrame(pick_result[stiffness]).T.plot(kind='bar')
    plt.xticks(rotation=0)
    plt.xlabel("Stem Force")
    plt.ylabel("Count")
    plt.ylim([0, 25])
    plt.title("Apple Pick Result")

    pd.DataFrame(cups_engagement_pose).T.plot(kind='bar')
    plt.xticks(rotation=0)
    plt.xlabel("Pose")
    plt.ylabel("Count")
    plt.ylim([0, 8])
    plt.title("Succesful picks at each pose (0: underneath, 9: 135 deg ")
    # plt.title("Cups Engagement at each pose (0: underneath, 9: 135 deg ")
    plt.axhline(y=6, color='black', linestyle='--')

    pd.DataFrame(experiment_counter['high_stiffness']).T.plot(kind='bar')
    plt.xticks(rotation=0)
    plt.xlabel("Pose")
    plt.ylabel("Count")
    plt.ylim([0, 80])
    plt.title("Number of experiments ")
    # plt.title("Cups Engagement at each pose (0: underneath, 9: 135 deg ")
    plt.axhline(y=18, color='black', linestyle='--')

    # plt.show()


def df_from_jsons(folder, dataset):

    # https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
    CRED = '\033[91m'
    CGREEN = '\033[92m'
    CEND = '\033[0m'

    # --- Create DataFrame ---
    COLUMN_NAMES = ['file', 'sampling point', 'stiffness', 'strength', 'yaw', 'x_offset', 'cup_a', 'cup_b', 'cup_c',
                    'cups engaged', 'result']
    df = pd.DataFrame(columns=COLUMN_NAMES)

    exp = 0
    for filename in os.listdir(folder + dataset):

        if filename.endswith(".json") and not filename.startswith("vacuum_test"):
            # print('\n' + CGREEN + "Experiment # %i" %exp)
            # print(filename, CEND)
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
                stiffness = proxy_info['branch stiffness'] + '_stiffness'
                strength = proxy_info['stem force'] + '_strength'
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
                           'cups engaged': cnt}
                df.loc[len(df)] = new_row

    return(df)


def main():

    # ------ Dataset Location ------
    # folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"
    folder = '/home/alejo/Documents/data/suction_gripper/'
    # dataset = "MEDIUM_STIFFNESS/"
    # dataset = 'all_jsons_together/'
    dataset = ''

    # Collect info from metadata json files    #
    # class stats_from_json():
    # Collect info from data files (csvs)
    # Step 1 - Collect Statistics from metadata json files

    # dict_approach(folder, dataset)
    df = df_from_jsons(folder, dataset)
    df.to_csv('suction_gripper_df.csv')

    cup_counts = (df.groupby(['cups engaged'])['sampling point']
                  .value_counts(normalize=True)
                  .rename('percentage')
                  .mul(100)
                  .reset_index()
                  .sort_values('sampling point'))


    # df.value_counts(normalize=True)
    sns.barplot(x='sampling point', y='percentage', hue='cups engaged', data=cup_counts)
    # sns.boxplot(x='sampling point', y='cups engaged', data=df)

    plt.show()


if __name__ == '__main__':
    main()
