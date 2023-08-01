# Created by alejo at 7/27/23

import os
import json
import matplotlib.pyplot as plt
import pandas as pd



# Collect info from metadata json files

class stats_from_json():



# Collect info from data files (csvs)


# Step 1 - Collect Statistics from metadata json files
folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"
# dataset = "5th run - HIGH STIFFNESS - HIGH FORCE/"
dataset = "LOW_STIFFNESS/"
dataset = "MEDIUM_STIFFNESS/"
dataset = 'all_jsons_together/'


# https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
CRED = '\033[91m'
CGREEN = '\033[92m'
CEND = '\033[0m'

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
for filename in os.listdir(folder + dataset):

    if filename.endswith(".json") and not filename.startswith("vacuum_test"):
        print('\n' + CGREEN, filename, CEND)

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
            print(cnt)

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
            print("Pose, Stiffness, Force and Pick_Label: ", pose, stiffness, force, pick_label)

            cups_engagement[stiffness][force][str(cnt)] += 1
            pick_result[stiffness][force][pick_label] += 1

            # if general_info['yaw'] == 0 or general_info['yaw'] == -15:
            # if general_info['yaw'] == 60 or general_info['yaw'] == 45:
            # cups_engagement_pose[pose][stiffness] += cnt

            cups_engagement_pose[pose][stiffness] += success

            print(cups_engagement[stiffness])
            print(pick_result[stiffness])

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
plt.show()

