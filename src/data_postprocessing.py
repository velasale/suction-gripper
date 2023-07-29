# Created by alejo at 7/27/23

import os
import json
import matplotlib.pyplot as plt
import pandas as pd

# Step 1 - Collect Statistics from metadata json files
folder = "/home/alejo/Documents/data/SUCTION_GRIPPER_EXPERIMENTS/"
# dataset = "5th run - HIGH STIFFNESS - HIGH FORCE/"
dataset = "all_jsons_together/"


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

    if filename.endswith(".json"):
        print('\n', filename)

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
            if pick_label == 'a':
                 pick_label = "Successful pick: after pick pattern"
            elif pick_label == 'b':
                pick_label = "Un-successful: apple picked but fell afterwards"
            elif pick_label == 'c':
                pick_label = "Un-successful: apple not picked"
            elif pick_label == 'd':
                pick_label = "Successful pick: before pick pattern"

            stiffness = proxy_info['branch stiffness'] + "_stiffness"
            force = proxy_info['stem force'] + "_force"
            print(stiffness, force)

            cups_engagement[stiffness][force][str(cnt)] += 1
            pick_result[stiffness][force][pick_label] += 1

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
plt.show()

