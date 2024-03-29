""" Handy to homogenize filenames"""
import os

# Paste files location
folder = 'C:/Users/avela/Dropbox/03 Temporal/03 Research/data/Mark10_experiments/'  # Personal Laptop

location = folder + 'experiment10_pullBack_angled/'


for f in os.listdir(location):

    if f.startswith('delta'):

        # read angle
        angle = f.split('_suction')[0]
        angle = angle.split('delta_')[1]

        # read repetition
        rep = f.split('.xlsx')[0]
        rep = rep.split('rep')[1]

        old_name = f
        new_name = 'exp(pullBack)_mode(suction)_angle(' + angle + ')_rep' + rep + '.xlsx'


        print('\nOld: ', old_name)
        print('New: ', new_name)

        os.rename(location + old_name, location + new_name)
