# @Time : 6/6/2023 11:15 AM
# @Author : Alejandro Velasquez

## --- Standard Library Imports
import os
import time
import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.animation as animation
import matplotlib.cm as cm
from matplotlib.gridspec import GridSpec


def running_plot_and_video(location, filename, experiment, xlabel='Elapsed time [sec]', ylabel='Pressure [kPa]'):
    """
    Plots a graph with a vertical line running while playing a video given images
    @param location: Folder
    @param filename: Subfolder with the jpeg/png files
    @param time_list: data for plot's x-axis
    @param values_list: data for plot's y-axis
    @return:

    Ref:https://stackoverflow.com/questions/61808191/is-there-an-easy-way-to-animate-a-scrolling-vertical-line-in-matplotlib
    """

    time_list = experiment.pressure_sc1_elapsed_time
    time_list_2 = experiment.pressure_sc2_elapsed_time
    time_list_3 = experiment.pressure_sc3_elapsed_time
    values_list = experiment.pressure_sc1_values
    values_list_2 = experiment.pressure_sc2_values
    values_list_3 = experiment.pressure_sc3_values

    force_time_list = experiment.wrench_elapsed_time
    net_force_values_list = experiment.wrench_sumforce_relative_values

    # --- Sort png files in a list
    lst = os.listdir(location + filename + '/pngs_fixed_cam')
    listop = []
    for i in lst:
        if i.endswith('.png'):
            x = i.split('.png')[0]
            listop.append(int(x))
    listop.sort()

    # --- Pressure Plots
    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(12, 4.8))

    ax[0].plot(time_list, values_list, 'red', linewidth=2, label='cup A')
    ax[0].plot(time_list_2, values_list_2, 'green', linewidth=2, label='cup B')
    ax[0].plot(time_list_3, values_list_3, 'blue', linewidth=2, label='cup C')
    ax[0].set_xlim(0, max(time_list))
    ax[0].set_ylim(0, 120)
    ax[0].grid()
    ax[0].set_xlabel(xlabel)
    ax[0].set_ylabel(ylabel)
    ax[0].legend()

    # --- Force Plots
    ax_1 = ax[0].twinx()       # same plot, sharing the x axis, but different y-axis
    ax_1.plot(force_time_list, net_force_values_list, 'k-', linestyle='dashed', linewidth=2, label='net force')
    ax_1.set_ylim(0, 15)
    ax_1.set_ylabel("Force [N]")
    ax_1.legend()

    plt.ion()
    plt.show()

    # --- Remove details from ax[1] because we are displaying only the image
    ax[1].xaxis.set_visible(False)
    ax[1].yaxis.set_visible(False)
    for spine in ['top', 'right', 'left', 'bottom']:
        ax[1].spines[spine].set_visible(False)

    # --- Display new figure for each png
    list_of_figures = []
    counter = 0
    for i in listop:

        # --- Display dashed vertical line moving along the x axis
        x = i / 1000
        line = ax[0].axvline(x=x, color='red', linestyle='dotted', linewidth=2)

        # --- Load picture from the rosbag file
        img = plt.imread(location + filename + '/pngs_fixed_cam/' + str(i) + '.png', 0)
        im = OffsetImage(img, zoom=0.55)
        ab = AnnotationBbox(im, (0, 0), xycoords='axes fraction', box_alignment=(0, 0))

        # --- Place image in subplot
        ax[1].add_artist(ab)
        plt.pause(0.00001)

        # --- Save image in order to create a video later
        fig_name = str(i) + '.png'
        temporal_dir = "/home/alejo/Documents/temporal/"
        plt.savefig(temporal_dir + fig_name)

        # --- Remove annotations to avoid RAM memory consumption
        ab.remove()
        line.remove()

        if counter == 0:
            time.sleep(1)      # Just allow some time before running the video

        counter += 1

    # --- Finally Save video with the new figures    
    out = None
    for i in listop:
        img = cv2.imread(temporal_dir + '/' + str(i) + '.png', cv2.IMREAD_ANYCOLOR)
        h, w, _ = img.shape

        if out is None:
            out = cv2.VideoWriter(temporal_dir + 'party.avi', cv2.VideoWriter_fourcc(*'MP4V'), 10, (w, h))
        out.write(img)

    out.release()
    plt.show()


if __name__ == '__main__':
    main()

# todo: same function but saving video