# @Time : 6/6/2023 11:15 AM
# @Author : Alejandro Velasquez

## --- Standard Library Imports
import os
import time
from matplotlib import pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
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
    values_list = experiment.pressure_sc1_values
    values_list_2 = experiment.pressure_sc2_values
    values_list_3 = experiment.pressure_sc3_values

    force_time_list = experiment.wrench_elapsed_time
    net_force_values_list = experiment.wrench_sumforce_relative_values
    # Sort png files in a list
    lst = os.listdir(location + filename + '/pngs_fixed_cam')
    listop = []
    for i in lst:
        if i.endswith('.png'):
            x = i.split('.png')[0]
            listop.append(int(x))
    listop.sort()

    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(12, 4.8))

    # --- Pressure Plots
    ax[0].plot(time_list, values_list, 'red', linewidth=2, label='cup A')
    ax[0].plot(time_list, values_list_2, 'green', linewidth=2, label='cup B')
    ax[0].plot(time_list, values_list_3, 'blue', linewidth=2, label='cup C')
    ax[0].set_xlim(0, max(time_list))
    ax[0].set_ylim(0, 120)
    ax[0].grid()
    ax[0].set_xlabel(xlabel)
    ax[0].set_ylabel(ylabel)
    ax[0].legend()

    # --- Force Plots
    ax_1 = ax[0].twinx()
    ax_1.plot(force_time_list, net_force_values_list, 'k-', linestyle='dashed', linewidth=2, label='net force')
    ax_1.set_ylim(0, 10)
    ax_1.set_ylabel("Force [N]")
    ax_1.legend()

    plt.ion()
    plt.show()
    # plt.title(filename, loc='right')


    # Remove details from ax[1] because we are displaying only the image
    ax[1].xaxis.set_visible(False)
    ax[1].yaxis.set_visible(False)
    for spine in ['top', 'right', 'left', 'bottom']:
        ax[1].spines[spine].set_visible(False)

    # out = None
    counter = 0
    for i in listop:
        # Vertical Line moving along the x axis
        x = i / 1000
        line = ax[0].axvline(x=x, color='red', linestyle='dotted', linewidth=2)

        # Picture from the rosbag file
        img = plt.imread(location + filename + '/pngs_fixed_cam/' + str(i) + '.png', 0)
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


# def main():



if __name__ == '__main__':
    main()

# todo: same function but saving video