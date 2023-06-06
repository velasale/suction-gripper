## --- Standard Library Imports
import os
import time
from matplotlib import pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox


def running_plot_and_video(location, filename, time_list, values_list, xlabel='Elapsed time [sec]', ylabel='Pressure [kPa]'):
    """
    Plots a graph with a vertical line running while playing a video given images
    @param location: Folder
    @param filename: Subfolder with the jpeg files
    @param time_list: data for plot's x-axis
    @param values_list: data for plot's y-axis
    @return:

    Ref:https://stackoverflow.com/questions/61808191/is-there-an-easy-way-to-animate-a-scrolling-vertical-line-in-matplotlib
    """

    # Sort png files in a list
    lst = os.listdir(location + filename + '/pngs')
    listop = []
    for i in lst:
        if i.endswith('.png'):
            x = i.split('.png')[0]
            listop.append(int(x))
    listop.sort()

    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(12, 4.8))
    ax[0].plot(time_list, values_list, 'k-', linewidth=2)
    ax[0].set_xlim(0, max(time_list))
    ax[0].set_ylim(0, 120)
    ax[0].grid()
    ax[0].set_xlabel(xlabel)
    ax[0].set_ylabel(ylabel)
    plt.ion()
    plt.show()
    plt.title(filename, loc='right')

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
        img = plt.imread(location + filename + '/pngs/' + str(i) + '.png', 0)
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



# todo: same function but saving video