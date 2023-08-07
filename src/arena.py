# --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import mpl_toolkits.mplot3d

import numpy as np
from numpy import pi, cos, sin, arccos, arange
import os
from random import random

import time
import statistics as st
import sys
import pyrosbag
import json
import datetime

## --- 3rd party related imports
## --- 3rd party related imports
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String, Int32, UInt16
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import cv2


## --- Self developed imports

from ros_scripts import *
## --- Self developed imports
from pressure_plot_scripts import *
from ros_scripts import *
from plot_scripts import *


def plot_vacuum(filename):
    """Simply plots vacuum using methods and functions from pressure_plot_scripts.py"""

    bag_to_csvs(filename + ".bag")
    metadata = read_json(filename + ".json")
    experim = read_csvs(metadata, filename)
    experim.elapsed_times()
    # experim.get_steady_vacuum('Steady', 'Vacuum Off')
    experim.plot_only_pressure()
    plt.show()


def main():

    folder = '/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/HIGH_STIFFNESS_GOOD_YAW/'
    file = '2023083_proxy_sample_9_yaw_-15_rep_1_stiff_high_force_medium'

    plot_vacuum(folder+file)



if __name__ == '__main__':
        main()
