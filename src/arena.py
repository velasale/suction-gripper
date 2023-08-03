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


## --- Self developed imports

from ros_scripts import *


# folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/HIGH STIFFNESS/4th run - HIGH STIFFNESS - LOW FORCE/"
folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/LOW_STIFFNESS/"
file = "20230731_proxy_sample_5_yaw_-15_rep_0_stiff_low_force_low.bag"
topic = "/usb_cam/image_raw"
# topic = "/camera/image_raw"

# bag_to_pngs(folder, file, topic)
# bag_to_csvs(folder + file)
bag_to_video(folder, file, topic)