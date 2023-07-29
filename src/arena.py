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


folder = "/home/alejo/Documents/data/SUCTION_GRIPPER_EXPERIMENTS/5th run - HIGH STIFFNESS - HIGH FORCE/"
file = "20230724_proxy_sample_0_yaw_0_rep_0_stiff_high_force_high.bag"
topic = "/usb_cam/image_raw"

# bag_to_pngs(folder, file, topic)
# bag_to_csvs(folder + file)
bag_to_video(folder, file, topic)