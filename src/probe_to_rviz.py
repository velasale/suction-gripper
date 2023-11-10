#!/usr/bin/env python
"""
This code read the coordinates taken with a probe in the real apple tree, and shows them in RVIZ
velasale@oregonstate.edu
"""

# ---- Standard Library Imports
import os
import sys
import copy
import rospy
import time
import csv

# ---- 3rd party related imports
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import tf
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_about_axis
from visualization_msgs.msg import Marker, MarkerArray
import math
from math import pi
import numpy as np
from numpy import pi, cos, sin, arccos, arange
from random import random
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon

# ---- Self developed imporst
from performance_test import *


def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


def main():
    try:

        print("Replicating in RVIZ the original positions of the real apples picked")

        gripper = RoboticGripper()

        # ... Step 1: Read the txt file with all the coordinates
        location = '/media/alejo/Elements/Prosser_Data/Probe/'
        file = '20231101_apples_coords.csv'
        with open(location + file, 'r') as f:
            apples_coords = list(csv.reader(f, delimiter=","))
        apples_coords.pop(0)  # Remove header

        apples = len(apples_coords)
        print('\nThe number of apples were:', apples)

        # ... Place all the apples and stems
        for i in range(apples):

            apple_coords = apples_coords[i]

            # --- Save properties into experiment class
            gripper.APPLE_DIAMETER = float(apple_coords[1])
            gripper.APPLE_HEIGHT = float(apple_coords[2])
            gripper.APPLE_SOUTH_POLE_COORD = slist_into_flist(apple_coords[3])
            gripper.APPLE_NORTH_POLE_COORD = slist_into_flist(apple_coords[4])
            gripper.ABCISSION_LAYER_COORD = slist_into_flist(apple_coords[5])
            center = np.mean([gripper.APPLE_NORTH_POLE_COORD, gripper.APPLE_SOUTH_POLE_COORD], axis=0)

            # --- Calculate the angles of the apple
            delta_x = gripper.APPLE_NORTH_POLE_COORD[0] - gripper.APPLE_SOUTH_POLE_COORD[0]
            delta_y = gripper.APPLE_NORTH_POLE_COORD[1] - gripper.APPLE_SOUTH_POLE_COORD[1]
            delta_z = gripper.APPLE_NORTH_POLE_COORD[2] - gripper.APPLE_SOUTH_POLE_COORD[2]

            pitch = - math.atan(-delta_x / delta_y)
            roll = - math.atan(math.sqrt(delta_x ** 2 + delta_y ** 2) / delta_z)
            gripper.apple_pose = (center[0], center[1], center[2], roll, pitch, 0)

            # --- Visualize apple and core vector in RVIZ
            gripper.place_marker_sphere(color=[1, 0, 0, 0.5],
                                        pos=center,
                                        scale=gripper.APPLE_DIAMETER / 1000,
                                        frame='base_link')
            gripper.place_marker_arrow(gripper.APPLE_SOUTH_POLE_COORD,
                                       gripper.APPLE_NORTH_POLE_COORD,
                                       frame='base_link')
            label = apple_coords[0]
            gripper.place_marker_text(center, 0.05, label, frame='base_link')

            time.sleep(0.15)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
