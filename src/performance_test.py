## --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import numpy as np
from numpy import pi, cos, sin, arccos, arange
import os
import keyboard
from random import random
import rospy
import time
import statistics as st
import subprocess, shlex, psutil
import sys
import rosbag
import json
import datetime

## --- Related 3rd party imports
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import String, Int32
# import sympy as sym
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

## --- Self developed imports
from bagfile_reader import *


def main():

    # TODO modify joint_limits.yaml for this test
    # TODO update the environment and scene
    # TODO place camera on Apple Proxy
    # TODO organize electronics of apple proxy

    # Initialize Class

    # --- Step 1: Place robot at waypoint (preliminary position)

    # --- Step 2: Obtain info from user
    # Info about gripper: pressure
    # Info about apple: stiffness -- type of springs, apples used
    # Pass these properties to the class

    # --- Step 3: Check that the vacuum circuit is free of holes
    # Turn on valve
    # Place flat object
    # Plot and check all sensors are @20KPa
    # Check camera, and all signals

    # --- Step 4: Pick the desired experiment
    # Exp 1: Real Apples
    # Exp 2: Apple Proxy

    ...


def proxy_picks():

    # --- Experiment Parameters ---
    # Number of experiments
    # Amount of noise

        # Measure Apple Position (simply add a fiducial marker)

        # Move to Starting Position

        # Start Recording Rosbag file

        # Add noise

        # Open Valve (apply vacuum)

        # Approach Surface

        # Label number of cups

        # Retreieve

        # Label result

        # Close Valve (stop vacuum)

        # Stop Recording Rosbag file

        # Save metadata in yaml file

        # Plot results, and decide to toss experiment away or not



    ...


def real_picks():
    ...



class RoboticGripper():

    def __init__(self):
        super(RoboticGripper, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robotic_gripper', anonymous=True)

        # ---- Initial Setup
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        #TODO update gripper geometry and mesh files
        move_group.set_end_effector_link("_todo_")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        event_publisher = rospy.Publisher('/experiment_steps', String, queue_size=20)
        marker_text_publisher = rospy.Publisher('captions', Marker, queue_size=1000, latch=True)

        planning_frame = move_group.get_planning_frame()
        print("=========== Planning frame: %s" % planning_frame)

        # ---- Variables and Parameters
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.event_publisher = event_publisher
        self.planning_frame = planning_frame

        # ---- Variables for markers
        self.marker_text_publisher = marker_text_publisher
        wiper = Marker()
        wiper.id = 0
        wiper.action = Marker.DELETEALL
        self.marker_text_publisher.publish(wiper)

        # ---- Experiment Parameters
        self.ROBOT_NAME = "UR5e"
        self.pressure_at_compressor = 100
        self.pressure_at_valve = 60
        self.PERSON = "Alejo"

        # Source https://www.piab.com/inriverassociations/0206204/#specifications
        self.SUCTION_CUP_NAME = "F-BX20 Silicone"
        self.SUCTION_CUP_GIVE = 0.010
        self.SUCTION_CUP_RADIUS = 0.021 / 2

        # ---- Noise variables
        self.noise_z_command = 0
        self.noise_z_real = 0
        self.noise_y_command = 0
        self.noise_y_real = 0
        self.noise_x_command = 0
        self.noise_x_real = 0
        self.noise_roll_command = 0
        self.noise_roll_real = 0
        self.noise_pitch_command = 0
        self.noise_pitch_real = 0
        self.noise_yaw_command = 0
        self.noise_yaw_real = 0

        # ---- Pose variables
        self.start_pose = tf2_geometry_msgs.PoseStamped()
        self.goal_pose = tf2_geometry_msgs.PoseStamped()
        self.previous_pose = tf2_geometry_msgs.PoseStamped()


    def go_to_preliminary_position(self):

        # --- Place marker with text in RVIZ
        caption = "Going to a preliminary pose"
        self.place_marker_text(x=0, y=0, z=1.5, scale=0.1, text=caption)

        # --- Initiate object joint
        goal_pose = self.move_group.get_current_joint_values()

        goal_pose[0] = +180 * pi /180


    def go_to_starting_position(self):

    def add_cartesian_noise(self):

    def place_marker_text(self):

    def save_metadata(self):

    def publish_event(self):

    def check_real_noise(self):











if __name__ == '__main__':
    main()

