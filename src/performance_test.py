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


def all_close(goal, current, tolerance):
    """
    Convenient method for testing if a lits of values are within a tolerance
    @param goal:
    @param current:
    @param tolerance:
    @return:
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(current[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        # TODO difference between Pose and PoseStamped
        return all_close(goal.pose, current.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(current), tolerance)

    return True



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
        self.TOLERANCE = 1/1000     # movement precision in [m]

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
        """Reaches a desired joint position (Forward Kinematics)"""

        # --- Place marker with text in RVIZ
        caption = "Going to a preliminary pose"
        self.place_marker_text(x=0, y=0, z=1.5, scale=0.1, text=caption)

        # --- Initiate object joint
        goal_pose = self.move_group.get_current_joint_values()

        goal_pose[0] = +180 * pi / 180
        goal_pose[1] = - 65 * pi / 180
        goal_pose[2] = -100 * pi / 180
        goal_pose[3] = - 90 * pi / 180
        goal_pose[4] = + 90 * pi / 180
        goal_pose[5] = +  0 * pi / 180

        # --- Move to the goal pose
        self.move_group.go(goal_pose, wait=True)
        self.move_group.stop()

        # --- Compare final pose with goal pose
        current_pose = self.move_group.get_current_joint_values()

        return all_close(goal_pose, current_pose, self.TOLERANCE)


    def go_to_starting_position(self):
        """Reaches a desired End Effector pose (Inverse Kinematics)"""

        # --- Place marker with text in RVIZ
        caption = "Going to an ideal starting End Effector pose (IK)"
        self.place_marker_text(x=0, y=0, z=1.5, scale=0.1, text=caption)

        # --- ROS tool for transformation among c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Set Goal in the intuitive/easy c-frame
        goal_pose = tf2_geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = "TODO"
        goal_pose.header.stamp = rospy.Time(0)

        goal_pose.pose.position.x =
        goal_pose.pose.position.y =
        goal_pose.pose.position.z =

        # Euler angles
        roll =
        pitch =
        yaw =
        q = quaternion_from_euler(roll, pitch, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.start_pose = goal_pose

        # --- Step 2: Transform Goal into Planning Frame
        try:
            goal_pose_pframe = tf_buffer.transform(goal_pose, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 3: Move robot
        self.move_group.set_pose_targets(goal_pose_pframe.pose)
        self.move_group.go(wait=True)
        self.move_group.stop()

        # --- Step 4: Compare goal with current state
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, current_pose, self.TOLERANCE)

        # --- Step 5: Save pose for future steps
        self.previous_pose = current_pose

        return success


    def add_cartesian_noise(self, x_noise, y_noise, z_noise):

        # --- Place marker with text in RVIZ
        caption = "Adding gaussian noise"
        self.place_marker_text(x=0, y=0, z=1.5, scale=0.05, text=caption)

        # --- ROS tool for transformation across c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Read the current pose in the planning frame
        cur_pose_pframe = tf2_geometry_msgs.PoseStamped()
        cur_pose_pframe.pose = self.move_group.get_current_pose().pose
        cur_pose_pframe.header.frame_id = self.planning_frame
        cur_pose_pframe.header.stamp = rospy.Time(0)

        # --- Step 2: Transform current pose into intuitive cframe and add noise
        try:
            cur_pose_ezframe = tf_buffer.transform(cur_pose_pframe, "TODO_ezframe", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        cur_pose_ezframe.pose.position.x += x_noise
        cur_pose_ezframe.pose.position.y += y_noise
        cur_pose_ezframe.pose.position.z += z_noise
        cur_pose_ezframe.header.stamp = rospy.Time(0)

        # --- Step 3: Transform goal pose back to planning frame
        try:
            goal_pose_pframe = tf_buffer.transform(cur_pose_ezframe, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Move to the new goal
        self.move_group.set_pose_targets(goal_pose_pframe.pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()    # good practice

        # --- Step 5: Compare poses
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe, current_pose, self.TOLERANCE)

        self.check_real_noise()

        return success





    def place_marker_text(self):

    def save_metadata(self):

    def publish_event(self):

    def check_real_noise(self):







if __name__ == '__main__':
    main()

