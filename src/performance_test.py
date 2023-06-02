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
import sys
import rosbag
import json
import datetime

## --- 3rd party related imports
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String, Int32
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

## --- Self developed imports
from bagfile_reader import *
from useful_ros import *


def main():

    # TODO place camera on Apple Proxy
    # TODO organize electronics of apple proxy

    # Initialize Class
    suction_gripper = RoboticGripper()

    # --- Step 1: Place robot at waypoint (preliminary position)
    suction_gripper.go_to_preliminary_position()

    # --- Step 2: Obtain info from user
    print("\n\n ***** Suction Gripper Experiments *****")
    print("a. Experiment Type: proxy, real")
    experiment = ''
    while (experiment == ''):
        experiment = input()
    suction_gripper.TYPE = experiment

    print("b. Pressure at the Valve [PSI]): ")
    print("Tip: If the desired pressure is lower than the current one (at the pressure regulator),\n then first pass that pressure and the go up to the desired pressure")
    suction_gripper.pressure_at_valve = int(input())

    # Info about gripper: pressure
    # Info about apple: stiffness -- type of springs, apples used
    # Pass these properties to the class

    # --- Step 3: Check that the vacuum circuit is free of holes
    # Turn on valve
    # Place flat object
    # Plot and check all sensors are @20KPa
    # Check camera, and all signals

    # --- Step 4: Pick the desired experiment
    if experiment == "proxy":
        proxy_picks(suction_gripper)

    elif experiment == "real":
        real_picks(suction_gripper)



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
        move_group.set_end_effector_link("eef")

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
        self.TYPE = "Proxy"

        # ---- Gripper Parameters
        # Source https://www.piab.com/inriverassociations/0206204/#specifications
        self.SUCTION_CUP_NAME = "F-BX20 Silicone"
        self.SUCTION_CUP_GIVE = 0.010
        self.SUCTION_CUP_RADIUS = 0.021 / 2

        # ---- Apple Proxy Parameters
        self.MAIN_SPRING_STIFFNESS = 540
        self.LATERAL_SPRING_STIFFNESS = 200

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
        print(goal_pose)

        goal_pose[0] = - 180 * pi / 180
        goal_pose[1] = -  80 * pi / 180
        goal_pose[2] = +  85 * pi / 180
        goal_pose[3] = +   0 * pi / 180
        goal_pose[4] = +  85 * pi / 180
        goal_pose[5] = +   0 * pi / 180

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

        goal_pose.pose.position.x = 0.5
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z = 0.5

        # Euler angles
        roll =      0 * pi / 180
        pitch =     0 * pi / 180
        yaw =       0 * pi / 180
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


    def place_marker_text(self, x=0,y=0,z=0, scale=0.01, text='caption'):
        """
        Places text as a marker in RVIZ
        @param x:
        @param y:
        @param z:
        @param scale:
        @param text:
        @return:
        """
        # Create a marker.  Markers of all shapes share a common type.
        caption = Marker()

        # Set the frame ID and type.  The frame ID is the frame in which the position of the marker
        # is specified.  The type is the shape of the marker, detailed on the wiki page.
        caption.header.frame_id = "world"
        caption.type = caption.TEXT_VIEW_FACING

        # Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
        # given time, then each needs to have a unique ID number.  If you publish a new marker with the same
        # ID number and an existing marker, it will replace the existing marker with that ID number.
        caption.id = 0

        # Set the action.  We can add, delete, or modify markers.
        caption.action = caption.ADD

        # These are the size parameters for the marker.  The effect of these on the marker will vary by shape,
        # but, basically, they specify how big the marker along each of the axes of the coordinate frame named
        # in frame_id.
        caption.scale.x = scale
        caption.scale.y = scale
        caption.scale.z = scale

        # Color, as an RGB triple, from 0 to 1.
        caption.color.r = 0
        caption.color.g = 0
        caption.color.b = 0
        caption.color.a = 1

        caption.text = text

        # Specify the pose of the marker.  Since spheres are rotationally invarient, we're only going to specify
        # the positional elements.  As usual, these are in the coordinate frame named in frame_id.  Every time the
        # marker is displayed in rviz, ROS will use tf to determine where the marker should appear in the scene.
        # in this case, the position will always be directly above the robot, and will move with it.
        caption.pose.position.x = x
        caption.pose.position.y = y
        caption.pose.position.z = z

        # Set up a publisher.  We're going to publish on a topic called balloon.
        self.marker_text_publisher.publish(caption)

        # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
        rate = rospy.Rate(10)


    def check_real_noise(self):
        """ Gets real noise"""

        ideal_pose = self.start_pose.pose

        # --- Step 1: Read current pose
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        current_pose_plframe = self.move_group.get_current_pose()
        current_pose_plframe.header.stamp = rospy.Time(0)

        # --- Step 2: Transform current pose into easy c-frame
        try:
            cur_pose_ezframe = tf_buffer.transform(current_pose_plframe, "EASYFRAME", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        self.noize_x_real = cur_pose_ezframe.pose.position.x - ideal_pose.position.x
        self.noize_y_real = cur_pose_ezframe.pose.position.y - ideal_pose.position.y
        self.noize_z_real = cur_pose_ezframe.pose.position.z - ideal_pose.position.z

        # TODO prints


    def save_metadata(self, filename):
        """
        Create json file and save it with the same name as the bag file
        @param filename:
        @return:
        """

        experiment_info = {
            "general": {
                "date": str(datetime.datetime.now()),
                "person": self.PERSON,
                "experiment type": self.experiment_type,
                "repetition": self.repetition
            },
            "robot": {
                "robot": self.ROBOT_NAME,
                "z noise command [m]": self.noise_z_command,
                "z noise real[m]": self.noise_z_real,
                "x noise command [m]": self.noise_x_command,
                "x noise real [m]": self.noise_x_real,
                # TODO"roll noise command [rad]":
            },
            "gripper": {
                "Suction Cup": self.SUCTION_CUP_NAME,
                "Pressure at Compressor": self.pressure_at_compressor,
                "Pressure at valve": self.pressure_at_valve
            },
            "target": {
                #TODO "Stiffnesses": self.STIFN
                #TODO "Apple Diameter": self.OBJECTRADIUS
            }
        }


    def publish_event(self, event):
        """
        Handy method to have the code events saved within the bagfile
        @param event:
        @return:
        """
        self.event_publisher.publisher(event)




if __name__ == '__main__':
    main()

