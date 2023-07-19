# @Time : 6/6/2023 11:15 AM
# @Author : Alejandro Velasquez

## --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import mpl_toolkits.mplot3d

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
from std_msgs.msg import String, Int32, UInt16
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

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

    # TODO place camera on Apple Proxy
    # TODO organize electronics of apple proxy

    # Initialize Class
    suction_gripper = RoboticGripper()

    # --- Step 1: Place robot at waypoint (preliminary position)
    suction_gripper.go_to_preliminary_position()

    # --- Step 2: Gather info from user
    print("\n\n****** Suction Gripper Experiments *****")

    print("a. Type your name:")
    name = ''
    while (name == ''):
        name = input().lower()      # convert to lower case
    suction_gripper.PERSON = name

    print("b. Experiment Type (proxy, real):")
    experiment = ''
    while (experiment != 'proxy' and experiment != 'real'):
        experiment = input()
    suction_gripper.TYPE = experiment

    print("c. Feed-In Pressure at valve [PSI] (60, 65, 70): ")
    print("Tip: If the desired pressure is lower than the current one (at the pressure regulator),\n then first pass that pressure and the go up to the desired pressure")
    pressure = 0
    while (pressure != 60 and pressure != 65 and pressure != 70):
        pressure = int(input())
    suction_gripper.pressure_at_valve = pressure

    # --- Proxy parameters
    print("d. Branch Stiffness level (low, medium, high):")
    stiffness = ''
    while (stiffness != 'low' and stiffness != 'medium' and stiffness != 'high'):
        stiffness = input()
    suction_gripper.SPRING_STIFFNESS_LEVEL = stiffness

    print("e. Magnet force level (low, medium, high):")
    magnet = ''
    while (magnet != 'low' and magnet != 'medium' and magnet != 'high'):
        magnet = input()
    suction_gripper.MAGNET_FORCE_LEVEL = magnet

    print("f. Apple Pick Pattern: (a) Retreat (b) Rotate and Retreat (c) Flex and Retreat:")
    pattern = ''
    while (pattern != 'a' and pattern != 'b' and pattern != 'c'):
        pattern = input()
    suction_gripper.PICK_PATTERN = pattern


    # --- Step 3: Check that the vacuum circuit is free of holes
    # TODO initial plot of pressures
    suction_gripper.suction_cup_test()

    # --- Step 4: Start the desired experiment
    if experiment == "proxy":
        proxy_picks(suction_gripper)

    elif experiment == "real":
        real_picks(suction_gripper)


def proxy_picks(gripper):

    # --- Experiment Parameters ---
    n_samples = 10  # starting positions to start gripper's pose
    yaws = [0, 60]
    n_reps = 1  # number of repetitions at each configuration

    cart_noises = [0, 5/1000, 10/1000, 15/1000, 20/1000]
    ang_noises = [0, 5, 10, 15, 20]

    # --- Provide location of Apple and Stem ---
    # Measure Apple Position (simply add a fiducial marker)
    # Place Apple in Rviz
    gripper.place_marker_sphere([1, 0, 0, 0.5], gripper.apple_pose[:3], gripper.APPLE_DIAMETER)
    # Place Sphere in Rviz
    gripper.place_marker_sphere([0, 1, 0, 0.5], gripper.apple_pose[:3], gripper.sphere_diam)

    # --- Sample points on Sphere
    gripper.point_sampling_2d(n_points=n_samples)
    apples_to_pick = len(gripper.x_coord)

    # --- Sample points on a sphere around the apple
    for sample in range(apples_to_pick):

        input("\n ** Press Enter to Start Sample Point %i **" % sample)
        gripper.sample = sample

        # --- First go to a way-point
        # gripper.go_to_preliminary_position()

        for yaw in yaws:

            input("\n **** Press Enter to Start with yam %i ****" % yaw)
            gripper.yaw = yaw

            for rep in range(n_reps):

                # --- Move to Ideal Starting Position
                input("\n ****** Press Enter to Continue with Rep %i *******" % rep)
                gripper.repetition = rep
                pose, move = gripper.go_to_starting_position_sphere(sample)
                gripper.apply_offset(0, 0, 0, yaw)
                gripper.gripper_pose = pose

                # --- Start Recording Rosbag file
                location = os.path.dirname(os.getcwd())
                folder = "/data/"
                name = datetime_simplified() + "_" + gripper.TYPE + \
                       "_sample_" + str(sample) + "_yaw_" + str(gripper.yaw) + "_rep_" + str(rep) + \
                       "_stiff_" + str(gripper.SPRING_STIFFNESS_LEVEL) + "_force_" + str(gripper.MAGNET_FORCE_LEVEL)
                filename = location + folder + name

                # topics = ["wrench", "joint_states", "experiment_steps", "/gripper/distance",
                #           "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3",
                #           "/usb_cam/image_raw"]

                topics = ["wrench", "joint_states",
                          "experiment_steps",
                          "/gripper/distance",
                          "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3"]

                command, rosbag_process = start_rosbag(filename, topics)
                print("\n... Start recording Rosbag")
                time.sleep(0.1)

                # --- Add Cartesian Noise
                x_noise = gripper.NOISE_RANGES[0] * np.random.uniform(-1, 1)
                y_noise = gripper.NOISE_RANGES[1] * np.random.uniform(-1, 1)
                z_noise = gripper.NOISE_RANGES[2] * np.random.uniform(0, 1)
                print(x_noise, y_noise, z_noise)
                input("\n\n----- Enter to add noise ----")

                # move = gripper.apply_offset(x_noise, y_noise, z_noise, 0)

                # --- Open Valve (apply vacuum)
                print("\n... Applying vacuum")
                gripper.publish_event("Vacuum On")
                service_call("openValve")

                # --- Approach Apple
                print("\n... Approaching apple")
                gripper.publish_event("Approach")
                move = gripper.move_normal(gripper.APPROACH, True)
                # todo: should we stop saving rosbag to avoid wasting space during labeling?

                # --- Label the cups that were engaged with apple
                print("\n... Label how did the suction cups engaged")
                gripper.publish_event("Labeling cups")
                # gripper.label_cups()

                # --- Retrieve
                print("\n... Picking Apple")
                gripper.publish_event("Retrieve")
                move = gripper.move_normal(gripper.RETRIEVE)

                # --- Label result
                print("\n... Label the pick result")
                gripper.publish_event("Labeling apple pick")
                # gripper.label_pick()
                # todo: should we stop saving rosbag to avoid wasting space during labeling?

                # --- Close Valve (stop vacuum)
                print("\n... Stop vacuum")
                gripper.publish_event("Vacuum Off")
                service_call("closeValve")
                time.sleep(0.05)

                # Stop Recording Rosbag file
                stop_rosbag(command, rosbag_process)
                print("\n... Stop recording Rosbag")
                time.sleep(1)

                # Save metadata in yaml file
                gripper.save_metadata(filename)
                print("\n... Saving metadata in *yaml file")

                # Plot results, and decide to toss experiment away or not


def real_picks(gripper):
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
        move_group.set_end_effector_link("eef")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        event_publisher = rospy.Publisher('/experiment_steps', String, queue_size=20)
        marker_text_publisher = rospy.Publisher('captions', Marker, queue_size=1000, latch=True)
        marker_balls_publisher = rospy.Publisher('balloons', MarkerArray, queue_size=100)

        # ---- Subscribers
        self.ps2_sub = rospy.Subscriber('/gripper/pressure/sc2', UInt16, self.read_pressure2, queue_size=1)
        self.ps2 = 1000.00
        self.ps3_sub = rospy.Subscriber('/gripper/pressure/sc3', UInt16, self.read_pressure3, queue_size=1)
        self.ps3 = 1000.00


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
        self.marker_id = 1
        self.proxy_markers = MarkerArray()
        self.marker_text_publisher = marker_text_publisher
        self.marker_balls_publisher = marker_balls_publisher

        wiper = Marker()
        wiper.id = 0
        wiper.action = Marker.DELETEALL
        self.marker_text_publisher.publish(wiper)
        self.proxy_markers.markers.append(wiper)
        self.marker_balls_publisher.publish(self.proxy_markers)

        # ---- Experiment Parameters
        self.ROBOT_NAME = "UR5e"
        self.pressure_at_compressor = 100
        self.pressure_at_valve = 60
        self.PERSON = "Alejo"
        self.TYPE = "Proxy"
        self.sample = 0
        self.repetition = 0

        # ---- Gripper Parameters
        # Source https://www.piab.com/inriverassociations/0206204/#specifications
        self.SUCTION_CUP_NAME = "F-BX20 Silicone"
        self.SUCTION_CUP_GIVE = 0.010
        self.SUCTION_CUP_RADIUS = 0.021 / 2

        # ---- Apple Proxy Parameters
        self.apple_pose = [-0.69, -0.275, +1.055, 0.00, 0.00, 0.00]
        self.stem_pose = [-0.49, -0.30, +1.28, 0.00, 0.00, 0.00]
        self.APPLE_DIAMETER = 80 / 1000  # units [m]
        self.APPLE_HEIGHT = 70 / 1000  # units [m]
        self.SPRING_STIFFNESS_LEVEL = 'high'
        self.MAGNET_FORCE_LEVEL = 'high'

        # ---- Noise variables
        # x,y,z and r,p,y
        # self.NOISE_RANGES = [15/1000, 15/1000, 15/1000, 15 * pi() / 180, 15 * pi() / 180, 15 * pi() / 180]
        self.NOISE_RANGES = [15 / 1000, 15 / 1000, 12 / 1000, 15, 15, 15]
        self.position_noise = [0.0, 0.0, 0.0]
        self.orientation_noise = [0.0, 0.0, 0.0]
        self.noise_reals = []       #todo

        # ---- Gripper Pose variables
        self.start_pose = tf2_geometry_msgs.PoseStamped()
        self.goal_pose = tf2_geometry_msgs.PoseStamped()
        self.previous_pose = tf2_geometry_msgs.PoseStamped()
        self.gripper_pose = tf2_geometry_msgs.PoseStamped().pose
        self.yaw = 0

        # --- Experiment Labels
        self.cupA_engaged = "no"
        self.cupB_engaged = "no"
        self.cupC_engaged = "no"
        self.pick_result =  ""

        # --- Variables for Spherical Sampling
        self.sphere_diam = self.APPLE_DIAMETER * 2.0
        self.x_coord = []
        self.y_coord = []
        self.z_coord = []
        self.pose_starts = []
        self.APPROACH = 2 * self.SUCTION_CUP_GIVE + (self.sphere_diam - self.APPLE_DIAMETER) / 2  # Distance to approach normal
        self.RETRIEVE = - 100 / 1000  # Distance to retrieve and pick apple
        self.PICK_PATTERN = 'a'


    def read_pressure2(self, msg):
        self.ps2 = msg.data

    def read_pressure3(self, msg):
        self.ps3 = msg.data

    def go_to_starting_position_sphere(self, index):
        """
        Places the end effector tangent to a sphere, and at the indexed point
        @param index: index of coordinates
        @return:
        """

        text = "Going to an IDEAL Starting Position # " + str(index)
        # Place a marker for the text
        self.place_marker_text([self.apple_pose[0], self.apple_pose[1], self.apple_pose[2] + 0.5], 0.1, text)

        current_pose = self.move_group.get_current_pose().pose

        # A. Sphere center location = apple's location
        # Center of Sphere = Location of apple
        h = self.apple_pose[0]
        k = self.apple_pose[1]
        l = self.apple_pose[2]

        x = self.x_coord[index] + h
        y = self.y_coord[index] + k
        z = self.z_coord[index] + l

        self.azimuth = math.atan2(self.y_coord[index], self.x_coord[index]) * 180 / pi

        # Step 1 - Get the vector pointing to the center of the sphere
        delta_x = x - h
        delta_y = y - k
        delta_z = z - l
        length = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
        # print("deltas", delta_x, delta_y, delta_z, length)

        # Step 2 - Make the cross product between that vector and the vector normal to the palm "z" to obtain the rotation vector
        V1 = [0, 0, 1]
        V2 = [-delta_x / length, -delta_y / length, -delta_z / length]  # Normalize it
        V3 = np.cross(V1, V2)

        self.vector = [delta_x, delta_y, delta_z]
        self.cross_vector = V3

        # Step 3 - Make the dot product to obtain the angle of rotation
        dot = np.dot(V1, V2)
        angle = math.acos(dot)

        # Step 4 - Obtain the quaternion from the single axis rotation
        q = quaternion_about_axis(angle, V3)

        pose_goal = self.move_group.get_current_pose().pose

        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        # Cartesian adjustment of the gripper's position, after PITCH rotation so the palm's normal vector points towards the apple
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        # Move the Arm
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()


        # Get the current pose to compare it
        current_pose = self.move_group.get_current_pose().pose

        # Save this pose in a global variable, to come back to it after picking the apple
        self.previous_pose = current_pose

        success = all_close(pose_goal, current_pose, 0.01)
        self.pose_starts.append(success)
        print("Pose Starts history", self.pose_starts)

        if success:
            self.place_marker_sphere(color=[0, 1, 0, 1], pos=[x, y, z], scale=0.01)
        else:
            self.place_marker_sphere(color=[1, 0, 0, 1], pos=[x, y, z], scale=0.01)

        return current_pose, success

    def go_to_preliminary_position(self):
        """Reaches a desired joint position (Forward Kinematics)"""

        # --- Place marker with text in RVIZ
        caption = "Going to a preliminary pose"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.1, text=caption)

        # --- Initiate object joint
        goal_pose = self.move_group.get_current_joint_values()
        print(goal_pose)
        #
        goal_pose[0] = + 32 * pi / 180
        goal_pose[1] = -  100 * pi / 180
        goal_pose[2] = -  85 * pi / 180
        goal_pose[3] = -  50 * pi / 180
        goal_pose[4] = +  70 * pi / 180
        goal_pose[5] = +  70 * pi / 180

        # goal_pose[0] = + 0.2
        # goal_pose[1] = - 1.4
        # goal_pose[2] = - 1.5
        # goal_pose[3] = - 1.4
        # goal_pose[4] = + 1.2
        # goal_pose[5] = + 1.2

        # --- Move to the goal pose
        self.move_group.go(goal_pose, wait=True)
        self.move_group.stop()

        # --- Compare final pose with goal pose
        current_pose = self.move_group.get_current_joint_values()

        return all_close(goal_pose, current_pose, self.TOLERANCE)

    def go_to_starting_position(self, yaw=0):
        """Reaches a desired End Effector pose (Inverse Kinematics)"""

        # --- Place marker with text in RVIZ
        caption = "Going to an ideal starting End Effector pose (IK)"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.1, text=caption, cframe=self.planning_frame)

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
        yaw = yaw * pi / 180
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

    def apply_offset(self, x_offset, y_offset, z_offset, Y_offset):

        # Save the noise commands
        self.position_noise = [x_offset, y_offset, z_offset]

        # --- Place marker with text in RVIZ
        caption = "Adding gaussian noise"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.05, text=caption)

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
            cur_pose_ezframe = tf_buffer.transform(cur_pose_pframe, 'eef', rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        cur_pose_ezframe.pose.position.x += x_offset
        cur_pose_ezframe.pose.position.y += y_offset
        cur_pose_ezframe.pose.position.z += z_offset

        q = [0, 0, 0, 0]
        q[0] = cur_pose_ezframe.pose.orientation.x
        q[1] = cur_pose_ezframe.pose.orientation.y
        q[2] = cur_pose_ezframe.pose.orientation.z
        q[3] = cur_pose_ezframe.pose.orientation.w

        e = euler_from_quaternion(q)
        new_yaw = e[2] - math.radians(Y_offset)

        q = quaternion_from_euler(e[0], e[1], new_yaw)
        cur_pose_ezframe.pose.orientation.x = q[0]
        cur_pose_ezframe.pose.orientation.y = q[1]
        cur_pose_ezframe.pose.orientation.z = q[2]
        cur_pose_ezframe.pose.orientation.w = q[3]

        cur_pose_ezframe.header.stamp = rospy.Time(0)

        # --- Step 3: Transform goal pose back to planning frame
        try:
            goal_pose_pframe = tf_buffer.transform(cur_pose_ezframe, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Move to the new goal
        self.move_group.set_pose_target(goal_pose_pframe.pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()    # good practice

        # --- Step 5: Compare poses
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, current_pose, self.TOLERANCE)

        self.check_real_noise()

        return success

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
            cur_pose_ezframe = tf_buffer.transform(current_pose_plframe, "eef", rospy.Duration(1))
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
        if self.PICK_PATTERN == 'a':
            pick_pattern = 'retreat'
        elif self.PICK_PATTERN == 'b':
            pick_pattern = 'rotate and retreat'
        elif self.PICK_PATTERN == 'c':
            pick_pattern = 'flex and retreat'

        # --- Organize metatada
        experiment_info = {
            "general": {
                "date": str(datetime.datetime.now()),
                "person": self.PERSON,
                "experiment type": self.TYPE,
                "sampling point": self.sample,
                "repetition": str(self.repetition),
                "pick pattern": str(pick_pattern),
            },
            "robot": {
                "robot": self.ROBOT_NAME,
                "suction cup": self.SUCTION_CUP_NAME,
                "pressure at compressor": self.pressure_at_compressor,
                "pressure at valve": self.pressure_at_valve,
                "gripper's ideal position": str(self.gripper_pose.position),
                "gripper's ideal orientation [quaternian]": str(self.gripper_pose.orientation),
                "position noise command [m]": self.position_noise,
                "orientation noise commmand ": self.orientation_noise,
            },
            "proxy": {
                "branch stiffness": self.SPRING_STIFFNESS_LEVEL,
                "stem force": self.MAGNET_FORCE_LEVEL,
                "apple diameter": self.APPLE_DIAMETER,
                "apple height": self.APPLE_HEIGHT,
                "apple pose": self.apple_pose,
                "stem pose": self.stem_pose,
            },
            "labels": {
                "suction cup a": self.cupA_engaged,
                "suction cup b": self.cupB_engaged,
                "suction cup c": self.cupC_engaged,
                "apple pick result": self.pick_result
            }
        }

        # --- Save metadata in file
        filename += ".json"
        with open(filename, "w") as outfile:
            json.dump(experiment_info, outfile, indent=4)

    def publish_event(self, event):
        """
        Handy method to have the code events saved within the bagfile
        @param event:
        @return:
        """
        self.event_publisher.publish(event)

    def label_cups(self):
        """Method to label if the suction cups engaged with the apple after the initial approach"""

        # https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
        CRED = '\033[91m'
        CGREEN = '\033[92m'
        CEND = '\033[0m'

        print("\n**** Label the engagement of the suction cups ****")

        print("Is Suction" + CGREEN + " cup-A " + CEND + "engaged? yes or no")
        state = ''
        while (state != 'yes' and state != 'no'):
            state = input()
        self.cupA_engaged = state

        print("Is Suction" + CGREEN + " cup-B " + CEND + "engaged? yes or no")
        state = ''
        while (state != 'yes' and state != 'no'):
            state = input()
        self.cupB_engaged = state

        print("Is Suction" + CGREEN + " cup-C " + CEND + "engaged? yes or no")
        state = ''
        while (state != 'yes' and state != 'no'):
            state = input()
        self.cupC_engaged = state

    def label_pick(self):
        """Method to label the result of the apple pick"""

        print("\n How was the apple pick?:")
        print("(a) Successful pick")
        print("(b) Un-successful: Apple picked but apple it fell afterwards")
        print("(c) Un-successful: Apple not picked")
        print("(d) Unsure and would like to repeat the trial")

        result = ''
        while (result != 'a' and result != 'b' and result != 'c' and result != 'd'):
            result = input()
        self.pick_result = result

    def move_normal(self, z, condition=False):
        """

        @param z:
        @param condition: This is meant to stop the movement if the suction cups engage
        @return:
        """

        # --- Place a marker with text in RVIZ
        text = "Moving in Z-Axis"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.05, text=text)

        # --- ROS tool for transformation across c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Read current pose (planning frame)
        cur_pose_pframe = tf2_geometry_msgs.PoseStamped()
        cur_pose_pframe.pose = self.move_group.get_current_pose().pose
        cur_pose_pframe.header.frame_id = self.planning_frame
        cur_pose_pframe.header.stamp = rospy.Time(0)

        # ---- Step 2: Transform current pose into the intutitive/easy frame and add noise
        try:
            cur_pose_ezframe = tf_buffer.transform(cur_pose_pframe, "eef", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        cur_pose_ezframe.pose.position.z += z
        cur_pose_ezframe.header.stamp = rospy.Time(0)

        # ---- Step 3: Transform again the goal pose into the planning frame
        try:
            goal_pose_pframe = tf_buffer.transform(cur_pose_ezframe, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Move to the new pose
        self.move_group.set_pose_target(goal_pose_pframe.pose)

        if condition == True:
            # Condition to stop if it senses the three sensors engaged
            plan = self.move_group.go(wait=False)
            close = False
            cnt = 0
            while close is False:
                rospy.sleep(.1)  # Sleep .1 second
                if cnt == 50 or (self.ps2 < 300 and self.ps3 < 300):
                    self.move_group.stop()
                    close = True
                print(cnt, self.ps2)
                cnt += 1
            print('finished moving')
        else:
            self.move_group.go(wait=True)
            self.move_group.stop()

        self.move_group.clear_pose_targets()

        # --- Step 5: Compare poses
        cur_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, cur_pose, self.TOLERANCE)

        return success

    def suction_cup_test(self):

        print("\n **** Testing Vacuum Levels ****")

        # --- Start Rosbag
        location = os.path.dirname(os.getcwd())
        foldername = "/data/"
        name = "vacuum_test"
        filename = location + foldername + name
        topics = ["experiment_steps", "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3"]
        command, rosbag_process = start_rosbag(filename, topics)

        print("Start recording Rosbag")
        time.sleep(1)

        # --- Apply Vacuum
        print("\n... Applying vacuum")
        self.publish_event("Vacuum On")
        # time.sleep(0.001)
        service_call("openValve")
        time.sleep(1)

        # --- Wait some time
        self.publish_event("Steady")
        time.sleep(5)

        # --- Stop Vacuum
        print("\n... Stop vacuum")
        self.publish_event("Vacuum Off")
        # time.sleep(0.01)
        service_call("closeValve")
        time.sleep(1)

        # --- Stop recording Rosbag
        stop_rosbag(command, rosbag_process)
        print("\n... Stop recording Rosbag")
        time.sleep(1)

        # --- Finally save the metadata
        self.save_metadata(filename)
        print("\n... Saving Metadata")

        # ----------------- Plot data to check vacuum levels --------------
        print("\n... Vacuum Preview")
        plot_vacuum(filename)

    def place_marker_text(self, pos=[0, 0, 0], scale=0.01, text='caption'):
        """
        Places text as marker in RVIZ
        @param pos: x,y,z location
        @param scale: scale of the text
        @param text: text to display
        @param cframe: cframe on which the coordinates are given
        @return:
        """
        # Create a marker.  Markers of all shapes share a common type.
        caption = Marker()

        # Set the frame ID and type.  The frame ID is the frame in which the position of the marker
        # is specified.  The type is the shape of the marker, detailed on the wiki page.
        caption.header.frame_id = 'world'
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
        caption.pose.position.x = pos[0]
        caption.pose.position.y = pos[1]
        caption.pose.position.z = pos[2]

        # Set up a publisher.  We're going to publish on a topic called balloon.
        self.marker_text_publisher.publish(caption)

        # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
        rate = rospy.Rate(10)

    def place_marker_sphere(self, color=[0, 0, 1, 1], pos=[0, 0, 0], scale=0.01):
        """
        Creates a Sphere as Marker, and appends it into an array of markers
        @param color: [r,g,b,a] where rgb are color i RGB format, and 'a' is visibility
        @param pos: [x,y,z] coordinates of the center of the marker
        @param scale:
        @param cframe: coordinate frame
        @return:
        """
        # Create a marker.  Markers of all shapes share a common type.
        sphere = Marker()

        # Set the frame ID and type.  The frame ID is the frame in which the position of the marker
        # is specified.  The type is the shape of the marker, detailed on the wiki page.
        sphere.header.frame_id = 'world'
        sphere.type = sphere.SPHERE

        # Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
        # given time, then each needs to have a unique ID number.  If you publish a new marker with the same
        # ID number and an existing marker, it will replace the existing marker with that ID number.
        sphere.id = self.marker_id + 1
        self.marker_id = sphere.id

        # Set the action.  We can add, delete, or modify markers.
        sphere.action = sphere.ADD

        # These are the size parameters for the marker.  The effect of these on the marker will vary by shape,
        # but, basically, they specify how big the marker along each of the axes of the coordinate frame named
        # in frame_id.
        sphere.scale.x = scale
        sphere.scale.y = scale
        sphere.scale.z = scale

        # Color, as an RGB triple, from 0 to 1.
        sphere.color.r = color[0]
        sphere.color.g = color[1]
        sphere.color.b = color[2]
        sphere.color.a = color[3]

        # Specify the pose of the marker.  Since spheres are rotationally invarient, we're only going to specify
        # the positional elements.  As usual, these are in the coordinate frame named in frame_id.  Every time the
        # marker is displayed in rviz, ROS will use tf to determine where the marker should appear in the scene.
        # in this case, the position will always be directly above the robot, and will move with it.
        sphere.pose.position.x = pos[0]
        sphere.pose.position.y = pos[1]
        sphere.pose.position.z = pos[2]
        sphere.pose.orientation.x = 0.0
        sphere.pose.orientation.y = 0.0
        sphere.pose.orientation.z = 0.0
        sphere.pose.orientation.w = 1.0

        self.proxy_markers.markers.append(sphere)
        # Set up a publisher.  We're going to publish on a topic called balloon.
        self.marker_balls_publisher.publish(self.proxy_markers)

        # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
        rate = rospy.Rate(10)

    def point_sampling_3d(self, n_points=1250):
        """
        This function samples points evenly distributed from the surface of a sphere
        Source: https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
        """
        # --- Step 1: Evenly distributed sampling
        indices = arange(0, n_points, dtype=float) + 0.5
        phi = arccos(1 - 2 * indices / n_points)
        theta = pi * (1 + 5 ** 0.5) * indices

        x, y, z = cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi)
        # Adjustment due to the distance between World and Base_Link frame

        # --- Step 2: Further selection of points for only one quarter of the sphere
        zoffset_from_center = + 0.8
        yoffset_from_center = - 0.2
        for i in range(len(x)):
            if z[i] < (zoffset_from_center):
                if y[i] < (yoffset_from_center):
                        self.x_coord.append(x[i] * self.sphere_diam/2)
                        self.y_coord.append(y[i] * self.sphere_diam/2)
                        self.z_coord.append(z[i] * self.sphere_diam/2)

        # --- Step 3: Apply Rotation matrix if needed
        stem_angle = -0
        rot_angle = math.radians(stem_angle)
        r_matrix = [[1,                 0,                      0                   ],
                    [0,                 +math.cos(rot_angle),   -math.sin(rot_angle)],
                    [0,                 +math.sin(rot_angle),   +math.cos(rot_angle)]]

        points = np.array([self.x_coord, self.y_coord, self.z_coord])
        new_points = np.dot(r_matrix, points)

        self.x_coord = new_points[0]
        self.y_coord = new_points[1]
        self.z_coord = new_points[2]

        x = new_points[0]
        y = new_points[1]
        z = new_points[2]
        # Sort the points in a way that is easier for the robot to go from one point to the other
        # self.x_coord.sort()

        # --- Step 4: Plot to see the points distributed along the surface
        pp.figure().add_subplot(111, projection='3d').scatter(x, y, z)
        pp.xlabel('x-axis')
        pp.ylabel('y-axis')

        pp.show()

    def point_sampling_2d(self, n_points=10):
        angular_range = 135
        angular_range = math.radians(angular_range)

        angle = 0
        angle_step = angular_range / (n_points-1)

        for i in range(n_points):
            angle = i * angle_step
            print(i, math.degrees(angle))
            y = - (self.sphere_diam / 2) * math.sin(angle)
            z = - (self.sphere_diam / 2) * math.cos(angle)
            self.x_coord.append(0)
            self.y_coord.append(y)
            self.z_coord.append(z)

        x = self.x_coord
        y = self.y_coord
        z = self.z_coord

        # --- Step 4: Plot to see the points distributed along the surface
        pp.figure().add_subplot(111, projection='3d').scatter(x, y, z)
        pp.xlabel('x-axis')
        pp.ylabel('y-axis')

        pp.show()





if __name__ == '__main__':
    main()

