# @Time : 6/6/2023 11:15 AM
# @Author : Alejandro Velasquez

## --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import mpl_toolkits.mplot3d

import numpy as np
import pandas as pd
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
from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import MoveGroupActionFeedback, MoveGroupGoal, MoveGroupActionResult

import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import cv2

## --- Self developed imports
from data_analysis_robot import *
from ros_scripts import *
from plot_scripts import *
from PressureTest import *


def slist_into_flist(string_list):
    """Converts a string list into a float list"""

    s = string_list.split('[')[1]
    ss = s.split(']')[0]
    sss = ss.split(',')

    return(np.float_(sss))


def draw_cross_hair():
    while True:
        # capturing Video from Webcam
        cap = cv2.VideoCapture(-1)

        # set Width and Height of output Screen
        frameWidth = 640
        frameHeight = 480
        success, img = cap.read()
        imgResult = img.copy()

        # Draw Vertical and Horizontal Line
        color = (0, 0, 0)
        x1 = int(frameWidth * 0.25)
        x2 = int(frameWidth * 0.50)
        x3 = int(frameWidth * 0.75)
        y1 = int(frameHeight * 0.25)
        y2 = int(frameHeight * 0.5)
        y3 = int(frameHeight * 0.75)
        cv2.line(imgResult, (x2, y1), (x2, y3), color, 2)
        cv2.line(imgResult, (x1, y2), (x3, y2), color, 2)

        # condition to break programs execution
        key = cv2.waitKey(1)
        if key == 27:
            break


def plot_vacuum(filename):
    """Simply plots vacuum using methods and functions from data_analysis_robot.py"""

    bag_to_csvs(filename + ".bag")
    metadata = read_json(filename + ".json")
    experim = read_csvs(metadata, filename)
    experim.elapsed_times()
    # experim.get_steady_vacuum('Steady', 'Vacuum Off')
    experim.plot_only_pressure()
    plt.show()


def main():

    # TODO organize electronics of apple proxy

    # Initialize Class
    suction_gripper = RoboticGripper()

    # --- Step 1: Place robot at waypoint (preliminary position)
    # suction_gripper.go_to_preliminary_position()

    # --- Step 2: Gather info from user
    print("\n\n****** Gripper Experiments *****")
    suction_gripper.info_from_user()

    # --- Step 3: Scan Apple and Stem
    # suction_gripper.scan_apple_and_stem()

    # --- Step 4: Check that the vacuum circuit is sealed
    # suction_gripper.suction_cup_test()

    # --- Step 5: Start the desired experiment
    if suction_gripper.TYPE == "proxy":
        proxy_picks(suction_gripper)
    elif suction_gripper.TYPE == "real":
        real_picks(suction_gripper)
    elif suction_gripper.TYPE == "sim":
        proxy_picks(suction_gripper)


def proxy_picks(gripper):

    # --- Experiment Parameters ---
    n_samples = 10  # starting positions to start gripper's pose
    yaws = [0, 60]
    # offsets = [0/1000, 5 / 1000, 10 / 1000, 15 / 1000, 20 / 1000, 25/1000]
    # offsets = [20 / 1000, 25 / 1000]

    # --- Uncomment if you need other poses
    # yaws = [60]
    # offsets = [5/1000, 10/1000]
    # offsets = [0 / 1000]
    # offsets = [25/1000]

    # --- Occlusion Trials Parameters ---
    n_samples = 10  # starting positions to start gripper's pose
    offsets = [0]
    yaws = [60]
    n_reps = 1

    cart_noises = [0, 5/1000, 10/1000, 15/1000, 20/1000]
    ang_noises = [0, 5, 10, 15, 20]

    # --- Adjust amount of retrieve depending on the type of stiffness
    if gripper.SPRING_STIFFNESS_LEVEL == 'low':
        gripper.RETRIEVE = - 140 / 1000  # Distance to retrieve and pick apple
    else:
        gripper.RETRIEVE = -100 / 1000

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
    for sample in range(5, apples_to_pick-1):

        gripper.sample = sample

        if sample > 5:
            gripper.RETRIEVE = - 140 / 1000  # Distance to retrieve and pick apple
        else:
            gripper.RETRIEVE = -100 / 1000

        # --- First go to a way-point
        # gripper.go_to_preliminary_position()

        for yaw in yaws:

            # Adjust yaw for the different pitches in order to make the fingers contact the clusters or not
            # Simply comment if unwanted
            if sample > 3:
                yaw = 0

            yaw += 25   # Adjustment to have Suction Cup A facing down first

            gripper.yaw = yaw

            for offset in offsets:

                for rep in range(0, n_reps):

                    # --- Move to Ideal Starting Position
                    input("\n *** Press Enter to Continue with Sample %i, Yaw %i, Offset %f, Rep %i ***" %(sample, yaw, offset, rep))
                    gripper.repetition = rep
                    # pose, move = gripper.go_to_starting_position_sphere(sample)
                    # gripper.gripper_pose = pose

                    # draw_cross_hair() #TODO

                    # --- Start Recording Rosbag file
                    location = os.path.dirname(os.getcwd())
                    folder = "/data/"
                    name = datetime_simplified() + "_" + gripper.TYPE + \
                           "_sample_" + str(sample) + "_yaw_" + str(gripper.yaw) + "_offset_" + str(offset) + "_rep_" + str(rep) + \
                           "_stiff_" + str(gripper.SPRING_STIFFNESS_LEVEL) + "_force_" + str(gripper.MAGNET_FORCE_LEVEL)
                    filename = location + folder + name

                    topics_with_cameras = ["wrench", "joint_states",
                                           "experiment_steps",
                                           "/gripper/distance",
                                           "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3",
                                           "/usb_cam/image_raw"
                                           # "/camera/image_raw"
                                           ]

                    topics_without_cameras = ["wrench", "joint_states",
                                              "experiment_steps",
                                              "/gripper/distance",
                                              "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3"]

                    command, rosbag_process = start_rosbag(filename, topics_with_cameras)
                    print("\n... Start recording Rosbag")
                    time.sleep(0.2)

                    # --- Add Cartesian Noise
                    # x_noise = gripper.NOISE_RANGES[0] * np.random.uniform(-1, 1)
                    # y_noise = gripper.NOISE_RANGES[1] * np.random.uniform(-1, 1)
                    # z_noise = gripper.NOISE_RANGES[2] * np.random.uniform(0, 1)
                    print('x_offset', offset)
                    input("\n\n----- Enter to add noise ----")
                    # move = gripper.apply_offset(x_noise, y_noise, z_noise, 0)

                    gripper.apply_offset(offset, 0, 0, yaw)

                    # --- Open Valve (apply vacuum)
                    if gripper.ACTUATION_MODE != 'fingers':
                        print("\n... Applying vacuum")
                        gripper.publish_event("Vacuum On")
                        service_call("openValve")               # See *.ino file for more details

                    # --- Approach Apple ---
                    print("\n... Approaching apple")
                    gripper.publish_event("Approach")
                    if gripper.ACTUATION_MODE != 'fingers':
                        # distance = gripper.APPROACH
                        distance = 0.03
                    else:
                        distance = 0.03
                    move = gripper.move_normal_until_suction(distance, speed_factor=0.1, condition=True)
                    # todo: should we stop saving rosbag to avoid wasting space during labeling?

                    # --- Label cup engagement
                    if gripper.ACTUATION_MODE != 'fingers':
                        print("\n... Label how did the suction cups engaged")
                        gripper.publish_event("Labeling cups")
                        gripper.label_cups()

                    # --- Close Fingers
                    if gripper.ACTUATION_MODE != 'suction':
                        input('Hit Enter to close the fingers')
                        gripper.publish_event("Closing fingers")
                        service_call("closeFingers")

                    # --- Retrieve
                    print("\n... Picking Apple")
                    gripper.publish_event("Retrieve")
                    move = gripper.move_normal_until_suction(gripper.RETRIEVE, speed_factor=0.1)

                    # --- Label result
                    print("\n... Label the pick result")
                    gripper.publish_event("Labeling apple pick")
                    gripper.label_pick()
                    # todo: should we stop saving rosbag to avoid wasting space during labeling?


                    # --- Close Valve
                    if gripper.ACTUATION_MODE != 'fingers':
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
        self.ps1_sub = rospy.Subscriber('/gripper/pressure/sc1', UInt16, self.read_pressure1, queue_size=1)
        self.ps1 = 1000.00
        self.ps2_sub = rospy.Subscriber('/gripper/pressure/sc2', UInt16, self.read_pressure2, queue_size=1)
        self.ps2 = 1000.00
        self.ps3_sub = rospy.Subscriber('/gripper/pressure/sc3', UInt16, self.read_pressure3, queue_size=1)
        self.ps3 = 1000.00
        self.tof_sub = rospy.Subscriber('/gripper/distance', UInt16, self.read_tof_distance, queue_size=1)
        self.tof_distance = 1000.00

        self.moveit_state_sub = rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.moveit_feedback_cb)
        self.moveit_state = []

        self.moveit_result_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult, self.moveit_result_cb, queue_size=10)
        self.moveit_result_var = []

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
        self.caption_id = 1
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
        self.TYPE = "proxy"
        self.sample = 0
        self.repetition = 0

        # ---- Gripper Parameters
        # Source https://www.piab.com/inriverassociations/0206204/#specifications
        self.SUCTION_CUP_NAME = "F-BX20 Silicone"
        self.SUCTION_CUP_GIVE = 0.010
        self.SUCTION_CUP_RADIUS = 0.021 / 2
        self.PRESSURE_THRESHOLD = 600
        self.SCUP_DISTANCE_TOCENTER = 0.032
        self.YAW_OFFSET = math.radians(15)      # In order for the x-axis to be aligned with suction cup B and C
        self.GRIPPER_TRACKS = "v8 70_80"
        self.ACTUATION_MODE = "dual"

        # ---- Apple Proxy Parameters
        # Medium Force z = 1.05
        # 1st Bushing y = -0.49    2nd Bushing y = -0.69
        self.apple_pose = [-0.315, -0.30, +1.125, 0.00, 0.00, 0.00]
        self.stem_pose = [-0.49, -0.30, +1.28, 0.00, 0.00, 0.00]
        self.APPLE_DIAMETER = 80 / 1000  # units [m]
        self.APPLE_HEIGHT = 70 / 1000  # units [m]
        self.SPRING_STIFFNESS_LEVEL = 'high'
        self.MAGNET_FORCE_LEVEL = 'high'
        self.APPLE_SOUTH_POLE_COORD = []
        self.APPLE_NORTH_POLE_COORD = []
        self.ABSCISSION_LAYER_COORD = []


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
        self.sphere_diam = self.APPLE_DIAMETER * 1.5
        self.x_coord = []
        self.y_coord = []
        self.z_coord = []
        self.pose_starts = []
        self.APPROACH = 4 * self.SUCTION_CUP_GIVE + 0.1*(self.sphere_diam - self.APPLE_DIAMETER)  # Distance to approach normal
        # self.APPROACH = 25
        # self.RETRIEVE = -100 / 1000
        self.RETRIEVE = -130 / 1000

        # --- Scanning Probe Parameters
        self.SCAN_PROBE_LENGTH = 0.1    # Length of scanning probe in [m]
        self.SCAN_PROBE_BASE_WIDTH = 1.3 * 0.0254 # Width of the base [m]. CAUTION: It includes the plate's width

        # --- Pick Pattern
        self.PICK_PATTERN = 'a'


    ### General Purpose methods ###
    def save_metadata(self, filename):
        """
        Create json file and save it with the same name as the bag file
        @param filename:
        @return:
        """
        if self.PICK_PATTERN == 'a':
            pick_pattern = 'straight retreat'
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
                "yaw": self.yaw,
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
                "tracks being used": self.GRIPPER_TRACKS,
                "actuation mode": self.ACTUATION_MODE,
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

    def check_real_noise(self):
        """ Reads eef cartesian noise"""

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


    ### Labeling methods ###
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
        print("(a) Successful pick: after pick pattern")
        print("(b) Un-successful: Apple picked but apple it fell afterwards")
        print("(c) Un-successful: Apple not picked")
        print("(d) Successful pick: before pick pattern ")
        print("(e) Successful pick: apple tweaked while closing fingers")

        result = ''
        while (result != 'a' and result != 'b' and result != 'c' and result != 'd' and result != 'e'):
            result = input()
        self.pick_result = result


    #### Pose Control methods ####
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

        # See pose in the EEF c-frame
        current_pose = self.move_group.get_current_pose()
        current_pose.header.frame_id = "world"
        current_pose.header.stamp = rospy.Time(0)
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        try:
            goal_pose_gripper = tf_buffer.transform(current_pose, 'eef', rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        q = [0, 0, 0, 0]
        q[0] = goal_pose_gripper.pose.orientation.x
        q[1] = goal_pose_gripper.pose.orientation.y
        q[2] = goal_pose_gripper.pose.orientation.z
        q[3] = goal_pose_gripper.pose.orientation.w

        e = euler_from_quaternion(q)
        print(e)


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

    def go_to_preliminary_position(self, prelim_pose_in_degrees):
        """Reaches a desired joint position (Forward Kinematics)"""

        # --- Place marker with text in RVIZ
        caption = "Going to a preliminary pose"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.1, text=caption)

        # --- Initiate object joint
        goal_pose = self.move_group.get_current_joint_values()
        # print(goal_pose)

        goal_pose = np.multiply(pi/180,prelim_pose_in_degrees)
        # print('Goal Pose:\n', goal_pose)

        # goal_pose[0] = - 180 * pi / 180
        # goal_pose[1] = -  60 * pi / 180
        # goal_pose[2] = + 120 * pi / 180
        # goal_pose[3] = + 240 * pi / 180
        # goal_pose[4] = +  90 * pi / 180
        # goal_pose[5] = + 150 * pi / 180

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

    def go_close_to_apple(self, yaw=0):
        """Reaches a desired End Effector pose (Inverse Kinematics)"""

        # --- Place marker with text in RVIZ
        caption = "Going to an ideal starting End Effector pose (IK)"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.1, text=caption, frame=self.planning_frame)

        # --- ROS tool for transformation among c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Set Goal in the intuitive/easy c-frame
        goal_pose = tf2_geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = rospy.Time(0)

        goal_pose.pose.position.x = self.apple_pose[0]
        goal_pose.pose.position.y = self.apple_pose[1]
        goal_pose.pose.position.z = self.apple_pose[2]

        # Euler angles
        roll = self.apple_pose[3]
        pitch = self.apple_pose[4]
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
        self.move_group.set_pose_target(goal_pose_pframe.pose)
        self.move_group.go(wait=True)
        self.move_group.stop()

        # --- Step 4: Compare goal with current state
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, current_pose, self.TOLERANCE)

        # --- Step 5: Save pose for future steps
        self.previous_pose = current_pose

        return success

    def apply_offset(self, x_offset=0, y_offset=0, z_offset=0, YAW_offset=0):

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
        new_yaw = e[2] - math.radians(YAW_offset)

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

    def move_normal_until_suction(self, z, speed_factor=1.0, cups=1, condition=False):
        """

        @param z:
        @param cups: Number of cups that need to engage before stopping
        @param condition: This is meant to stop the movement if suction cups engage
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

        # --- Step 4: Move to the new pose in a straight line
        waypoints = []
        waypoints.append(copy.deepcopy(goal_pose_pframe.pose))
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        self.move_group.set_max_acceleration_scaling_factor(speed_factor)
        self.move_group.set_max_velocity_scaling_factor(speed_factor)

        if condition:
            # --- Stop if sensors are engaged
            self.move_group.execute(plan, wait=False)
            close = False
            cnt = 0
            tof_flag = 0
            airp_thr = self.PRESSURE_THRESHOLD
            while not close:
                rospy.sleep(.1)  # Sleep .1 second
                thr_cnt = 0

                # --- Switch air-on
                if self.tof_distance < 100 and tof_flag == 0:
                    print("\n... Applying vacuum")
                    self.publish_event("Vacuum On")
                    service_call("openValve")
                    tof_flag = 1

                # --- Check if air-pressure signals are below the threshold
                for i in [self.ps1, self.ps2, self.ps3]:
                    if i < airp_thr:
                        thr_cnt += 1

                # --- Stop motion if one of these conditions is met
                if cnt == 50 or thr_cnt >= cups or tof_flag == 1:
                # https://docs.ros.org/en/jade/api/actionlib_msgs/html/msg/GoalStatus.html
                    self.move_group.stop()
                    close = True

                print(cnt, self.moveit_state)

                # print(cnt, self.ps1, self.ps2, self.ps3)
                cnt += 1
            print('finished moving')
        else:
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()

        self.move_group.clear_pose_targets()

        # --- Step 5: Compare poses
        cur_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, cur_pose, self.TOLERANCE)

        return success


    def simple_pitch_roll(self, axis, angle, cor, speed_factor, condition=False):
        """
        Adjusts the orientation of the end effector with respect to a given center of rotation.
        This method simply uses the axis-angle representation anc converts it into a quaternion.
        @param axis [degrees]: orientation of the axis of rotation
        @param angle [degrees]: magnitude of the angle of rotation
        @param cor [list of floats]: x,y,z location of center of rotation
        @param speed_factor:
        @param condition:
        @return:
        """

        # --- Place marker with text in RVIZ
        caption = "Adjusting PITCH"
        self.place_marker_text(pos=[0, 0, 1.5], scale=0.05, text=caption)

        # --- ROS tool for transformation across c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Read the current pose in the planning frame
        cur_pose_pframe = tf2_geometry_msgs.PoseStamped()
        cur_pose_pframe.pose = self.move_group.get_current_pose().pose
        cur_pose_pframe.header.frame_id = self.planning_frame
        cur_pose_pframe.header.stamp = rospy.Time(0)

        # --- Step 2: Transform current pose into intuitive cframe
        # A - Translate
        try:
            cur_pose_ezframe_step1 = tf_buffer.transform(cur_pose_pframe, 'eef', rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Transformation error: {e}")
            raise
        cur_pose_ezframe_step1.pose.position.x -= cor[0]
        cur_pose_ezframe_step1.pose.position.y -= cor[1]

        # B -  Rotate
        try:
            cur_pose_ezframe_step2 = tf_buffer.transform(cur_pose_ezframe_step1, 'eef', rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Transformation error: {e}")
            raise

        # Convert axis-angle representation to quaternion
        # https: // robotics.stackexchange.com / questions / 101253 / ur5 - how - to - convert - axis - angles - to - quaternions
        angle_rad = math.radians(angle / 2)
        s = math.sin(angle_rad)
        c = math.cos(angle_rad)
        q = [s * math.cos(math.radians(axis)), s * math.sin(math.radians(axis)), 0, c]

        cur_pose_ezframe_step2.pose.orientation.x = q[0]
        cur_pose_ezframe_step2.pose.orientation.y = q[1]
        cur_pose_ezframe_step2.pose.orientation.z = q[2]
        cur_pose_ezframe_step2.pose.orientation.w = q[3]

        cur_pose_ezframe_step2.header.stamp = rospy.Time(0)

        # C - Translate back to origin
        try:
            cur_pose_ezframe_step3 = tf_buffer.transform(cur_pose_ezframe_step2, 'eef', rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Transformation error: {e}")
            raise
        cur_pose_ezframe_step3.pose.position.x += cor[0]
        cur_pose_ezframe_step3.pose.position.y += cor[1]

        # Step 3: Transform goal pose back to planning frame
        try:
            goal_pose_pframe = tf_buffer.transform(cur_pose_ezframe_step3, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Transformation error: {e}")
            raise

        # Step 4: Move to the new goal
        pressure_threshold = self.PRESSURE_THRESHOLD
        self.move_group.set_pose_target(goal_pose_pframe.pose)

        self.move_group.set_max_acceleration_scaling_factor(speed_factor)
        self.move_group.set_max_velocity_scaling_factor(speed_factor)

        if condition:
            motion_flag = self.move_group.go(wait=False)
            close = False
            cnt = 0
            max_attempts = 50

            while not close:
                rospy.sleep(.1)
                thr_cnt = 0

                # --- Check if air-pressure signals are below the threshold
                for i in [self.ps1, self.ps2, self.ps3]:
                    if i < pressure_threshold:
                        thr_cnt += 1

                # --- Stop motion if one of these conditions is met
                # https://robotics.stackexchange.com/questions/77823/how-to-check-sate-of-plan-execution-in-moveit-during-async-execution-in-python
                if cnt == max_attempts or thr_cnt >= 3 or self.moveit_state == "IDLE":
                    self.move_group.stop()
                    close = True

                # print('Simple pitch_roll counter:',cnt)
                cnt += 1

        else:
            self.move_group.go(wait=True)
            self.move_group.stop()

        self.move_group.clear_pose_targets()    # Good practice: clear psoe targets

        # Step 5: Compare current pose with the goal pose
        current_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, current_pose, self.TOLERANCE)

        self.check_real_noise()

        return success


    #### Air-Pressure methods ####
    def moveit_feedback_cb(self, msg):
        self.moveit_state = msg.feedback.state
        # rospy.loginfo("Received /move_group/feedback: %s", msg.feedback.state)

    def moveit_result_cb(self, msg):
        self.moveit_result_var = msg.status.status
        # rospy.loginfo("Received /move_group/result: %s", msg.status.status)

    def read_pressure1(self, msg):
        self.ps1 = msg.data

    def read_pressure2(self, msg):
        self.ps2 = msg.data

    def read_pressure3(self, msg):
        self.ps3 = msg.data

    def read_tof_distance(self, msg):
        self.tof_distance = msg.data

    def suction_cup_test(self):

        print("\n **** Testing Vacuum Levels ****")

        # --- Start Rosbag
        location = "/media/alejo/Elements"
        foldername = "/Prosser_Data/"
        name = "vacuum_test_"
        filename = location + foldername + name
        topics = ["experiment_steps",
                  "/gripper/pressure/sc1",
                  "/gripper/pressure/sc2",
                  "/gripper/pressure/sc3"]
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


    def center_of_rotation(self, pA, pB, pC):

        x = 0
        y = 0
        # Case A
        if pA < self.PRESSURE_THRESHOLD:
            x = np.cos(np.pi / 3) * self.SCUP_DISTANCE_TOCENTER
            y = np.sin(np.pi / 3) * self.SCUP_DISTANCE_TOCENTER
        # Case B
        if pB < self.PRESSURE_THRESHOLD:
            x = -self.SCUP_DISTANCE_TOCENTER
            y = 0
        # Case C
        if pC < self.PRESSURE_THRESHOLD:
            x = np.cos(-np.pi / 3) * self.SCUP_DISTANCE_TOCENTER
            y = np.sin(-np.pi / 3) * self.SCUP_DISTANCE_TOCENTER
        # Case A & B
        if pA < self.PRESSURE_THRESHOLD and pB < self.PRESSURE_THRESHOLD:
            x = (1 / 2) * ((pA + pC - (2 * pB)) / (pA - (2 * pC) + pB)) * self.SCUP_DISTANCE_TOCENTER
            y = (np.sqrt(3) / 2) * ((pA - pC) / (pA - (2 * pC) + pB)) * self.SCUP_DISTANCE_TOCENTER
        # Case A & C
        if pA < self.PRESSURE_THRESHOLD and pC < self.PRESSURE_THRESHOLD:
            x = (1 / 2) * self.SCUP_DISTANCE_TOCENTER
            y = (np.sqrt(3) / 2) * ((pA - pC) / (pA + pC - (2 * pB))) * self.SCUP_DISTANCE_TOCENTER
        # Case B & C
        if pB < self.PRESSURE_THRESHOLD and pC < self.PRESSURE_THRESHOLD:
            x = (-1 / 2) * ((pA + pC - (2 * pB)) / ((2 * pA) - pC - pB)) * self.SCUP_DISTANCE_TOCENTER
            y = (-np.sqrt(3) / 2) * ((pA - pC) / ((2 * pA) - pC - pB)) * self.SCUP_DISTANCE_TOCENTER
        return x, y

    #### RVIZ marker methods ###
    def place_marker_text(self, pos=[0, 0, 0], scale=0.01, text='caption', frame='world'):
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
        caption.header.frame_id = frame
        caption.type = caption.TEXT_VIEW_FACING

        # Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
        # given time, then each needs to have a unique ID number.  If you publish a new marker with the same
        # ID number and an existing marker, it will replace the existing marker with that ID number.
        caption.id = self.caption_id + 1
        self.caption_id = caption.id

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

    def place_marker_sphere(self, color=[0, 0, 1, 1], pos=[0, 0, 0], scale=0.01, frame='world'):
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
        sphere.header.frame_id = frame
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

    def place_marker_arrow(self, p1, p2, frame='world'):

        stem = Marker()
        stem.header.frame_id = frame
        stem.type = stem.ARROW

        # Since we want to publish an array of markers, the id must be updated.
        stem.id = self.marker_id + 1
        self.marker_id = stem.id

        stem.action = stem.ADD

        # For line markers, only sclae.x works, and it defines the line width
        stem.scale.x = 0.01    # shaft diameter
        stem.scale.y = 0.02    # head diameter
        stem.scale.z = 0.02    # head length

        stem.color.r = 0
        stem.color.g = 1
        stem.color.b = 0
        stem.color.a = 1

        # Translate points into floats
        p1x = float(p1[0])
        p1y = float(p1[1])
        p1z = float(p1[2])
        p2x = float(p2[0])
        p2y = float(p2[1])
        p2z = float(p2[2])

        p1 = Point()
        p1.x = p1x
        p1.y = p1y
        p1.z = p1z

        p2 = Point()
        p2.x = p2x
        p2.y = p2y
        p2.z = p2z

        stem.points.append(p1)
        stem.points.append(p2)

        self.proxy_markers.markers.append(stem)
        self.marker_balls_publisher.publish(self.proxy_markers)

        rate = rospy.Rate(10)

    ### Point sampling methods ###
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

        # pp.show()

    def info_from_user(self):
        print("a. Type your name:")
        name = ''
        while (name == ''):
            name = input().lower()  # convert to lower case
        self.PERSON = name

        print("b. Experiment Type (sim, proxy, real):")
        experiment = ''
        while (experiment != 'sim' and experiment != 'proxy' and experiment != 'real'):
            experiment = input()
        self.TYPE = experiment

        print("c. Feed-In Pressure at valve [PSI] (60, 65, 70): ")
        print(
            "Tip: If the desired pressure is lower than the current one (at the pressure regulator),\n then first pass that pressure and the go up to the desired pressure")
        pressure = 0
        while (pressure != 60 and pressure != 65 and pressure != 70):
            pressure = int(input())
        self.pressure_at_valve = pressure

        if self.TYPE == 'proxy':
            # --- Proxy parameters
            print("d. Branch Stiffness level (low, medium, high):")
            stiffness = ''
            while (stiffness != 'low' and stiffness != 'medium' and stiffness != 'high'):
                stiffness = input()
            self.SPRING_STIFFNESS_LEVEL = stiffness

            print("e. Magnet force level (low, medium, high):")
            magnet = ''
            while (magnet != 'low' and magnet != 'medium' and magnet != 'high'):
                magnet = input()
            self.MAGNET_FORCE_LEVEL = magnet

        print("f. Apple Pick Pattern: (a) Retreat (b) Rotate and Retreat (c) Flex and Retreat:")
        pattern = ''
        while (pattern != 'a' and pattern != 'b' and pattern != 'c'):
            pattern = input()
        self.PICK_PATTERN = pattern

        print("What gripper tracks are you using 'v8 70_80' or 'v8 80_90'?")
        tracks = ''
        while(tracks != 'v8 70_80' and tracks != 'v8 80_90'):
            tracks = input()
        self.GRIPPER_TRACKS = tracks

        print("What actuation mode are you doing on these experiments ('dual, 'suction' or 'fingers')")
        act_mode = ''
        while(act_mode != 'dual' and act_mode != 'suction' and act_mode != 'fingers'):
            act_mode = input()
        self.ACTUATION_MODE = act_mode

    ### Probe methods ###

    def scan_point(self):
        """Transforms the position of the probe's tip into base_link frame"""

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        probe_tool = tf2_geometry_msgs.PoseStamped()

        # Probe's (x,y,z) coordinate at 'tool0' frame
        probe_tool.pose.position.x = 0
        probe_tool.pose.position.y = 0
        probe_tool.pose.position.z = self.SCAN_PROBE_LENGTH + self.SCAN_PROBE_BASE_WIDTH - (0.162 + 0.02) # CAUTION: See urdf.xacro
        probe_tool.header.frame_id = "eef"
        probe_tool.header.stamp = rospy.Time(0)

        # Transform the position of the tip of the probe to the desired frame
        try:
            probe_base = tf_buffer.transform(probe_tool, "base_link", rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # Just pass the x,y,z, coordinates
        point_coordinates = [float(probe_base.pose.position.x),
                             float(probe_base.pose.position.y),
                             float(probe_base.pose.position.z)]
        print("Probe coordinates at 'base_link' frame:", point_coordinates)

        return point_coordinates

    def scan_apple_and_stem(self):

        print("--- Scanning Apple ---")

        input("--- Place probe at apple CALIX (south pole), hit ENTER when ready.")
        self.APPLE_SOUTH_POLE_COORD = self.scan_point()

        input("--- Place probe at apple's north pole, hit ENTER when ready.")
        self.APPLE_NORTH_POLE_COORD = self.scan_point()

        input("--- Place probe at stem's abcission layer, hit ENTER when ready.")
        self.ABSCISSION_LAYER_COORD = self.scan_point()

        print("The point are: ")
        print(self.APPLE_SOUTH_POLE_COORD)
        print(self.APPLE_NORTH_POLE_COORD)
        print(self.ABSCISSION_LAYER_COORD)

        input("Hit Enter to check the vacuum line")


def scan_apples():
    """Simple method to (i) scan a bunch of points in space, an (ii) saves them as .csv for further use."""

    suction_gripper = RoboticGripper()

    fields = ['Label', 'Apple Diameter [mm]', 'Apple Height [mm]', 'South Pole coords', 'North Pole coords', 'Abcision coords']
    number_of_apples = 20
    apples_coords = []

    for apple in range(number_of_apples):

        print("\n\nPick No.", apple + 1)

        # Inputs from keyword
        label = input("--- Label item to be scanned (e.g. apple #1, imu #2), or hit ESC to finish : ")

        if label == '\x1b':
            break

        apple_diameter = input("--- Measure the Apple Main Diameter in [mm]: ")
        apple_height = input("--- Measure the Apple Height in [mm]: ")

        # Inputs from scanning probe
        input("--- Place probe at apple CALIX (south pole), hit ENTER when ready.")
        APPLE_SOUTH_POLE_COORD = suction_gripper.scan_point()

        input("--- Place probe at apple's north pole, hit ENTER when ready.")
        APPLE_NORTH_POLE_COORD = suction_gripper.scan_point()

        input("--- Place probe at stem's abcission layer, hit ENTER when ready.")
        ABCISSION_LAYER_COORD = suction_gripper.scan_point()

        apple_coords = [label, apple_diameter, apple_height,
                        APPLE_SOUTH_POLE_COORD, APPLE_NORTH_POLE_COORD, ABCISSION_LAYER_COORD]

        apples_coords.append(apple_coords)

    # Save into csv

    with open('../data/apples_coords.csv', 'w') as f:
        # using csv.writer method from CSV package
        write = csv.writer(f)
        write.writerow(fields)
        write.writerows(apples_coords)


def real_picks(gripper=RoboticGripper()):

    # TODO: Fix pressure plots
    # TODO: Fix limits
    # TODO: During each apple pick, simply allow user to choose the apple label

    # TODO: For each apple:
    #       b) Define the plane wich Define the orientation

    # --- Load list of apples' coordinates
    with open("../data/20231101_2nd_apples_coords.csv", "r") as f:
        apples_coords = list(csv.reader(f, delimiter=","))
    apples_coords.pop(0)    # Remove header

    # --- Add a column to keep track of the attempts at each apple
    for apple in apples_coords:
        apple.append(0)

    while True:

        # --- First ask what apple to approach
        apple_label = input('What is the apple number?')

        # --- Find the apple coordinates
        index = 0
        for apple in apples_coords:
            if apple[0] == ('apple' + apple_label):
                break
            index += 1
        apple_coords = apples_coords[index]
        apples_coords[index][-1] += 1
        attempt = apples_coords[index][-1]

        print('\nApple%s - Picking attempt No.%i' % (apple_label, attempt))

        # --- Save properties into experiment class
        gripper.APPLE_DIAMETER = float(apple_coords[1])
        gripper.APPLE_HEIGHT = float(apple_coords[2])
        gripper.APPLE_SOUTH_POLE_COORD = slist_into_flist(apple_coords[3])
        gripper.APPLE_NORTH_POLE_COORD = slist_into_flist(apple_coords[4])
        gripper.ABSCISSION_LAYER_COORD = slist_into_flist(apple_coords[5])
        center = np.mean([gripper.APPLE_NORTH_POLE_COORD, gripper.APPLE_SOUTH_POLE_COORD], axis=0)

        # --- Calculate the angles of the apple
        delta_x = gripper.APPLE_NORTH_POLE_COORD[0] - gripper.APPLE_SOUTH_POLE_COORD[0]
        delta_y = gripper.APPLE_NORTH_POLE_COORD[1] - gripper.APPLE_SOUTH_POLE_COORD[1]
        delta_z = gripper.APPLE_NORTH_POLE_COORD[2] - gripper.APPLE_SOUTH_POLE_COORD[2]

        pitch = - math.atan(-delta_x/delta_y)
        roll = - math.atan(math.sqrt(delta_x**2+delta_y**2)/delta_z)
        gripper.apple_pose = (center[0], center[1], center[2], roll, pitch, 0)

        # --- Visualize apple and core vector in RVIZ
        gripper.place_marker_sphere(color=[1, 0, 0, 0.5],
                                    pos=center,
                                    scale=gripper.APPLE_DIAMETER/1000,
                                    frame='base_link')
        gripper.place_marker_arrow(gripper.APPLE_SOUTH_POLE_COORD,
                                   gripper.APPLE_NORTH_POLE_COORD,
                                   frame='base_link')
        label = apple_coords[0]
        gripper.place_marker_text(center, 0.05, label, frame='base_link')

        # --- Manually place gripper close to the apple
        input('Please move the EEF close to the desired apple, and hit ENTER')

        orientations = [0, 15, 30, 45, 60, 75, 90]
        yaws = [0, 60]

        # TODO: Measure offset between gripper and apple center
        # TODO: approaches

        orientations = [0]
        yaws = [0]

        for orientation in orientations:

            gripper.sample = orientation

            for yaw in yaws:

                gripper.yaw = yaw

                # --- Start Recording bagfile
                location = '/media/alejo/Elements'
                # location = os.path.dirname(os.getcwd())
                folder = "/Prosser_Data/"

                name = datetime_simplified() + "_" + gripper.TYPE + \
                       str(label) + \
                       "_mode_" + gripper.ACTUATION_MODE +\
                       "_attempt_" + str(attempt) + \
                       "_orientation_" + str(orientation) + \
                       "_yaw_" + str(gripper.yaw)

                filename = location + folder + name

                topics_with_cameras = ["wrench", "joint_states",
                                       "experiment_steps",
                                       "/gripper/distance",
                                       "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3",
                                       "/usb_cam/image_raw",
                                       "/camera/image_raw"]

                topics_without_cameras = ["wrench", "joint_states",
                                          "experiment_steps",
                                          "/gripper/distance",
                                          "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3"]

                topics_just_camera = ["/usb_cam/image_raw"]

                command, rosbag_process = start_rosbag(filename, topics_with_cameras)
                print("\n... Start recording Rosbag")
                time.sleep(1)

                # --- Go to the desired pose
                # TODO align gripper with apple vector pose
                # TODO approach to a distance of 80mm from center
                # gripper.go_close_to_apple()

                # --- Adopt desired yaw
                # gripper.apply_offset(0, 0, 0, yaw)

                # --- Open Valve
                if gripper.ACTUATION_MODE != 'fingers':
                    print("\n... Applying vacuum")
                    gripper.publish_event("Vacuum On")
                    service_call("openValve")

                # --- Approach Apple
                print("\n... Approaching apple")
                gripper.publish_event("Approach")
                move = gripper.move_normal_until_suction(gripper.APPROACH, speed_factor=0.1, condition=True)

                # --- Label cup engagement
                if gripper.ACTUATION_MODE != 'fingers':
                    print("\n... Label how did the suction cups engaged")
                    gripper.publish_event("Labeling cups")
                    gripper.label_cups()

                # --- Close Fingers
                if gripper.ACTUATION_MODE != 'suction':
                    input('Hit Enter to close the fingers')
                    gripper.publish_event("Closing fingers")
                    service_call("closeFingers")

                # # --- Retrieve
                time.sleep(0.05)
                input('Hit Enter to start picking the apple')
                print("\n... Picking Apple")
                gripper.publish_event("Retrieve")
                move = gripper.move_normal_until_suction(gripper.RETRIEVE, speed_factor=0.1)

                # --- Label Result
                print("\n... Label the pick result")
                gripper.publish_event("Labeling apple pick")
                gripper.label_pick()

                # --- Open Fingers
                if gripper.ACTUATION_MODE != 'suction':
                    input('Hit Enter to open the fingers')
                    gripper.publish_event("Opening fingers")
                    service_call("openFingers")
                    time.sleep(0.05)

                # --- Close Valve
                if gripper.ACTUATION_MODE != 'fingers':
                    print("\n... Stop vacuum")
                    gripper.publish_event("Vacuum Off")
                    service_call("closeValve")
                    time.sleep(0.05)

                # --- Stop recording Rosbag file
                stop_rosbag(command, rosbag_process)
                print("\n... Stop recording Rosbag")
                time.sleep(1)

                # Save metadata in yaml file
                gripper.save_metadata(filename)
                print("\n... Saving metadata in *yaml file")


def pressure_servoing():

    # --- Instantiate gripper
    gripper = RoboticGripper()

    # --- Parameters
    SERVO_ADJUST_DISTANCE = 0.01
    SERVO_ADJUST_SPEEDFACTOR = 0.35
    # SERVO_ADJUST_SPEEDFACTOR = 0.80

    APPROACH_DISTANCE = 0.20
    APPROACH_SPEED_FACTOR = 0.0005

    FRUITPICK_SPEED_FACTOR = 0.02

    TIME_SLEEP_FOR_ROSSERIAL = 0.020
    MAX_ATTEMPTS = 5
    KP = 0.015


    # --- Inputs from user
    # sequence 1: vac on / sense / rotate
    # sequence 2: vac on / sense / vac off / rotate
    # sequence 3: vac on / sense / vac off / back / rotate / approach
    print("Choose experiment sequence (1, 2 or 3):")
    sequence = ''
    while sequence != '1' and sequence != '2' and sequence != '3':
        sequence = input()

    print("Choose branch stiffness (low, medium or high):")
    stiffness = ''
    while stiffness != 'high' and stiffness != 'low' and stiffness != 'medium':
        stiffness = input()
        gripper.SPRING_STIFFNESS_LEVEL = stiffness

    # Location of apple with LOW stiffness stem
    if stiffness == 'low':
        gripper.apple_pose = [-0.285, -0.30, 1.135, 0.00, 0.00, 0.00]
        gripper.go_to_preliminary_position([-22.9, -54.64, 115.79, 261.28, 71.74, 56.84])

    # Location of apple with HIGH stiffness stem
    if stiffness == 'medium':
        gripper.apple_pose = [-0.36, -0.30, 1.135, 0.00, 0.00, 0.00]
        # gripper.go_to_preliminary_position([-11.92, -40.72, 107.92, 243.87, 85.91, 49.25])
        gripper.go_to_preliminary_position([-164, -168, -87, -56.26, -69, 28])

    # Location of apple with HIGH stiffness stem
    if stiffness == 'high':
        gripper.apple_pose = [-0.43, -0.30, 1.135, 0.00, 0.00, 0.00]
        # gripper.go_to_preliminary_position([-7.13, -54.61, 113.66, 266.56, 83.48, 50])
        gripper.go_to_preliminary_position([-159.18, -168.81, -83.02, -61.85, -68.54, 28])

    # Place Apple and Sphere in Rviz
    # TODO: Why is not placing balloons until it moves?
    gripper.place_marker_sphere([1, 0, 0, 0.5], gripper.apple_pose[:3], gripper.APPLE_DIAMETER)
    gripper.place_marker_sphere([0, 1, 0, 0.5], gripper.apple_pose[:3], gripper.sphere_diam)

    print("Choose initial offset (1cm or 2cm or 3cm):")
    offset = ''
    while offset != '1' and offset != '2' and offset != '3':
        offset = input()
    gripper.apply_offset(float(int(offset)/100),0, -0.15,0)

    input("\n*** Press Enter to begin sequence***")

    # --- Start recording bagfile
    location = '/home/alejo/gripper_ws/src/suction-gripper/data/'
    folder = ''
    name = (datetime_simplified()
            + "__stf_" + str(stiffness)
            + "__offset_" + str(offset)
            + "__seq_" + str(sequence)
            + "__rep_" + str('todo')
            )
    filename = location + folder + name

    topics_without_cameras = ["wrench", "joint_states",
                              "experiment_steps",
                              "/gripper/distance",
                              "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3"]

    topics_with_cameras = ["wrench", "joint_states",
                           "experiment_steps",
                           "/gripper/distance",
                           "/gripper/pressure/sc1", "/gripper/pressure/sc2", "/gripper/pressure/sc3",
                           "/usb_cam/image_raw"
                           ]

    command, rosbag_process = start_rosbag(filename, topics_with_cameras)
    print("\n... Start recording Rosbag")
    time.sleep(1)

    # --- Step 1: Initial approach to fruit in a straight line
    print("\n... Approaching apple")
    move = gripper.move_normal_until_suction(APPROACH_DISTANCE, APPROACH_SPEED_FACTOR, cups=1, condition=True)

    # --- Step 2: Control loop
    cnt = 0
    while cnt < MAX_ATTEMPTS:

        # --- A - Sense: Pressure Readings
        ps1_list = []
        ps2_list = []
        ps3_list = []
        readings = 10
        for i in range(readings):
            ps1_list.append(gripper.ps1)
            ps2_list.append(gripper.ps2)
            ps3_list.append(gripper.ps3)
        ps1_mean = np.mean(ps1_list)
        ps2_mean = np.mean(ps2_list)
        ps3_mean = np.mean(ps3_list)
        print("\nMean pressure readings: ", int(ps1_mean), int(ps2_mean), int(ps3_mean))

        threshold = gripper.PRESSURE_THRESHOLD
        if ps1_mean < threshold and ps2_mean < threshold and ps3_mean < threshold:
            print("All suction cups engaged!!!")
            break

        # --- B - Switch air off (depending on sequence)
        if sequence == '2' or sequence == '3':
            print("\n... Closing vacuum")
            gripper.publish_event("Vacuum Off")
            service_call("closeValve")
            time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

        # --- C - Move back (depending on sequence)
        if sequence == '3':
            print("\n... Moving back a bit")
            move = gripper.move_normal_until_suction(-0.9*SERVO_ADJUST_DISTANCE, SERVO_ADJUST_SPEEDFACTOR)

        # --- D - Adjust pose
        # Net Air Pressure magnitude and orientation
        print("\n... Adjusting pose")
        magnitude, net_angle = olivia_test(ps1_mean, ps2_mean, ps3_mean)
        print("P_SUM magnitude %.2f, P_SUM angle %.2f" % (magnitude, math.degrees(net_angle)))
        magnitude = magnitude * KP
        net_angle = math.degrees(net_angle)
        axis_of_rotation = net_angle - 90
        print('Axis of rotation %.0f' % axis_of_rotation)
        # Find Center of rotation
        x, y = gripper.center_of_rotation(ps1_mean, ps2_mean, ps3_mean)
        # Adjust pose
        gripper.simple_pitch_roll(axis_of_rotation, magnitude, [x, y], SERVO_ADJUST_SPEEDFACTOR, condition=True)

        # --- E - Switch air back on (depending on sequence)
        if sequence == '2' or sequence == '3':
            print("\n... Applying vacuum")
            gripper.publish_event("Vacuum On")
            service_call("openValve")
            time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

        # --- F - Approach again (depending on sequence)
        if sequence == '3':
            print("\n... Approaching apple")
            move = gripper.move_normal_until_suction(1.1*SERVO_ADJUST_DISTANCE, SERVO_ADJUST_SPEEDFACTOR, cups=2, condition=True)

        cnt += 1
        print(cnt)

    time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

    # --- Step 3: Deploy Fingers
    print("\n... Deploying fingers")
    gripper.publish_event("Closing fingers")
    service_call("closeFingers")

    time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

    # --- Step 4: Pick Apple
    print("\n... Picking Apple")
    gripper.publish_event("Retrieve")
    move = gripper.move_normal_until_suction(gripper.RETRIEVE, FRUITPICK_SPEED_FACTOR)

    time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

    # --- Step 5: Open fingers
    print("\n... Opening fingers")
    gripper.publish_event("Opening fingers")
    service_call("openFingers")

    time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

    # --- Step 6: Close Air
    print("\n... Closing vacuum")
    gripper.publish_event("Vacuum Off")
    service_call("closeValve")

    time.sleep(TIME_SLEEP_FOR_ROSSERIAL)

    # --- Step 7: Finish saving bagfile and metadata
    stop_rosbag(command, rosbag_process)
    print("\n... Stop recording Rosbag")
    time.sleep(1)
    # Save metadata in yaml file
    gripper.save_metadata(filename)
    print("\n... Saving metadata in *yaml file")


if __name__ == '__main__':
    # main()
    # scan_apples()
    # real_picks()
    # just record camera

    pressure_servoing()
