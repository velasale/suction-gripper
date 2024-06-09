# @Time : 6/6/2023 11:15 AM
# @Author : Alejandro Velasquez

# --- Standard Library Imports
import subprocess, shlex, psutil
import os
from bagpy import bagreader
import datetime
from pathlib import Path

# --- 3rd party imports
import rospy
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker, MarkerArray

import cv2
import rosbag
from cv_bridge import CvBridge


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


def service_call(service):
    """Method to call service from the command line.
    Note: The services are described in the Arduino file"""
    text = "rosservice call " + service
    os.system(text)


# ---------------- ROS MARKER FUNCTIONS ------------------------------#
def place_marker_text(pos=[0, 0, 0], scale=0.01, text='caption', cframe='world'):
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
    caption.header.frame_id = cframe
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


def place_marker(color=[0, 0, 1, 1], pos=[0, 0, 0], scale=0.01, cframe='world'):
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
    sphere.header.frame_id = cframe
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
    sphere.pose.position.x = x
    sphere.pose.position.y = y
    sphere.pose.position.z = z
    sphere.pose.orientation.x = 0.0
    sphere.pose.orientation.y = 0.0
    sphere.pose.orientation.z = 0.0
    sphere.pose.orientation.w = 1.0

    self.proxy_markers.markers.append(sphere)
    # Set up a publisher.  We're going to publish on a topic called balloon.
    self.markerPublisher.publish(self.proxy_markers)

    # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
    rate = rospy.Rate(10)


# ---------------- ROSBAG FUNCTIONS AND METHODS --------------------- #
def start_rosbag(name='trial', topics=""):
    """Convenient method to start saving a bagfile"""

    filename = name

    topics_string = ""
    for topic in topics:
        topics_string = topics_string + " " + topic

    command = "rosbag record -O " + filename + topics_string
    command = shlex.split(command)

    return command, subprocess.Popen(command)


def stop_rosbag(cmd, process):
    """Stop saving rosbag"""

    for proc in psutil.process_iter():
        if "record" in proc.name() and set(cmd[2:]).issubset(proc.cmdline()):
            proc.send_signal(subprocess.signal.SIGINT)
    process.send_signal(subprocess.signal.SIGINT)


def bag_to_csvs(file):
    """
    Opens bagfile and extracts csvs from all topics (except image topics)
    @param file: file path
    @return:
    """

    if file.endswith(".bag"):
        print(file)
        bag = bagreader(file)
        # print("\n\n", file)

        # --- Get the topics available in the bagfile ---
        topics = bag.topic_table
        # print("Bagfile topics:\n", topics)

        # --- Read the desired topic ---
        for topic in topics["Topics"]:
            if topic not in ['/usb_cam/image_raw', '/camera/image_raw']:
                # Once opened, data is saved automatically into a csv file.
                data = bag.message_by_topic(topic)
            else:
                pass


def bag_to_pngs(input_dir, bag_file, cam_topic, output_folder='/pngs'):
    """
    Method to extract images from a bagfile and save them in a folder named PNGS
    @param input_dir: file location
    @param bag_file: filename (including .bag extension)
    @param cam_topic: rostopic with the camera messages
    @return:
    """

    # Create folder to save pngs
    only_filename = bag_file.split(".bag")[0]
    output_dir = input_dir + only_filename + output_folder
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Read bagfile
    bag = rosbag.Bag(input_dir + bag_file, "r")
    bridge = CvBridge()

    count = 0
    initial_time_stamp = 0.0
    for topic, msg, t in bag.read_messages(topics=[cam_topic]):

        # print(t)
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Add white background for the text
        # img_height, img_width, __ = cv_img.shape
        # cv2.rectangle(cv_img, (0, img_height), (250, img_height - 40), (255, 255, 255), -1)

        # Add elapsed time as text at the bottom of the image
        if count == 0:
            initial_time_stamp = t

        elapsed_time = round((t.to_sec() - initial_time_stamp.to_sec()), 1)
        et_string = str(elapsed_time)
        decimals = et_string.split(".")[1]

        if len(decimals) == 0:
            et_string = et_string + '0'
        elif len(decimals) == 1:
            et_string = et_string + ''

        # font = cv2.FONT_HERSHEY_COMPLEX
        font = cv2.FONT_HERSHEY_DUPLEX
        font_color = (0,0,0)     # BGR
        cv2.putText(cv_img, 'time: ' + et_string + ' s', (int(cv_img.shape[0] * 0.025), int(cv_img.shape[1] * 0.73)), font, 1.0, font_color, 1, cv2.LINE_AA)

        # Save file
        cv2.imwrite(os.path.join(output_dir, str(int(elapsed_time*10)) + ".png"), cv_img)
        print("Wrote image %i" % count)

        count += 1

    bag.close()


def bag_to_video(input_dir, bag_file, cam_topic):
    """
    Method to extract video from a camera rostopic
    @param input_dir: file location
    @param bag_file: file (including .bag extension)
    @param cam_topic:
    @return:
    """

    # Create folder to save avi
    only_filename = bag_file.split(".bag")[0]
    output_dir = input_dir + only_filename + "/"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Open and read bagfile
    bag = rosbag.Bag(input_dir + bag_file)
    bridge = CvBridge()

    out = None
    count = 0
    initial_time_stamp = 0.0
    for topic, msg, t in bag.read_messages(topics=[cam_topic]):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = img.shape

        # Add elapsed time as text at the bottom of the image
        if count == 0:
            initial_time_stamp = t

        elapsed_time = round((t.to_sec() - initial_time_stamp.to_sec()), 3)
        print(elapsed_time)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_color = (67, 211, 255)  # BGR
        cv2.putText(img, 'time: ' + str(elapsed_time) + ' sec',
                    (int(img.shape[0] * 0.05), int(img.shape[1] * 0.73)), font, 0.5, font_color, 1,
                    cv2.LINE_AA)

        if out is None:
            fps = bag.get_type_and_topic_info()[1][cam_topic][3]
            print("\nFrames per second: ", fps)
            out = cv2.VideoWriter(output_dir + only_filename + '.avi', cv2.VideoWriter_fourcc(*'MP4V'), fps, (w, h))
        out.write(img)

        count += 1

    bag.close()

    out.release()


# ---------------- FILE NAMING CONVENTION ---------------------------- #
def datetime_simplified():
    """Convenient method to adapt datetime """
    year = datetime.datetime.now().year
    month = datetime.datetime.now().month
    day = datetime.datetime.now().day
    hour = datetime.datetime.now().hour
    minute = datetime.datetime.now().minute

    day = str(year) + str(month//10) + str(month%10) + str(day)
    time = str(hour//10) + str(hour%10) + str(minute//10) + str(minute%10)

    return(day)


def main():

    #################### Small tutorial to use 'bag_to_video' #################
    # --- Step 1: Provide file ---

    # --- Lab's Laptop - Hard Drive A ---
    folder = '/home/alejo/gripper_ws/src/suction-gripper/data/'
    # --- Lab's Laptop - Hard Drive B ---
    # folder = "/media/alejo/DATA/SUCTION_GRIPPER_EXPERIMENTS/"
    # --- Lab's Laptop - Hard Drive C ---
    folder = '/media/alejo/042ba298-5d73-45b6-a7ec-e4419f0e790b/home/avl/data/REAL_APPLE_PICKS/'

    # subfolder = "LOW_STIFFNESS/"
    # subfolder = "MEDIUM_STIFFNESS/"
    # subfolder = "HIGH_STIFFNESS/"
    subfolder = ''

    # file = '2023096_realapple9_attempt3.bag'
    # file = '20230920_realapple2_orientation_0_yaw_0.bag'
    # file = '20230922_realapple2_attempt_2_orientation_0_yaw_0.bag'

    # --- Trials used for ICRA24 media ---
    storage = '/media/alejo/Elements/'
    folder = 'Alejo - Apple Pick Data/Apple Proxy Picks/04 - 2023 summer - suctionCup gripper/LOW STIFFNESS/'
    location = storage + folder

    # --- Prosser Data ---
    storage = '/media/alejo/Elements'
    folder = '/Alejo - Apple Pick Data/Real Apple Picks/05 - 2023 fall (Prosser-WA)/Dataset - apple picks/'
    location = storage + folder

    file = '2023111_realapple7_mode_dual_attempt_1_orientation_0_yaw_0.bag'


    # --- Proxy Data ---
    folder = '/media/alejo/Elements/Alejo - Apple Pick Data/Apple Proxy Picks/05 - 2024 winter - finger and dual trials/FINGER_GRIPPER_EXPERIMENTS_rep2/'

    # file = '20230731_proxy_sample_4_yaw_45_rep_0_stiff_low_force_medium.bag'
    # file = '20230731_proxy_sample_5_yaw_45_rep_0_stiff_low_force_medium.bag'
    # file = '20230731_proxy_sample_5_yaw_45_rep_0_stiff_low_force_low.bag'
    # file = '2023082_proxy_sample_5_yaw_45_rep_0_stiff_high_force_low.bag'
    # file = '2023083_proxy_sample_5_yaw_45_rep_1_stiff_high_force_medium.bag'

    # file = '20230922_realapple3_attempt_1_orientation_0_yaw_0.bag'
    # file = '20230922_realapple2_attempt_1_orientation_0_yaw_0.bag'
    # file = '2023111_realapple21_mode_dual_attempt_3_orientation_0_yaw_0.bag'

    # --- Data Location ---
    # folder = '/media/alejo/Elements/Prosser_Data/'
    # subfolder = 'Dataset - apple grasps/'

    open_batch = 'yes'

    if open_batch == 'yes':
        # --- Opening bags in a batch ---
        for filename in os.listdir(folder + subfolder):
            if filename.endswith(".bag"):
                print(filename)

                # --- Skip if there is already a folder
                just_name = filename.split('.', 1)[0]
                print(just_name)
                if Path(folder + subfolder + just_name).is_dir():
                    print('there is already a folder with this name')
                    continue

                # --- Extract topics
                print('there is not a folder yet')
                topic = "/camera/image_raw"
                # bag_to_pngs(folder + subfolder, filename, topic,'/pngs_inhand_cam')

                topic = "/usb_cam/image_raw"
                # bag_to_pngs(folder + subfolder, filename, topic, '/pngs_fixed_cam')

                # bag_to_video(folder + subfolder, filename, topic)

                bag_to_csvs(folder + subfolder + filename)

    else:
        # --- Open a single bag file ---
        # file = '2023111_realapple21_mode_dual_attempt_3_orientation_0_yaw_0.bag'
        #
        # # --- Step 2: You may save the png files as well ---
        topic = "/camera/image_raw"
        bag_to_pngs(location, file, topic, '/pngs_inhand_cam')

        topic = "/usb_cam/image_raw"
        bag_to_pngs(location, file, topic, '/pngs_fixed_cam')

        bag_to_csvs(location + file)
        # bag_to_video(location, file, topic)


if __name__ == '__main__':
    main()


