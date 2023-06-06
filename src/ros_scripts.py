# --- Standard Library Imports
import subprocess, shlex, psutil
import os


# --- 3rd party imports
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


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
    """Open all bagfiles in a folder and saves all topics as csvs"""

    if file.endswith(".bag"):
        print(file)
        bag = bagreader(file)
        # print("\n\n", file)

        # --- Get the topics available in the bagfile ---
        topics = bag.topic_table
        # print("Bagfile topics:\n", topics)

        # --- Read the desired topic ---
        for topic in topics["Topics"]:
            if topic != '/usb_cam/image_raw':
                # Once opened, data is saved automatically into a csv file.
                data = bag.message_by_topic(topic)
            else:
                pass
