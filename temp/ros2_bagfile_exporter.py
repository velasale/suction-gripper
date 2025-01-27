# https://github.com/fishros/ros2bag_convert/blob/main/README_EN.md

import os
import subprocess

def find_and_convert_bagfiles(start_path):
    """
    Recursively search for .db3 files starting from start_path and run `ros2bag-convert` on each file.
    
    Args:
        start_path (str): The root directory to start searching for ROS 2 bagfiles.
    """
    # List to hold paths of found bagfiles
    bagfile_paths = []

    # Walk through all directories and subdirectories
    for root, dirs, files in os.walk(start_path):
        for file in files:
            if file.endswith(".db3"):
                # Add full path to the list of bagfiles
                bagfile_paths.append(os.path.join(root, file))
    
    if not bagfile_paths:
        print("No ROS 2 bagfiles (.db3) found.")
        return

    print(f"Found {len(bagfile_paths)} ROS 2 bagfiles:")
    for bagfile in bagfile_paths:
        print(f"- {bagfile}")

    # Process each bagfile
    for bagfile in bagfile_paths:
        try:
            print(f"Converting bagfile: {bagfile}")
            # Run the `ros2bag-convert` command
            subprocess.run(["ros2bag-convert", bagfile], check=True)
            print(f"Successfully converted: {bagfile}")
        except subprocess.CalledProcessError as e:
            print(f"Error converting {bagfile}: {e}")

if __name__ == "__main__":
    # Note: Install this repository to use the `ros2bag-convert` command
    # https://github.com/fishros/ros2bag_convert/blob/main/README_EN.md
    
    # Change this to the mount point of your USB drive
    usb_drive_path = "/media/alejo/VELASALE/Data"
    find_and_convert_bagfiles(usb_drive_path)
