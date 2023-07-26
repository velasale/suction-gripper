# Suction Gripper Experiment
Experiments to test the performance of the suction gripper to pick apples in an physical apple proxy. 
Both cartesian and angular offsets are added to the position of the gripper w.r.t. the center of an apple.


# Basic Installation

The following steps were performed under Ubuntu 20.04.5 LTS (Focal Fossa)(https://www.releases.ubuntu.com/focal/)

### Arduino  
1. Install Arduino  
2. Search and install the libraries:
    * **rosserial**
    * **adafruit mprls** (Atmospheric Pressure Sensor)
    * **adafruit VL53L0X** (Time Of Flight Sensor)
3. Warning:
    * If after compiling you encounter issues with *cstring*, try:  
        - open **msg.h** file
        - replace `#include <cstring>` with `#include<string.h>`  
        - replace `std::memcpy()` with `memcpy()` 
    * If you are using a SAMD21 based Arduino board (e.g. Arduino Zero) then you will need a SAMD21 Compatible **ros_lib** for rosserial which can be found [here](https://github.com/MWahbahCC/ros_lib/tree/main) as suggested [here](https://answers.ros.org/question/328712/rosserial_python-on-samd21/):
        - Simply copy the files **ArduinoHardware.h** and **ros.h** into your host pc *Arduino/libraries/ros_lib* subfolder
        - These upgraded files are found in this repo at *arduino/ros_lib*
     
### ROS
1. Install ROS [noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Create WorkSpace  
```console
mkdir -p your_ws/src && cd catkin_ws
```
3. Don't forget to source the workspace   
`gedit ~/.bashrc` and add this line at the end `source ~/<your_ws>/devel/setup.bash` 

### ROS - Serial

4. Install **rosserial** package  
```console
cd <your_ws>/src  
git clone https://github.com/ros-drivers/rosserial.git  
cd <your_ws>  
catkin_make
```

### ROS - Camera
Install **opencv** or **usb_cam**. The latter allows to adjust color parameters (e.g. saturation, contrast, brightness)

5. Install **opencv** package
```console
cd <your_ws>/src 
git clone https://github.com/ros-drivers/video_stream_opencv
cd <your_ws>
catkin_make
```

6. Install **usb_cam** package if you have issues with Color
```console
cd <your_ws>/src
git clone https://github.com/ros-drivers/usb_cam 
cd <your_ws>
catkin_make
```

### ROS - MoveIt

7. Install **moveit** package
```console
sudo apt install ros-noetic-moveit
```

# Running
1. Upload code into **arduino board**.
2. Check the port name, and make sure it matches the one written at line 50 of **serial_node.py**.
3. Launch the lab setup in 1st terminal (modify the parameter **with_robot** depending on whether you are working or not (i.e. yes, no) with real ur5e.
```console
roslaunch suction-gripper suction_gripper_experiment.launch with_robot:=no
```

4. Run experiment code in 2nd terminal:
```console
python3 suction_experiment.py
 ```

## Tips  
If you want to read a certain sensor/topic from command line:
```console
rostopic echo /gripper/pressure
```
Also, if you want to send a control command from command line:
```console
rosservice call openValve
```
or
```console
rosservice call closeValve
```

Tips for the camera:
https://askubuntu.com/questions/348838/how-to-check-available-webcams-from-the-command-line