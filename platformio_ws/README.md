# How to enable more than ONE ROS services
micro-ros-platformio and micro-ros-arduino come by default with just ONE ROS service enabled.

The only way that I managed to modify was by using platformio, and doing these steps:

* Build Workspace
* Look for directory .pio/libdeps/esp32dev/micro_ros_platformio/metas
* Open colcon.meta
* Modify the argument DRMW_UXRCE_MAX_SERVICES 
* Manually delete folder .pio/libdeps/esp32dev/micro_ros_platformio/libmicroros
* Build Workspace again
