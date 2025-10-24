# 🛠️ Enabling Multiple ROS Services in Micro-ROS

By default, both `micro-ros-platformio` and `micro-ros-arduino` **only support one ROS service**.

This grippers should provide two services: switch valve, operate fingres. Hence, make sure to enable **more than one**, following these steps using PlatformIO:

---

## ✅ Steps to Enable Multiple Services

1. **Build the PlatformIO Workspace**  
   This generates the necessary dependency folders.

2. **Navigate to the `colcon.meta` File**  
   Go to:  
```bash
.pio/libdeps/esp32dev/micro_ros_platformio/metas/colcon.meta
```


3. **Edit the Maximum Services Parameter**  
Inside `colcon.meta`, find and modify the following line:
```yaml
"DRMW_UXRCE_MAX_SERVICES": <your_desired_number>
```

4. **Clean the Micro-ROS Library**  
Manually delete the folder to force the rebuild:

```bash
rm -rf .pio/libdeps/esp32dev/micro_ros_platformio/libmicroros
```

5. **Rebuild the Workspace**
This will regenerate libmicroros with your updated configuration.

---
## ✅ Notes about ROS2 transports

1. **WIFI TRANSPORTS**  
   Make sure to **include** or **uncomment** the following line inside `platform.ini`
   ```yaml
   board_microros_transport = wifi
   ```
   
   If you want to use your PC as a wifi hotspot, a quick way to set it up is through:

   ```bash
   nmcli device wifi hotspot ifname <wdev> ssid <ssidname> password <password>
   ```

   For example, in our case is:
   ```bash
   nmcli device wifi hotspot ifname wlp108s0f0 ssid alejos password harvesting
   ```

   And then run the micro-ros agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ``` 

2. **SERIAL TRANSPORTS**  
   Make sure to **exclude** or **comment** the following line inside `platform.ini`
   ```yaml
   ;board_microros_transport = wifi
   ```

   And run the micro-ros agent depending on your case:

   2.1. ***Serial cable***  
   Run the micro-ros agent. Adjust the parameters as needed (i.e. baudrate, device)
   
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 2000000
   ```

   2.2. ***Bluetooth***  
   First look for the IP address *XX:XX:XX:XX:XX:XX* of your ESP32 bluetooth, and pair to it. You can find this in **bluetooth** settings. Then, bind it to the serial port:

   ```bash
   sudo rfcomm bind /dev/rfcomm0 <XX:XX:XX:XX:XX:XX>
   ```
   Finally, run the micro-ros agent:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/rfcomm0
   ```


