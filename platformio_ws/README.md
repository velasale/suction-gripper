# 🛠️ Enabling Multiple ROS Services in Micro-ROS

By default, both `micro-ros-platformio` and `micro-ros-arduino` **only support one ROS service**.

If you want to enable **more than one**, follow these steps using PlatformIO:

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
