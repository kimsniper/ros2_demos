# ROS2 - LAUNCH FILES

Basic utilities, commands and tools when working with Launch files

Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

## Basic Commands

### Launch any URDF file using the urdf_tutorial package

This is useful to quickly launch any URDF file for visualization.

Command: ros2 launch urdf_tutorial display.launch.py model:=< urdf_file_path >/< urdf_filemame >

```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/kim/ros2/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
```

**Note: The urdf_tutorial package should be installed

```bash
sudo apt install ros-humble-urdf-tutorial
```