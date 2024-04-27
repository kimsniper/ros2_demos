# ROS2 - LIFECYCLE

Basic utilities, commands and tools when working with ROS2 Lifecycles 

Reference: https://github.com/ros2/demos/blob/humble/lifecycle/README.rst

## To test the following commands

Run any lifecycle node in the my_robot_lifecycle package

```bash
ros2 run my_robot_lifecycle number_publisher
```

## Basic Commands

### List active lifecycle nodes

While a single or multiple active lifecycle nodes were executed using `ros2 run` command, run the following command

```bash
ros2 lifecycle list
```

### List possible lifecycle node states

While a single or multiple active lifecycle nodes were executed using `ros2 run` command, run the following command

Command: ros2 lifecycle list `lifecycle node name` \

```bash
ros2 lifecycle list /number_publisher
```

### Get lifecycle node state

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 lifecycle get `lifecycle node name` \
Depending upon the current state of the lifecycle node, this command will display the current state.

```bash
ros2 lifecycle get /number_publisher
```

### Set lifecycle node state

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Run the command to list the possible transitions for the current state \
Command: ros2 lifecycle set `lifecycle node name` `state transition` \
Depending upon the current state of the lifecycle node, this command will set the node from current state to new transition.

```bash
ros2 lifecycle set /number_publisher configure
```

### Launch lifecycle node and transitioning using launch file

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 launch my_robot_bringup lifecycle_test.launch.xml 
```
