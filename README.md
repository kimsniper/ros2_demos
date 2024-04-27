# ROS2 - Robot Operating System 2 For Beginners

A beginner's guide to start working with projects based on ROS2 using C++

## Overview

This reposotory contains useful ROS2 demos.

### Preferred Environment Setup

To run this example without issues, the following environment setup are preferred.

|                  |                      |
|------------------|----------------------|
| Operating System | UBUNTU 22.04 (Jammy) |
| ROS2 Version     | ROS2 Humble          |

**Note: ** In my case, I have an Ubuntu 22.04 boot on my PC.

### Installing ROS2 Humble

ROS2 humble only works well with Ubuntu 22.04. To better follow the installation process \
clone this repo at home directory.

```bash
cd
git clone https://github.com/kimsniper/ros2.git
```

Step 1: Follow installation mentioned in the link below:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Step 2: Navigate to /opt/ros/humble. There is a file named setup.bash, this file should be source at bashrc file

Place the text below at the bottom area of .bashrc file

```bash
source /opt/ros/humble/setup.bash
```

Step 3: Install colcon build tool


```bash
sudo apt install python3-colcon-common-extensions
```

Step 4: Source colcon build tool argcomplete

This is to allow colcon build to have autocompletion. Place the text below at the bottom area of .bashrc file

```bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Step 5: In home directory create the workspace directory and under it, create src directory. Then inside the workspace directory, run the following command

```bash
colcon build
```

Step 6: An additional three directories will be created inside the workspace. Navigate inside install directory and there will be a setup.bash file \

Be sure to source it everytime a new terminal windows is opened

OR

Place the text below at the bottom area of .bashrc file

```bash
source ~/ros2/ros2_ws/install/setup.bash
```

## Colcon build commands

Navigate to the ros2 workspace directory and run the following command. 

Build all packages 

```bash
colcon build
```
Build specific package

```bash
colcon build --packages-select `name_of_package`
```

See the [Colcon Tutorial Link](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) for full steps to configure and build ROS2 packages.

## Installing Gazebo

```bash
sudo apt install ros-humble-gazebo*
```

Then source the setup.bash at .bashrc file

```bash
source /usr/share/gazebo/setup.bash
```

## Installing Package To Run URDF - Joint State Publisher GUI

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

## Installing Package To Enable Xacro

```bash
sudo apt install ros-humble-xacro
```

## Final note

Usage tutorial to every packages is provided in a readme file inside package directories.
