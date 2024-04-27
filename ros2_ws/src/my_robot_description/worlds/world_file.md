# GAZEBO - WORLD FILE

.world config file defines the defaut appearance of the urdf file in gazebo everytime it is launch.

### Steps

1. Launch the gazebo launch.xml file.

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

2. Modify the gazebo interface based on desired appearance. Or add objects withing the gazebo world

3. In gazebo tab, navigate to File->Save World As

4. Save the file in this directory

5. Integration of the .world config file is demonstrated in my_robot_bringup/my_robot_gazebo.launch.xml