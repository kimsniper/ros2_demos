# RVIZ - RVIZ FILE

.rviz config file defines the defaut appearance of the urdf file in RViz everytime it is launch.

### Steps

1. Launch the RViz launch.xml file.

```bash
ros2 launch my_robot_bringup my_robot_rviz.launch.xml
```

2. Modify the RViz interface based on desired appearance.

3. In RViz tab, navigate to File->Save Config As

4. Save the file in this directory

5. Integration of the .rviz config file is demonstrated in my_robot_bringup/my_robot_rviz.launch.xml