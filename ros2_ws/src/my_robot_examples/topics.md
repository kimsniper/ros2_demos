# ROS2 - TOPICS

Basic utilities, commands and tools when working with ROS2 topics 

Reference: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html

## To test the following commands

Run any publisher node in the my_robot_examples package

```bash
ros2 run my_robot_examples robot_news_publisher
```

## Basic Commands

### List active topics

While a single or multiple active topic nodes were executed using `ros2 run` command, run the following command

```bash
ros2 topic list
```

### Get topic info

While a single or multiple active topic nodes were executed using `ros2 run` command, run the following command

Command: ros2 topic info <topic_name>

```bash
ros2 topic info /robot_news
```

or for extended info

Command: ros2 topic info <topic_name> --verbose

```bash
ros2 topic info /robot_news --verbose
```

### Get topic publisher frequency (hz)

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 topic hz <topic_name>

```bash
ros2 topic hz /robot_news
```

### Test topic publisher using command line subscriber

This command will use command line as subscriber to listen to a certain topic

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 topic echo <topic_name>

```bash
ros2 topic echo /robot_news 
```

### Test topic publisher node using topic subscriber node

This command will use command line as subscriber to listen to a certain topic

While the robot_news_publisher node is running, run the subscriber node

```bash
ros2 my_robot_examples robot_news_subscriber
```

### Test topic publisher node using command line publisher

This command will use command line as subscriber to listen to a certain topic

While the robot_news_subscriber node is running, run the following in the cmd line

Command: ros2 topic pub <topic_name> <msg_type> <msg_args>

```bash
ros2 topic pub /robot_news example_interfaces/msg/String "data: 'Hello'" 
```