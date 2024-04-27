# ROS2 - ACTIONS

Basic utilities, commands and tools when working with ROS2 Actions 

Reference: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## Basic Commands

### Show action interface definition

Navigate to the ros2 workspace directory and run the following command.

Command: ros2 interface show `package name`/`path of action`/`action file`

```bash
ros2 run my_robot_actions count_until_server_blocking
ros2 interface show my_robot_interfaces/action/CountUntil
```
### List active action interfaces

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

```bash
ros2 action list
```
### Display active action information

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 action info `action name` \
This command should display the number of clients and services as well as the name of the node where it is created

```bash
ros2 run my_robot_actions count_until_server_blocking
ros2 action info /count_until
```
### Display active action topics

Actions actually provide topics hidden from a user's perspective because we dont deal with these topics directly. \
But for some reason, we may use them such as subscribing to these topics. 

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 topic list --include-hidden-topics
```
### Display active action services

Actions actually provide topics hidden from a user's perspective because we dont deal with these services directly. \
But for some reason, we may use them such as using these services. 

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 service list --include-hidden-services
```

### List active action with interfaces

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 action list -t
```

### Test action server through command line

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 action send_goal `action name` `package name`/`path of action`/`action file` `parameter arguments` 

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_blocking
```

In another terminal, execute the following command:

```bash
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 7, period: 1}"
```

Execute with feedback

```bash
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 7, period: 1}" --feedback
```

This test will print the counter in server terminal with a period of 1 second.

### Test action server through action client

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_blocking
```

In another terminal, execute the following command:

```bash
ros2 run my_robot_actions count_until_client
```

This test will print the counter in server terminal with a period of 1 second.

### Test goal canceling in action server through action client

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_non_blocking
```

In another terminal, execute the following command:

```bash
ros2 run my_robot_actions count_until_client_cancel_test
```

This test will print the counter in server terminal with a period of 1 second. Timer will be initialized to trigger \
the timer callback after 2 seconds, then a cancellation request will be executed.

### Test multiple goal requests in action server through multiple action client

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_non_blocking
```

In other two terminals, execute the following command on each terminal:

```bash
ros2 run my_robot_actions count_until_client
```

This test will print the counter in server terminal with a period of 1 second to two clients and one server. To test multiple clients, ensure that \
client nodes will be executed at the same time.

### Test action server through action client to only execute a single goal request

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_single_request
```

In other two terminals, execute the following command on each terminal:

```bash
ros2 run my_robot_actions count_until_client
```

This test will implement a single goal request policy. To test multiple clients, ensure that \
client nodes will be executed at the same time. The client node that was executed later than the other client node will get rejected.

### Test action server through action clients to preempt the current goal execution and run the new goal request

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_preempt_current_goal
```

In other two terminals, execute the following command on each terminal:

```bash
ros2 run my_robot_actions count_until_client
```

This test will implement a current goal execution preemption policy. To test multiple clients, ensure that \
client nodes will be executed at the same time. The client node that was executed first will be preempted and the later goal \
request will be executed.

### Test action server through action clients to add every request in queue and execute them one at a time

In a terminal execute the following command

```bash
ros2 run my_robot_actions count_until_server_queue_goals
```

In other two terminals, execute the following command on each terminal:

```bash
ros2 run my_robot_actions count_until_client
```

This test will implement a current goal queueing policy. To test multiple clients, ensure that \
client nodes will be executed at the same time. All client node goal requests will be added to the queue with the first \
request be executed first.