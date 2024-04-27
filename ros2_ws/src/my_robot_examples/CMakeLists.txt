cmake_minimum_required(VERSION 3.8)
project(my_robot_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_publisher src/robot_news_publisher.cpp)
ament_target_dependencies(robot_news_publisher rclcpp example_interfaces)

add_executable(robot_news_subscriber src/robot_news_subscriber.cpp)
ament_target_dependencies(robot_news_subscriber rclcpp example_interfaces)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp my_robot_interfaces)

add_executable(compute_rectangle_area_server src/compute_rectangle_area_server.cpp)
ament_target_dependencies(compute_rectangle_area_server rclcpp my_robot_interfaces)

add_executable(compute_rectangle_area_client src/compute_rectangle_area_client.cpp)
ament_target_dependencies(compute_rectangle_area_client rclcpp my_robot_interfaces)

add_executable(battery src/battery.cpp)
ament_target_dependencies(battery rclcpp my_robot_interfaces)

add_executable(led_panel src/led_panel.cpp)
ament_target_dependencies(led_panel rclcpp my_robot_interfaces)

install(TARGETS
  cpp_node
  robot_news_publisher
  robot_news_subscriber
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client
  hw_status_publisher
  compute_rectangle_area_server
  compute_rectangle_area_client
  battery
  led_panel
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
