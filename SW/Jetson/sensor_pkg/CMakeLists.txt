cmake_minimum_required(VERSION 3.8)
project(sensor_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(
  include
)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(uwb_to_ros src/uwb_to_ros.cpp)
add_executable(uwb_tf_broadcaster src/uwb_tf_broadcaster.cpp)
add_executable(odom_publisher src/odom_publisher.cpp)
add_executable(md_control src/MDControl.cpp)
add_executable(ultrasonic_parser_node src/ultrasonic_parser_node.cpp)
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
add_executable(battery_reader_node src/battery_reader_node.cpp)
add_executable(imu_node
  src/imu_node.cpp
  src/bno055.cpp
)
add_executable(uwb_offset_node src/uwb_offset_node.cpp)

ament_target_dependencies(uwb_to_ros rclcpp geometry_msgs Boost)
ament_target_dependencies(uwb_tf_broadcaster rclcpp tf2_ros geometry_msgs)
ament_target_dependencies(odom_publisher rclcpp geometry_msgs sensor_msgs nav_msgs tf2)
ament_target_dependencies(md_control rclcpp geometry_msgs)
ament_target_dependencies(ultrasonic_parser_node rclcpp std_msgs)
ament_target_dependencies(obstacle_detector_node rclcpp std_msgs)
ament_target_dependencies(battery_reader_node rclcpp std_msgs)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  std_srvs
  tf2
)

ament_target_dependencies(uwb_offset_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
)

install(TARGETS
  uwb_to_ros
  uwb_tf_broadcaster
  imu_node
  odom_publisher
  md_control
  ultrasonic_parser_node
  obstacle_detector_node
  battery_reader_node
  uwb_offset_node       
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

