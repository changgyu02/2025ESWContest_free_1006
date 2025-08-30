cmake_minimum_required(VERSION 3.10)
project(drive_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateUwbPose.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(goal_server_node src/goal_server_node.cpp)
ament_target_dependencies(goal_server_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(go_home_node src/go_home_node.cpp)
ament_target_dependencies(go_home_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs              
  tf2
  tf2_geometry_msgs
  std_srvs
)

add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  OpenCV
  yaml-cpp
)
target_link_libraries(planner_node yaml-cpp)

add_executable(map_to_odom_tf_broadcaster src/map_to_odom_tf_broadcaster.cpp)
ament_target_dependencies(map_to_odom_tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(goal_server_node ${cpp_typesupport_target})
target_link_libraries(go_home_node ${cpp_typesupport_target})

install(TARGETS
  controller_node
  goal_server_node
  planner_node
  go_home_node
  map_to_odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${cpp_typesupport_target})
ament_export_targets(${cpp_typesupport_target})
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
