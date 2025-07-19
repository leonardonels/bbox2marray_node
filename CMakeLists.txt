cmake_minimum_required(VERSION 3.8)
project(bbox2marker)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)


include_directories(
  include
)

add_library(bbox2marker src/bbox2marker.cpp)
ament_target_dependencies(
  bbox2marker
  rclcpp
  sensor_msgs
  vision_msgs
  std_msgs
  cv_bridge
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

add_executable(bbox2marker_node src/bbox2marker_node.cpp)
ament_target_dependencies(
  bbox2marker_node
  rclcpp
  sensor_msgs
  vision_msgs
  std_msgs
  cv_bridge
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

target_link_libraries(bbox2marker_node
  bbox2marker
)

install(TARGETS
  bbox2marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
