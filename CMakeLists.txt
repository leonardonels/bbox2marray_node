cmake_minimum_required(VERSION 3.8)
project(bbox2marray)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_library(bbox2marray src/bbox2marray.cpp)
ament_target_dependencies(
  bbox2marray
  rclcpp
  sensor_msgs
  vision_msgs
  visualization_msgs
)

add_executable(bbox2marray_node src/bbox2marray_node.cpp)
ament_target_dependencies(
  bbox2marray_node
  rclcpp
  sensor_msgs
  vision_msgs
  visualization_msgs
)

target_link_libraries(bbox2marray_node
  bbox2marray
)

install(TARGETS
  bbox2marray_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
