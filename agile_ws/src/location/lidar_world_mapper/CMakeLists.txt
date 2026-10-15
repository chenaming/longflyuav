cmake_minimum_required(VERSION 3.0.2)
project(lidar_world_mapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf
  message_filters
  std_srvs
  livox_ros_driver2
)

find_package(PCL REQUIRED COMPONENTS common filters io)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs pcl_ros pcl_conversions tf message_filters std_srvs livox_ros_driver2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_world_mapper_node src/lidar_world_mapper_node.cpp)
target_link_libraries(lidar_world_mapper_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(lidar_world_mapper_node ${catkin_EXPORTED_TARGETS})
