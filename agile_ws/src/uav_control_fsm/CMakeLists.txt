cmake_minimum_required(VERSION 3.0.2)
project(uav_control_fsm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  message_generation
)
find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(
  FILES
  uav_command.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs nav_msgs roscpp std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 动态库：odometry_bridge 实现（按定位源订阅里程计并转发到 mavros/vision_pose/pose）
add_library(uav_odometry_bridge SHARED src/odometry_bridge.cpp)
add_dependencies(uav_odometry_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_odometry_bridge ${catkin_LIBRARIES})

# 动态库：control_manager 实现（含 Move_mode=0 软件平滑）
add_library(uav_control_manager SHARED src/control_manager.cpp)
add_dependencies(uav_control_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_control_manager ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/uav_control_fsm_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node uav_odometry_bridge uav_control_manager ${catkin_LIBRARIES})

add_executable(uav_path_recorder_node src/uav_path_recorder_node.cpp)
add_dependencies(uav_path_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_path_recorder_node ${catkin_LIBRARIES})
