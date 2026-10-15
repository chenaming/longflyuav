cmake_minimum_required(VERSION 3.0.2)
project(traj_tracking_demo)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  uav_control_fsm
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp std_msgs uav_control_fsm visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(traj_generators src/traj_generators.cpp)
target_link_libraries(traj_generators ${catkin_LIBRARIES})

add_executable(traj_tracking_node
  src/traj_tracking_node.cpp
  src/traj_tracking_main.cpp
)
add_dependencies(traj_tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_tracking_node traj_generators ${catkin_LIBRARIES})

install(TARGETS traj_generators traj_tracking_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
