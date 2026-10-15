cmake_minimum_required(VERSION 3.0.2)
project(vla_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  VLAAction.msg
  VLAHeartbeat.msg
  VLAStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/vla_image_node.py
  scripts/vla_infer_client_node.py
  scripts/vla_cmd_node.py
  scripts/vla_watchdog_node.py
  scripts/mock_inference_server.py
  scripts/publish_fake_observation.py
  scripts/vla_record_session.py
  scripts/bag_to_uavflow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
