# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "odin_ros_driver: 0 messages, 4 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(odin_ros_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" NAME_WE)
add_custom_target(_odin_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odin_ros_driver" "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" ""
)

get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" NAME_WE)
add_custom_target(_odin_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odin_ros_driver" "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" ""
)

get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" NAME_WE)
add_custom_target(_odin_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odin_ros_driver" "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" ""
)

get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" NAME_WE)
add_custom_target(_odin_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odin_ros_driver" "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_cpp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_cpp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_cpp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver
)

### Generating Module File
_generate_module_cpp(odin_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(odin_ros_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(odin_ros_driver_generate_messages odin_ros_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_cpp _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_cpp _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_cpp _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_cpp _odin_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odin_ros_driver_gencpp)
add_dependencies(odin_ros_driver_gencpp odin_ros_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odin_ros_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_eus(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_eus(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_eus(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver
)

### Generating Module File
_generate_module_eus(odin_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(odin_ros_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(odin_ros_driver_generate_messages odin_ros_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_eus _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_eus _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_eus _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_eus _odin_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odin_ros_driver_geneus)
add_dependencies(odin_ros_driver_geneus odin_ros_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odin_ros_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_lisp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_lisp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_lisp(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver
)

### Generating Module File
_generate_module_lisp(odin_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(odin_ros_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(odin_ros_driver_generate_messages odin_ros_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_lisp _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_lisp _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_lisp _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_lisp _odin_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odin_ros_driver_genlisp)
add_dependencies(odin_ros_driver_genlisp odin_ros_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odin_ros_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_nodejs(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_nodejs(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_nodejs(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver
)

### Generating Module File
_generate_module_nodejs(odin_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(odin_ros_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(odin_ros_driver_generate_messages odin_ros_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_nodejs _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_nodejs _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_nodejs _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_nodejs _odin_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odin_ros_driver_gennodejs)
add_dependencies(odin_ros_driver_gennodejs odin_ros_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odin_ros_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_py(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_py(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver
)
_generate_srv_py(odin_ros_driver
  "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver
)

### Generating Module File
_generate_module_py(odin_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(odin_ros_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(odin_ros_driver_generate_messages odin_ros_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_py _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/GetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_py _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAe.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_py _odin_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/agile/odin_ws/src/odin_ros_driver/srv/SetAwb.srv" NAME_WE)
add_dependencies(odin_ros_driver_generate_messages_py _odin_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odin_ros_driver_genpy)
add_dependencies(odin_ros_driver_genpy odin_ros_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odin_ros_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odin_ros_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(odin_ros_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odin_ros_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(odin_ros_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odin_ros_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(odin_ros_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odin_ros_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(odin_ros_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odin_ros_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(odin_ros_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
