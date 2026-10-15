cmake_minimum_required(VERSION 3.0.2)
project(uav_rtsp_stream)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV 4 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

# System -dev package or vendored headers under deps/
set(UAV_RTSP_VENDORED_INC
  "${CMAKE_CURRENT_LIST_DIR}/deps/gstrtspserver/include/gstreamer-1.0")
if(EXISTS "${UAV_RTSP_VENDORED_INC}/gst/rtsp-server/rtsp-server.h")
  message(STATUS "uav_rtsp_stream: using vendored RTSP headers at ${UAV_RTSP_VENDORED_INC}")
  set(GST_RTSP_INCLUDE_DIRS "${UAV_RTSP_VENDORED_INC}")
else()
  pkg_check_modules(GST_RTSP REQUIRED gstreamer-rtsp-server-1.0)
endif()

find_library(GST_RTSP_SERVER_LIB
  NAMES gstrtspserver-1.0
  PATHS /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
)
if(NOT GST_RTSP_SERVER_LIB)
  set(GST_RTSP_SERVER_LIB "/usr/lib/x86_64-linux-gnu/libgstrtspserver-1.0.so.0")
endif()
if(NOT EXISTS "${GST_RTSP_SERVER_LIB}")
  message(FATAL_ERROR
    "libgstrtspserver-1.0 not found; apt install libgstrtspserver-1.0-0 "
    "(and optionally libgstrtspserver-1.0-dev), or run scripts/fetch_gst_rtsp_deps.sh")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_rtsp_stream_core
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_RTSP_INCLUDE_DIRS}
)

add_library(uav_rtsp_stream_core
  src/encoder_profile.cpp
  src/low_latency_rtsp_streamer.cpp
)

target_link_libraries(uav_rtsp_stream_core
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
  ${GST_RTSP_SERVER_LIB}
)

add_executable(rtsp_server_node src/rtsp_server_node.cpp)
target_link_libraries(rtsp_server_node
  uav_rtsp_stream_core
  ${catkin_LIBRARIES}
)

install(TARGETS uav_rtsp_stream_core rtsp_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
