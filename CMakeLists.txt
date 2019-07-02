cmake_minimum_required(VERSION 3.5)
project(opencv_cam)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_shared_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

#=============
# OpenCV camera component
#=============

add_library(
  opencv_cam_component SHARED
  src/opencv_cam_component.cpp
)
ament_target_dependencies(
  opencv_cam_component
  cv_bridge
  rclcpp
  sensor_msgs
)

# Export incantations:
# Call ament_export_interfaces()
# Reference this in the install() with an EXPORT line
# Source install/local_setup.bash after the build
ament_export_interfaces(
  opencv_cam_component
  HAS_LIBRARY_TARGET
)

#=============
# OpenCV camera node
#=============

add_executable(
  opencv_cam_node
  src/opencv_cam_node.cpp
)
target_link_libraries(
  opencv_cam_node
  opencv_cam_component
)
ament_target_dependencies(
  opencv_cam_node
  rclcpp
)

#=============
# Install
#=============

## Install C++ targets
install(
  TARGETS opencv_cam_node opencv_cam_component
  EXPORT opencv_cam_component
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

ament_package()
