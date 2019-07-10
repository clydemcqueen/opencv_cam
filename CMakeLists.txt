cmake_minimum_required(VERSION 3.5)
project(opencv_cam)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Emulate colcon by providing paths to other projects in the workspace
if ($ENV{CLION_IDE})
  message("Running inside CLion")
  set(CMAKE_LIBRARY_PATH "/opt/ros/dashing/lib") # Fixes CMake error and link error
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if (_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach (_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach ()
endif ()

include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${ros2_shared_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# OpenCV camera component
#=============

add_library(
  opencv_cam_component SHARED
  src/opencv_cam_component.cpp
)
ament_target_dependencies(
  opencv_cam_component
  class_loader
  cv_bridge
  rclcpp
  sensor_msgs
)
rclcpp_components_register_nodes(opencv_cam_component "opencv_cam::OpencvCamNode")
set(node_plugins "${node_plugins}opencv_cam::OpencvCamNode;$<TARGET_FILE:OpencvCamNode>\n")

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
# IPC test nodes
#=============

add_executable(
  ipc_test_nodes
  src/ipc_test_nodes.cpp
)
target_link_libraries(
  ipc_test_nodes
  opencv_cam_component
)
ament_target_dependencies(
  ipc_test_nodes
  rclcpp
)

#=============
# Install
#=============

## Install C++ targets
install(
  TARGETS opencv_cam_node ipc_test_nodes opencv_cam_component
  EXPORT opencv_cam_component
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

ament_package()
