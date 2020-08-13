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
  message(STATUS "Running inside CLion")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif ()

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# OpenCV camera node
#=============

add_library(
  opencv_cam_node SHARED
  src/opencv_cam_node.cpp
)
target_compile_definitions(
  opencv_cam_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  opencv_cam_node
  camera_calibration_parsers
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
)
rclcpp_components_register_nodes(opencv_cam_node "opencv_cam::OpencvCamNode")
set(node_plugins "${node_plugins}opencv_cam::OpencvCamNode;$<TARGET_FILE:opencv_cam_node>\n")

#=============
# Test subscriber node
#=============

add_library(
  subscriber_node SHARED
  src/subscriber_node.cpp
)
target_compile_definitions(
  subscriber_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  subscriber_node
  class_loader
  rclcpp
  sensor_msgs
)
rclcpp_components_register_nodes(subscriber_node "opencv_cam::ImageSubscriberNode")
set(node_plugins "${node_plugins}opencv_cam::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

#=============
# Export incantations, see https://github.com/ros2/demos/blob/master/composition/CMakeLists.txt
#=============

if (NOT WIN32)
  ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif ()

#=============
# OpenCV camera main, IPC=true
#=============

add_executable(
  opencv_cam_main
  src/opencv_cam_main.cpp
  src/opencv_cam_node.cpp
)
target_link_libraries(
  opencv_cam_main
#  opencv_cam_node
)
ament_target_dependencies(
  opencv_cam_main
  camera_calibration_parsers
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
  rclcpp
)

#=============
# Manual composition of camera and subscriber nodes, IPC=true
#=============

add_executable(
  ipc_test_main
  src/ipc_test_main.cpp
)
target_link_libraries(
  ipc_test_main
  opencv_cam_node
  subscriber_node
)
ament_target_dependencies(
  ipc_test_main
  rclcpp
)

#=============
# Install
#=============

# Install nodes
install(
  TARGETS opencv_cam_node subscriber_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS opencv_cam_main ipc_test_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
