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
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
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
target_compile_definitions(
  opencv_cam_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  opencv_cam_component
  camera_calibration_parsers
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
)
rclcpp_components_register_nodes(opencv_cam_component "opencv_cam::OpencvCamNode")
set(node_plugins "${node_plugins}opencv_cam::OpencvCamNode;$<TARGET_FILE:opencv_cam_component>\n")

#=============
# Test subscriber component
#=============

add_library(
  subscriber_component SHARED
  src/subscriber_component.cpp
)
target_compile_definitions(
  subscriber_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  subscriber_component
  class_loader
  rclcpp
  sensor_msgs
)
rclcpp_components_register_nodes(subscriber_component "opencv_cam::ImageSubscriberNode")
set(node_plugins "${node_plugins}opencv_cam::ImageSubscriberNode;$<TARGET_FILE:subscriber_component>\n")

#=============
# Export incantations, see https://github.com/ros2/demos/blob/master/composition/CMakeLists.txt
#=============

if (NOT WIN32)
  ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif ()

#=============
# OpenCV camera node, IPC=true
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
# Manual composition of camera and subscriber components, IPC=true
#=============

add_executable(
  manual_composition
  src/manual_composition.cpp
)
target_link_libraries(
  manual_composition
  opencv_cam_component
  subscriber_component
)
ament_target_dependencies(
  manual_composition
  rclcpp
)

#=============
# Install
#=============

# Install components
install(
  TARGETS opencv_cam_component subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS opencv_cam_node manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
