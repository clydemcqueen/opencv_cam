# Build the image:
# docker build --build-arg ROS_DISTRO=$ROS_DISTRO --tag opencv_cam:$ROS_DISTRO .

# Run a test using /dev/video0:
# docker run -it --device=/dev/video0:/dev/video0 opencv_cam:$ROS_DISTRO

# Interactive session with Rocker (https://github.com/osrf/rocker):
# rocker --x11 --nvidia --devices /dev/video0 -- opencv_cam:$ROS_DISTRO bash

ARG ROS_DISTRO=foxy

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO

RUN apt-get update && apt-get upgrade -y

RUN apt-get install wget

WORKDIR /work/opencv_cam_ws

ARG ROS2_SHARED_BRANCH=master
RUN git clone https://github.com/ptrmu/ros2_shared.git -b $ROS2_SHARED_BRANCH

# Changes to opencv_cam will cause a re-build
COPY . src/opencv_cam/

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Requires a webcam on /dev/video0:
CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 run opencv_cam opencv_cam_main"]

# View images:
# ros2 run image_tools showimage --ros-args -p reliability:=reliable -r image:=/image_raw