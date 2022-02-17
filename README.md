# opencv_cam

A simple [ROS2](https://index.ros.org/doc/ros2/) camera driver based on [OpenCV](https://opencv.org/).

Supports [ROS2 intra-process comms](https://index.ros.org//doc/ros2/Tutorials/Intra-Process-Communication/).

Requires [ROS2 Foxy or Galactic](https://index.ros.org/doc/ros2/Installation/) and
`ros-$ROS_DISTRO-camera-calibration-parsers`.

## Install and build

~~~
mkdir ~/ros2/opencv_cam_ws/src
cd ~/ros2/opencv_cam_ws/src
git clone https://github.com/clydemcqueen/opencv_cam.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ~/ros2/opencv_cam_ws/
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
~~~

## Usage

Default is to publish images from `/dev/video0`:
~~~
ros2 run opencv_cam opencv_cam_main
~~~

A more complex example:
~~~
ros2 run opencv_cam opencv_cam_main --ros-args --remap /image_raw:=/my_camera/image_raw --params-file opencv_cam_params.yaml
~~~
... where opencv_cam_params.yaml is:
~~~
/opencv_cam:
  ros__parameters:
    file: True
    filename: 'my_camera.MOV'
    camera_info_path: 'my_camera_info.ini'
    camera_frame_id: 'my_camera'
~~~

### Intra-process comms

IPC test -- CLI composition:
~~~
# First shell
ros2 run rclcpp_components component_container

# Second shell (ignore the deprecation warning, see https://github.com/ros2/ros2cli/issues/336)
ros2 component load /ComponentManager opencv_cam opencv_cam::ImageSubscriberNode -e use_intra_process_comms:=true
ros2 component load /ComponentManager opencv_cam opencv_cam::OpencvCamNode -e use_intra_process_comms:=true
~~~

Launch file composition:
~~~
ros2 launch opencv_cam composition_launch.py
~~~

Manual composition -- handy for debugging:
~~~
ros2 run opencv_cam ipc_test_main
~~~

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| file | bool | False | Read from file vs. read from device |
| fps | int | 0 | Framerate. Specify 0 to publish at the recorded (file) or default (device) framerate  |
| filename | string | "" | Filename, ignored if file is False |
| index | int | 0 | Device index, 0 for /dev/video0. Ignored if file is True |
| width | int | 0 | Device width in pixels. Specify 0 for default. Ignored if file is True |
| height | int | 0 | Device width in pixels. Specify 0 for default. Ignored if file is True |
| camera_info_path | string | "info.ini" | Camera info path |
| camera_frame | string | "camera_frame" | Camera frame id |

## Camera info file formats

Uses the [ROS standard camera calibration formats](http://wiki.ros.org/camera_calibration_parsers).
Files must end in `.ini` or `.yaml`.