# opencv_cam

A simple [ROS2](https://index.ros.org/doc/ros2/) camera driver based on [OpenCV](https://opencv.org/).

Tested with Ubuntu Bionic, OpenCV 3.2 and ROS2 Dashing.

## Install and build

~~~
mkdir ~/ros2/opencv_cam_ws/src
cd ~/ros2/opencv_cam_ws/src
git clone https://github.com/clydemcqueen/opencv_cam.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ~/ros2/opencv_cam_ws/
source /opt/ros/dashing/setup.bash
colcon build
~~~

## Usage

Default is to publish images from `/dev/video0`:
~~~
ros2 run opencv_cam opencv_cam_node
~~~

A more complex example:
~~~
ros2 run opencv_cam opencv_cam_node /image_raw:=/my_camera/image_raw __params:=opencv_cam_params.yaml
~~~
... where opencv_cam_params.yaml is:
~~~
/opencv_cam:
  ros__parameters:
    str_api: True
    filename: 'my_camera.MOV'
    camera_info_path: 'my_camera_info.txt'
    camera_frame: 'my_camera'
~~~
... and my_camera_info.txt is:
~~~
480 640
464.015707 462.756973
353.670391 247.734851
0.007742 -0.008235 -0.000835 0.002624 0.000000
~~~

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| str_api | bool | False | True: call VideoCapture(filename, api), False: call VideoCapture(api) |
| filename | string | "" | Movie filename, ignored if str_api is False |
| api | int | 0 | API, see cv::VideoCaptureAPIs for details |
| camera_info_path | string | "" | Camera info path |
| camera_frame | string | "camera_frame" | Camera frame id |

## Camera info file format

2 ints and 9 floats, separated by whitespace:
~~~
height width fx fy cx cy k1 k2 t1 t2 k3
~~~