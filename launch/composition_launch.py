"""
Dynamically compose OpencvCamNode and ImageSubscriberNode in a component_container.

Limitations of this container:
 -- use_intra_process_comms is set to false
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    movie = 'test.mov'
    movie = '/home/clyde/Desktop/4K_camera/wet/20180101_133138.MOV'
    camera_info_path = 'info.txt'
    camera_info_path = '/home/clyde/ros2/orca_ws/install/orca_driver/share/orca_driver/cfg/brusb_info.txt'

    container = ComposableNodeContainer(
        node_name='my_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='opencv_cam',
                node_plugin='opencv_cam::OpencvCamNode',
                node_name='image_publisher',
                parameters=[{
                    'str_api': True,
                    'filename': movie,
                    'camera_info_path': camera_info_path,
                }]
            ),
            ComposableNode(
                package='opencv_cam',
                node_plugin='opencv_cam::ImageSubscriberNode',
                node_name='image_subscriber',
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
