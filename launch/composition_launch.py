"""
Dynamically compose OpencvCamNode and ImageSubscriberNode in a component_container.

Limitations of this container:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    movie = 'test.mov'
    camera_info_path = 'info.ini'

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
                    'file': True,
                    'filename': movie,
                    'camera_info_path': camera_info_path,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='opencv_cam',
                node_plugin='opencv_cam::ImageSubscriberNode',
                node_name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
