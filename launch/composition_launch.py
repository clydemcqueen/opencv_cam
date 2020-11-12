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
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='opencv_cam',
                plugin='opencv_cam::OpencvCamNode',
                name='image_publisher',
                parameters=[{
                    'file': True,
                    'filename': movie,
                    'camera_info_path': camera_info_path,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='opencv_cam',
                plugin='opencv_cam::ImageSubscriberNode',
                name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
