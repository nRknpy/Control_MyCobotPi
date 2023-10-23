from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{'tf_publish_rate': 10.0}]
        ),
        Node(
            package='controller',
            executable='controller_mc',
            respawn='true',
        )
    ])
