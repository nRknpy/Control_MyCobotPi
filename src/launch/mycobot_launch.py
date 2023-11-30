from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{'serial_no': '_138422073197'}],
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{'serial_no': '_128422271786'}],
        ),
        Node(
            package='controller',
            executable='controller_mc',
            respawn='true',
        )
    ])
