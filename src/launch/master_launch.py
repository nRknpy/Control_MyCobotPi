from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
        ),
        Node(
            package='sensor',
            executable='sensor_listener',
        ),
        Node(
            package='controller',
            executable='pybullet_sim',
        )
    ])
