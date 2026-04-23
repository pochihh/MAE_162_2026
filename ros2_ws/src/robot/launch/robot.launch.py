from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot",
                executable="robot",
                name="robot",
                output="screen",
            ),
            Node(
                package="sensors",
                executable="robot_gps",
                name="robot_gps",
                output="screen",
            ),
        ]
    )
