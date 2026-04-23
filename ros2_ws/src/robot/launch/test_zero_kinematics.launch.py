"""
test_zero_kinematics.launch.py
==============================
Starts the zero-kinematics test node.

Prerequisite — the bridge node must already be running before launching this:
    ros2 run bridge bridge

Usage:
    ros2 launch robot test_zero_kinematics.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot",
                executable="test_zero_kinematics",
                name="zero_kinematics_test",
                output="screen",
            ),
        ]
    )
