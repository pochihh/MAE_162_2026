"""
test_position_fusion.launch.py
================================
Starts the position-fusion test node.

Prerequisite — the bridge node must already be running before launching this:
    ros2 run bridge bridge

The robot will drive in a straight line for DRIVE_DISTANCE_MM, then stop and
save a position/trajectory plot to ~/position_fusion_test_result.png.

Edit GPS_OFFSET_X_MM and GPS_OFFSET_Y_MM in
robot/test_position_fusion.py (the module at ros2_ws/src/robot/robot/) before
running once the arena offset has been measured.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot",
                executable="test_position_fusion",
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
