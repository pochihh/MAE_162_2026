"""
Launch file for the Global GPS stack on the Jetson Nano.

Starts:
  1. realsense2_camera  — RealSense D4xx driver (colour + aligned depth)
  2. ground_localizer   — ArUco field localizer, publishes /global_gps/tag_detections

Usage (inside the Jetson Docker container):
    ros2 launch global_gps global_gps.launch.py
    ros2 launch global_gps global_gps.launch.py marker_size:=0.15 corner_ids:=[0,1,2,3]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # ── Launch arguments ──────────────────────────────────────────────────
    marker_size_arg = DeclareLaunchArgument(
        "marker_size",
        default_value="0.10",
        description="Physical side length of ArUco markers in metres.",
    )
    corner_ids_arg = DeclareLaunchArgument(
        "corner_ids",
        default_value="[0, 1, 2, 3]",
        description="Marker IDs used as fixed field-corner anchors.",
    )
    rover_ids_arg = DeclareLaunchArgument(
        "rover_ids",
        default_value="[11, 12, 13, 14, 15, 16, 17, 18]",
        description="Marker IDs that appear on rovers.",
    )
    tcp_port_arg = DeclareLaunchArgument(
        "tcp_port",
        default_value="7777",
        description="TCP port for the robot push server (NAT-friendly delivery).",
    )

    # ── RealSense camera driver ───────────────────────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("realsense2_camera"),
            "/launch/rs_launch.py",
        ]),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            # Align depth to colour frame so pixel coordinates match.
            "align_depth.enable": "true",
            # Publish at camera native rate (typically 30 Hz).
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
        }.items(),
    )

    # ── Ground localizer node ─────────────────────────────────────────────
    localizer_node = Node(
        package="global_gps",
        executable="ground_localizer",
        name="ground_localizer",
        output="screen",
        parameters=[{
            "marker_size": LaunchConfiguration("marker_size"),
            "corner_ids": LaunchConfiguration("corner_ids"),
            "rover_ids": LaunchConfiguration("rover_ids"),
            "tcp_port": LaunchConfiguration("tcp_port"),
        }],
    )

    return LaunchDescription([
        marker_size_arg,
        corner_ids_arg,
        rover_ids_arg,
        tcp_port_arg,
        realsense_launch,
        localizer_node,
    ])
