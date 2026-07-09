from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    topic_name = LaunchConfiguration("topic_name")
    scan_mode = LaunchConfiguration("scan_mode")
    scan_frequency = LaunchConfiguration("scan_frequency")
    angle_compensate = LaunchConfiguration("angle_compensate")
    inverted = LaunchConfiguration("inverted")
    flip_x_axis = LaunchConfiguration("flip_x_axis")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/rplidar",
            description="Stable serial device path for the RPLidar C1.",
        ),
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="460800",
            description="RPLidar C1 UART baud rate.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="laser",
            description="LaserScan frame id.",
        ),
        DeclareLaunchArgument(
            "topic_name",
            default_value="scan",
            description="LaserScan topic name.",
        ),
        DeclareLaunchArgument(
            "scan_mode",
            default_value="Standard",
            description="RPLidar C1 scan mode.",
        ),
        DeclareLaunchArgument(
            "scan_frequency",
            default_value="10.0",
            description="Requested scan frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "angle_compensate",
            default_value="true",
            description="Publish an evenly binned 360 degree LaserScan.",
        ),
        DeclareLaunchArgument(
            "inverted",
            default_value="true",
            description="Invert scan direction if the lidar is mounted upside down.",
        ),
        DeclareLaunchArgument(
            "flip_x_axis",
            # The C1 driver's angle transform is: ros_angle = π − lidar_angle, which places
            # the lidar's physical front at ROS 180°. flip_x_axis adds another π, giving
            # ros_angle = −lidar_angle, so the lidar's front maps to ROS 0° (robot forward).
            # Keep true when the lidar's front faces the robot's front (standard upright mounting).
            default_value="true",
            description="Rotate published scan by 180 degrees (true for standard upright mounting).",
        ),
        Node(
            package="rplidar_ros",
            executable="rplidar_c1_node",
            name="rplidar_c1",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "serial_baudrate": ParameterValue(serial_baudrate, value_type=int),
                "frame_id": frame_id,
                "topic_name": topic_name,
                "scan_mode": scan_mode,
                "scan_frequency": ParameterValue(scan_frequency, value_type=float),
                "angle_compensate": ParameterValue(angle_compensate, value_type=bool),
                "inverted": ParameterValue(inverted, value_type=bool),
                "flip_x_axis": ParameterValue(flip_x_axis, value_type=bool),
            }],
        ),
    ])
