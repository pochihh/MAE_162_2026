from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('scan_visualizer')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def listener_callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # Filter invalid values
        valid = np.isfinite(ranges)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        self.ax.clear()
        self.ax.scatter(x, y, s=5)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_title("LiDAR Scan")
        self.ax.set_aspect('equal')

        plt.grid()
        plt.draw()
        plt.pause(0.01)
        plt.savefig('/data/plot.png')


class SensorNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_node")
        self.get_logger().info("sensors package scaffold ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
