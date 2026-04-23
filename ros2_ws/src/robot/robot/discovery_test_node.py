import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DiscoveryTestNode(Node):
    def __init__(self) -> None:
        super().__init__("discovery_test")
        self._sub = self.create_subscription(
            String,
            "test_string",
            self._callback,
            10,
        )
        self.get_logger().info(
            "discovery_test node ready — listening on /test_string"
        )

    def _callback(self, msg: String) -> None:
        self.get_logger().info(f"received: '{msg.data}'")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiscoveryTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
