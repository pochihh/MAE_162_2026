"""
Robot GPS node.

Receives global ArUco localizer detections from the Jetson Nano over a TCP
connection and re-publishes them on /tag_detections in the local ROS2 domain.

Why TCP instead of ROS2 DDS:
    The lab WiFi access point performs NAT for WiFi clients (robots).  The
    Jetson (wired Ethernet) cannot initiate connections back to a WiFi client.
    By having the robot *connect to* the Jetson's TCP server (port 7777),
    the NAT table is established by the outbound connection and the Jetson
    can push data back over the same socket.  No network-admin changes needed.

Parameters
----------
jetson_ip  : str  (default "192.168.8.120")
    IP address of the Jetson running the global_gps stack.
jetson_port : int  (default 7777)
    TCP port of the Jetson's detection push server.

Topic published  (visible locally on this machine):
    /tag_detections  (bridge_interfaces/msg/TagDetectionArray)
"""

from __future__ import annotations

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from bridge_interfaces.msg import TagDetection, TagDetectionArray


_RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

_RECONNECT_DELAY = 3.0  # seconds between reconnect attempts


class RobotGpsNode(Node):
    """Receive global GPS detections from Jetson via TCP and republish locally."""

    def __init__(self) -> None:
        super().__init__("robot_gps")

        self.declare_parameter("jetson_ip", "192.168.8.120")
        self.declare_parameter("jetson_port", 7777)
        self.declare_parameter("tracked_tag_id", -1)

        self._jetson_ip: str = self.get_parameter("jetson_ip").value
        self._jetson_port: int = int(self.get_parameter("jetson_port").value)
        self._tracked_tag_id: int = int(self.get_parameter("tracked_tag_id").value)

        self._pub = self.create_publisher(
            TagDetectionArray,
            "/tag_detections",
            _RELIABLE_QOS,
        )

        tag_desc = str(self._tracked_tag_id) if self._tracked_tag_id != -1 else "any"
        self.get_logger().info(
            f"Robot GPS node started. "
            f"Connecting to Jetson at {self._jetson_ip}:{self._jetson_port} "
            f"(tracked_tag_id={tag_desc})"
        )

        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True
        )
        self._recv_thread.start()

    # ── TCP receive loop ──────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        """Connect to Jetson TCP server and receive line-delimited JSON."""
        while rclpy.ok():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._jetson_ip, self._jetson_port))
                self.get_logger().info(
                    f"Connected to Jetson at "
                    f"{self._jetson_ip}:{self._jetson_port}"
                )
                buf = b""
                while rclpy.ok():
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._handle_line(line.decode("utf-8", errors="ignore"))
            except (OSError, ConnectionRefusedError) as exc:
                self.get_logger().warn(
                    f"Jetson connection lost ({exc}). "
                    f"Retrying in {_RECONNECT_DELAY:.0f}s…"
                )
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
            time.sleep(_RECONNECT_DELAY)

    def _handle_line(self, line: str) -> None:
        """Parse one JSON detection line and publish to /tag_detections."""
        line = line.strip()
        if not line:
            return
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Bad JSON from Jetson: {line[:80]}")
            return

        stamp_sec = float(data.get("stamp", 0.0))
        sec = int(stamp_sec)
        nanosec = int((stamp_sec - sec) * 1e9)

        msg = TagDetectionArray()
        msg.header = Header()
        msg.header.stamp = Time(sec=sec, nanosec=nanosec)
        msg.header.frame_id = "world"

        for det in data.get("detections", []):
            tid = int(det["tag_id"])
            if self._tracked_tag_id != -1 and tid != self._tracked_tag_id:
                continue
            d = TagDetection()
            d.tag_id = tid
            d.x = float(det["x"])
            d.y = float(det["y"])
            d.theta = float(det["theta"])
            msg.detections.append(d)

        self._pub.publish(msg)
        self.get_logger().debug(
            f"Forwarded {len(msg.detections)} detection(s): "
            f"{[d.tag_id for d in msg.detections]}"
        )


def main() -> None:
    rclpy.init()
    node = RobotGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
