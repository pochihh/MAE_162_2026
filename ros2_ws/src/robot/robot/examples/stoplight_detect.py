"""
stoplight_detect.py — green traffic-light detection with live camera feed
=========================================================================
Uses the vision node's built-in traffic-light detector via the Robot API:
  1. get_detections("traffic light") — finds the stoplight and its bbox
  2. Checks the "color" attribute for "green"

Annotated stream served at http://<pi-ip>:8083

Requires vision_node running (provides /camera/image_raw + /vision/detections):
    ros2 run vision vision_node

Run via ROS2 entry point:
    ros2 run robot stoplight_detect
"""

from __future__ import annotations

import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from sensor_msgs.msg import Image

from robot.robot import Robot

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

_CAM_WIDTH  = 640
_CAM_HEIGHT = 480
_CAM_FPS    = 15

# ---------------------------------------------------------------------------
# Vision
# ---------------------------------------------------------------------------

MIN_CONFIDENCE   = 0.30   # stoplight scores 35-60%; keep this below that floor
VISION_STALE_SEC = 3.0

# ---------------------------------------------------------------------------
# Stream
# ---------------------------------------------------------------------------

STREAM_PORT = 8083

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

_frame_lock   = threading.Lock()
_latest_frame: np.ndarray | None = None

_jpeg_lock = threading.Lock()

def _placeholder_jpeg() -> bytes:
    img = np.zeros((_CAM_HEIGHT, _CAM_WIDTH, 3), dtype=np.uint8)
    cv2.putText(img, "waiting for camera...", (20, _CAM_HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2, cv2.LINE_AA)
    _, buf = cv2.imencode(".jpg", img)
    return buf.tobytes()

_latest_jpeg: bytes = _placeholder_jpeg()

# ---------------------------------------------------------------------------
# ROS image subscriber callback
# ---------------------------------------------------------------------------

_got_first_frame = False

def _on_image_msg(msg: Image) -> None:
    global _latest_frame, _got_first_frame
    frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        msg.height, msg.width, 3
    ).copy()
    with _frame_lock:
        _latest_frame = frame
    if not _got_first_frame:
        _got_first_frame = True
        print(f"[cam] first frame received  {msg.width}x{msg.height}")


# ---------------------------------------------------------------------------
# Detection helpers
# ---------------------------------------------------------------------------

def _best_traffic_light(robot: Robot) -> dict | None:
    """Return the highest-confidence traffic light detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best      = None
    best_conf = -1.0
    for det in robot.get_detections("traffic light"):
        conf = float(det["confidence"])
        if conf >= MIN_CONFIDENCE and conf > best_conf:
            best      = det
            best_conf = conf
    return best


def _scale_bbox(bbox: dict, vis_w: int, vis_h: int) -> tuple[int, int, int, int]:
    sx = _CAM_WIDTH  / max(vis_w, 1)
    sy = _CAM_HEIGHT / max(vis_h, 1)
    return (
        int(bbox["x"]      * sx),
        int(bbox["y"]      * sy),
        int(bbox["width"]  * sx),
        int(bbox["height"] * sy),
    )


# ---------------------------------------------------------------------------
# Annotation
# ---------------------------------------------------------------------------

def _annotate(
    frame: np.ndarray,
    robot: Robot,
    detection: dict | None,
    scaled_bbox: tuple[int, int, int, int] | None,
    is_green: bool,
) -> np.ndarray:
    out = frame.copy()

    # ── Main status ───────────────────────────────────────────────────────────
    if detection is None:
        vision_ok = robot.is_vision_active(timeout_s=VISION_STALE_SEC)
        msg = "vision active — no traffic light detected" if vision_ok \
              else "waiting for vision_node..."
        cv2.putText(out, msg, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 180, 180), 1, cv2.LINE_AA)
        return out

    x, y, w, h = scaled_bbox
    color_val = detection.get("attributes", {}).get("color", {}).get("value", "?")
    conf      = float(detection["confidence"])
    box_color = (0, 255, 0) if is_green else (0, 220, 255)

    cv2.rectangle(out, (x, y), (x + w, y + h), box_color, 2)
    cv2.putText(out, f"traffic light: {color_val} ({conf:.0%})",
                (x, max(y - 6, 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1, cv2.LINE_AA)

    if is_green:
        banner = "GREEN LIGHT RECOGNIZED"
        (tw, th), _ = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        bx = (_CAM_WIDTH  - tw) // 2
        by = _CAM_HEIGHT  - 20
        cv2.rectangle(out, (bx - 8, by - th - 8), (bx + tw + 8, by + 8),
                      (0, 0, 0), -1)
        cv2.putText(out, banner, (bx, by),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(out, f"stoplight found ({color_val}) — waiting for green", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, box_color, 1, cv2.LINE_AA)

    return out


# ---------------------------------------------------------------------------
# MJPEG HTTP server
# ---------------------------------------------------------------------------

class _StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == "/":
            html = (
                b"<html><body style='background:#000;margin:0'>"
                b"<img src='/stream' style='width:100%'>"
                b"</body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _jpeg_lock:
                        jpeg = _latest_jpeg
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                    time.sleep(1.0 / _CAM_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    global _latest_jpeg

    robot.enable_vision()

    server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), _StreamHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    ip = socket.gethostbyname(socket.gethostname())
    print(f"[stoplight] stream → http://{ip}:{STREAM_PORT}")
    print("[stoplight] searching for traffic light  Ctrl+C to stop")

    green_active = False

    try:
        while True:
            with _frame_lock:
                frame = _latest_frame

            if frame is None:
                time.sleep(0.05)
                continue

            detection   = _best_traffic_light(robot)
            is_green    = False
            scaled_bbox = None

            if detection:
                color = detection.get("attributes", {}).get("color", {}).get("value")
                is_green = (color == "green")
                vis_w, vis_h = robot.get_detection_image_size()
                scaled_bbox  = _scale_bbox(detection["bbox"], vis_w, vis_h)

                if is_green and not green_active:
                    print("[stoplight] GREEN LIGHT RECOGNIZED")
                green_active = is_green

            ann = _annotate(frame, robot, detection, scaled_bbox, is_green)
            _, buf = cv2.imencode(".jpg", ann, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with _jpeg_lock:
                _latest_jpeg = buf.tobytes()

            time.sleep(1.0 / _CAM_FPS)

    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        print("[stoplight] stopped")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    class _Node(Node):
        def __init__(self) -> None:
            super().__init__("stoplight_detect")
            self.robot = Robot(self)
            self.create_subscription(Image, "/camera/image_raw", _on_image_msg, 1)

    node = _Node()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    def _sighandler(_sig, _frame):
        raise KeyboardInterrupt()

    old_int  = signal.getsignal(signal.SIGINT)
    old_term = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT,  _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    try:
        run(node.robot)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT,  old_int)
        signal.signal(signal.SIGTERM, old_term)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
