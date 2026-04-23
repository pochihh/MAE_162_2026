#!/usr/bin/env python3
"""
capture_photo.py — Save a single colour frame from the GPS camera.

Connects to the Intel RealSense D4xx camera directly via pyrealsense2
(libusb/V4L2 backend — no ROS2 stack required).

If the required V4L2 device nodes (/dev/video0-5) are missing inside the
Docker container (they can disappear after a USB re-enumeration), this script
creates them automatically using mknod (requires privileged container).

Usage (inside the Docker container):
    python3 /ros2_ws/src/global_gps/capture_photo.py
    python3 /ros2_ws/src/global_gps/capture_photo.py --output /tmp/field.jpg

Or from the host:
    docker exec docker-global_gps-1 \
        python3 /ros2_ws/src/global_gps/capture_photo.py
"""

from __future__ import annotations

import argparse
import datetime
import os
import subprocess
import sys

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit(
        "ERROR: pyrealsense2 not found.\n"
        "Install it with: pip3 install --break-system-packages pyrealsense2"
    )

try:
    import numpy as np
    import cv2
except ImportError as e:
    sys.exit(f"ERROR: {e}")


_VIDEO_MAJOR = 81  # Linux V4L2 major device number


def _ensure_video_nodes() -> None:
    """Create /dev/video0-5 inside the container if they don't exist.

    The privileged Docker container starts with device nodes that were present
    at image build/start time.  If the RealSense camera disconnects and
    reconnects it may get assigned new minor numbers on the host; this call
    re-creates the canonical ones (minor 0-5) that librealsense expects.
    """
    needed = [n for n in range(6) if not os.path.exists(f"/dev/video{n}")]
    if not needed:
        return
    for n in needed:
        try:
            subprocess.run(
                ["mknod", "-m", "666", f"/dev/video{n}", "c",
                 str(_VIDEO_MAJOR), str(n)],
                check=True, capture_output=True,
            )
        except subprocess.CalledProcessError as e:
            # Non-fatal: librealsense may still work via libusb.
            print(f"[warn] mknod /dev/video{n} failed: {e.stderr.decode().strip()}",
                  file=sys.stderr)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Snapshot the GPS camera's current field of view."
    )
    p.add_argument(
        "--output", "-o",
        default="",
        help="Output file path (default: /tmp/gps_snapshot_<timestamp>.jpg)",
    )
    p.add_argument(
        "--warmup", type=int, default=15,
        help="Frames to discard before saving (auto-exposure settle, default: 15)",
    )
    p.add_argument(
        "--width",  type=int, default=640,  help="Stream width  (default: 640)")
    p.add_argument(
        "--height", type=int, default=480,  help="Stream height (default: 480)")
    p.add_argument(
        "--fps",    type=int, default=30,   help="Stream FPS    (default: 30)")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    output_path = args.output or (
        f"/tmp/gps_snapshot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
    )

    # Ensure device nodes exist (auto-repairs after USB re-enumeration).
    _ensure_video_nodes()

    # ── Connect to camera ─────────────────────────────────────────────────
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        sys.exit(
            "ERROR: No RealSense device found.\n"
            "Check that:\n"
            "  • The camera USB cable is connected\n"
            "  • No other process is using the camera\n"
            "  • The container has privileged access (/dev/bus/usb mounted)"
        )

    dev = devices[0]
    print(
        f"Camera: {dev.get_info(rs.camera_info.name)}"
        f" | S/N: {dev.get_info(rs.camera_info.serial_number)}"
        f" | FW: {dev.get_info(rs.camera_info.firmware_version)}"
    )

    # ── Configure pipeline ────────────────────────────────────────────────
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(
        rs.stream.color,
        args.width, args.height,
        rs.format.bgr8,
        args.fps,
    )

    print(f"Starting stream ({args.width}×{args.height} @ {args.fps} fps)…")
    try:
        pipeline.start(config)
    except RuntimeError as e:
        sys.exit(f"ERROR: Could not start pipeline: {e}")

    try:
        for i in range(args.warmup):
            pipeline.wait_for_frames()
            print(f"\r  Warming up … {i+1}/{args.warmup}", end="", flush=True)
        print()

        frames       = pipeline.wait_for_frames()
        color_frame  = frames.get_color_frame()

        if not color_frame:
            sys.exit("ERROR: No colour frame received from camera.")

        image = np.asanyarray(color_frame.get_data())  # BGR uint8

    finally:
        pipeline.stop()

    # ── Save ─────────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    if not cv2.imwrite(output_path, image):
        sys.exit(f"ERROR: cv2.imwrite failed for: {output_path}")

    h, w = image.shape[:2]
    print(f"Saved {w}×{h} image → {output_path}")


if __name__ == "__main__":
    main()
