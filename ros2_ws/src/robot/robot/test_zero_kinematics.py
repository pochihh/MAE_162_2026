"""
test_zero_kinematics.py — drive → reset → drive → reset, then graph odometry
=============================================================================
Drives the robot forward for a fixed duration, resets odometry, drives again,
resets again, and plots x/y trajectory and heading over time for both runs.

Usage:
    ros2 run robot test_zero_kinematics

Prerequisite:
    ros2 run bridge bridge
"""

from __future__ import annotations

import signal
import threading
import time

import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
DRIVE_SPEED_MM_S = 100.0   # forward speed (mm/s — Robot default unit is mm)
DRIVE_DURATION_S = 3.0     # how long to drive each leg
POLL_TIMEOUT_S   = 0.2     # max wait for each /sensor_kinematics update
# ---------------------------------------------------------------------------


def collect_leg(robot: Robot, label: str, duration: float) -> list[tuple[float, float, float, float]]:
    """Drive forward for *duration* seconds, collecting (t, x, y, theta_deg) samples."""
    print(f"[zero_kinematics] Starting leg: {label}")
    robot.set_velocity(DRIVE_SPEED_MM_S, 0.0)

    samples: list[tuple[float, float, float, float]] = []
    t_start = time.monotonic()

    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= duration:
            break
        if robot.wait_for_pose_update(timeout=POLL_TIMEOUT_S):
            x, y, theta = robot.get_pose()
            samples.append((elapsed, x, y, theta))

    robot.stop()
    print(f"[zero_kinematics] Leg '{label}' done — {len(samples)} samples collected")
    return samples


def run(robot: Robot) -> None:
    # Wait for the first pose update so we know the bridge is publishing and
    # DDS discovery has settled before we try to call any services.
    print("[zero_kinematics] Waiting for bridge…")
    if not robot.wait_for_pose_update(timeout=10.0):
        raise RuntimeError("[zero_kinematics] Timed out waiting for /sensor_kinematics — is the bridge running?")

    if robot.get_state() != FirmwareState.RUNNING:
        print("[zero_kinematics] Setting firmware to RUNNING…")
        if not robot.set_state(FirmwareState.RUNNING, timeout=10.0):
            raise RuntimeError("[zero_kinematics] Failed to set firmware to RUNNING state.")
    else:
        print("[zero_kinematics] Firmware already RUNNING.")

    try:
        _run_test(robot)
    finally:
        robot.stop()
        robot.set_state(FirmwareState.IDLE)
        print("[zero_kinematics] Firmware returned to IDLE.")


def _run_test(robot: Robot) -> None:
    # ---- leg 1 ----
    samples_1 = collect_leg(robot, "leg 1", DRIVE_DURATION_S)

    print("[zero_kinematics] Resetting odometry after leg 1…")
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        raise RuntimeError("Timed out waiting for firmware to confirm first odometry reset.")
    x, y, theta = robot.get_pose()
    print(f"[zero_kinematics] Pose after reset 1: x={x:.2f}, y={y:.2f}, theta={theta:.2f} deg")

    # ---- leg 2 ----
    samples_2 = collect_leg(robot, "leg 2", DRIVE_DURATION_S)

    print("[zero_kinematics] Resetting odometry after leg 2…")
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        raise RuntimeError("Timed out waiting for firmware to confirm second odometry reset.")
    x, y, theta = robot.get_pose()
    print(f"[zero_kinematics] Pose after reset 2: x={x:.2f}, y={y:.2f}, theta={theta:.2f} deg")

    # ---- plot ----
    _plot(samples_1, samples_2)


def _plot(
    samples_1: list[tuple[float, float, float, float]],
    samples_2: list[tuple[float, float, float, float]],
) -> None:
    if not samples_1 and not samples_2:
        print("[zero_kinematics] No samples to plot.")
        return

    def unzip(samples):
        ts = [s[0] for s in samples]
        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        ths = [s[3] for s in samples]
        return ts, xs, ys, ths

    t1, x1, y1, th1 = unzip(samples_1)
    t2, x2, y2, th2 = unzip(samples_2)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("Odometry: drive → reset → drive → reset", fontweight="bold")

    # --- X vs time ---
    ax = axes[0]
    ax.plot(t1, x1, label="Leg 1", color="tab:blue")
    ax.plot(t2, x2, label="Leg 2", color="tab:orange")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("X (mm)")
    ax.set_title("X Position vs Time")
    ax.legend()
    ax.grid(True)

    # --- Y vs time ---
    ax = axes[1]
    ax.plot(t1, y1, label="Leg 1", color="tab:blue")
    ax.plot(t2, y2, label="Leg 2", color="tab:orange")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Y Position vs Time")
    ax.legend()
    ax.grid(True)

    # --- heading vs time ---
    ax = axes[2]
    ax.plot(t1, th1, label="Leg 1", color="tab:blue")
    ax.plot(t2, th2, label="Leg 2", color="tab:orange")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heading (deg)")
    ax.set_title("Heading vs Time")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    out_path = "/tmp/zero_kinematics_result.png"
    plt.savefig(out_path, dpi=150)
    print(f"[zero_kinematics] Plot saved to {out_path}")
    plt.show()
    print("[zero_kinematics] Plot displayed.")


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    class _ZeroKinematicsNode(Node):
        def __init__(self) -> None:
            super().__init__("zero_kinematics_test")
            self.robot = Robot(self)

    node = _ZeroKinematicsNode()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    def _raise_keyboard_interrupt(signum, frame):
        raise KeyboardInterrupt()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        run(node.robot)
    except KeyboardInterrupt:
        node.get_logger().info("test interrupted; shutting down")
    finally:
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
