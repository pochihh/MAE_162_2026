"""
test_orientation_fusion.py — complementary-filter orientation test
==================================================================
Spins the robot in place through 180 degrees, recording odometry theta and the
complementary-filter (magnetometer-blended) heading at every tick.
After the run, matplotlib plots:
  1. Trajectory — x/y path from odometry, coloured by fused-heading drift
  2. Heading comparison — raw odometry vs fused heading over time
  3. Fusion correction — per-sample difference (fused − odometry)

Usage (replace the run() import in main.py, or use the dedicated launch file):
    ros2 launch robot test_orientation_fusion.launch.py

Tuning parameters at the top of this file:
    SPIN_ANGLE_DEG    — total in-place rotation angle (degrees)
    ANGULAR_DEG_S     — commanded angular speed (degrees/s)
    ORIENTATION_FUSION_ALPHA      — complementary-filter mag weight (0 = pure odometry)
"""

from __future__ import annotations

import math
import time

import matplotlib
matplotlib.use("Agg")   # headless-safe; switched to TkAgg below if a display exists
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

from robot.robot import FirmwareState, Robot
from robot.hardware_map import DEFAULT_FSM_HZ

# ── Tunable parameters ────────────────────────────────────────────────────────

SPIN_ANGLE_DEG = 180.0   # degrees — in-place sweep to record heading fusion
ANGULAR_DEG_S  = 20.0    # deg/s — CCW positive
ORIENTATION_FUSION_ALPHA   = 1       # complementary-filter magnetometer weight

# Derived motion duration for the in-place spin
_SPIN_DURATION_S = abs(SPIN_ANGLE_DEG / ANGULAR_DEG_S)

# ── Plot output path ──────────────────────────────────────────────────────────
# /host_home is the host $HOME bind-mounted in docker-compose.rpi.yml.
# Fall back to the container's own home if the mount is absent (e.g. vm target).
import os
_HOST_HOME = "/host_home"
_PLOT_PATH = (
    os.path.join(_HOST_HOME, "orientation_fusion_test_result.png")
    if os.path.isdir(_HOST_HOME)
    else os.path.expanduser("~/orientation_fusion_test_result.png")
)


# =============================================================================
# Data collection helpers
# =============================================================================

class _Record:
    """Lightweight ring-buffer replacement — just appends to lists."""
    def __init__(self) -> None:
        self.t:           list[float] = []
        self.x:           list[float] = []
        self.y:           list[float] = []
        self.odom_deg:    list[float] = []
        self.fused_deg:   list[float] = []

    def append(self, t, x, y, odom_deg, fused_deg) -> None:
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.odom_deg.append(odom_deg)
        self.fused_deg.append(fused_deg)

    # Convenience: arrays for numpy/matplotlib
    def arrays(self):
        return (
            np.array(self.t),
            np.array(self.x),
            np.array(self.y),
            np.array(self.odom_deg),
            np.array(self.fused_deg),
        )


# =============================================================================
# In-place spin
# =============================================================================

def _spin_in_place(robot: Robot, rec: _Record) -> None:
    """Command the robot to spin in place and record sensor data."""
    total_duration = _SPIN_DURATION_S
    period = 1.0 / float(DEFAULT_FSM_HZ)

    # ── Prerequisite: bridge must be running ─────────────────────────────────
    _SERVICE_WAIT_S = 10.0
    if not robot._set_state_client.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
        raise RuntimeError(
            f"[orientation_fusion_test] /set_firmware_state service not available after "
            f"{_SERVICE_WAIT_S:.0f}s. Start the bridge first:\n"
            "  ros2 run bridge bridge"
        )

    # ── Wait for firmware to leave INIT (state 0) ────────────────────────────
    _IDLE_WAIT_S = 10.0
    _t0 = time.monotonic()
    while robot.get_state() == 0:   # 0 = INIT, not in FirmwareState enum
        if time.monotonic() - _t0 > _IDLE_WAIT_S:
            raise RuntimeError(
                f"[orientation_fusion_test] Firmware still in INIT after {_IDLE_WAIT_S:.0f}s."
            )
        time.sleep(0.1)

    # ── If ERROR or ESTOP, reset to IDLE first ───────────────────────────────
    if robot.get_state() not in (int(FirmwareState.IDLE), int(FirmwareState.RUNNING)):
        if not robot.set_state(FirmwareState.IDLE):
            raise RuntimeError(
                f"[orientation_fusion_test] Could not reset firmware to IDLE "
                f"(current state: {robot.get_state()})."
            )

    if not robot.set_state(FirmwareState.RUNNING):
        # The bridge service may time out before it observes the state echo
        # (the ROS single-threaded executor blocks its own timer while the
        # service callback is waiting).  The START command was still sent;
        # give the bridge timer a moment to flush the decoded queue.
        _deadline = time.monotonic() + 1.0
        while robot.get_state() != int(FirmwareState.RUNNING):
            if time.monotonic() > _deadline:
                raise RuntimeError(
                    f"[orientation_fusion_test] Firmware did not reach RUNNING "
                    f"(state: {robot.get_state()})."
                )
            time.sleep(0.05)
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        raise RuntimeError(
            "[orientation_fusion_test] Timed out waiting for firmware to confirm odometry reset."
        )

    # Capture initial values — both should be ~0 after reset, but subtract
    # as a safety net against any residual UART timing jitter.
    fused_deg_0 = robot.get_fused_orientation()
    with robot._lock:
        odom_deg_0 = math.degrees(robot._pose[2])

    t_start = time.monotonic()
    next_tick = t_start

    print(
        f"[orientation_fusion_test] Spinning in place {SPIN_ANGLE_DEG:.0f}°: "
        f"v=0.0 mm/s, ω={ANGULAR_DEG_S:.1f}°/s, α={ORIENTATION_FUSION_ALPHA}\n"
        f"[orientation_fusion_test] Expected duration: {total_duration:.1f} s"
    )

    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= total_duration:
            break

        x, y, _ = robot.get_pose()
        with robot._lock:
            odom_deg = math.degrees(robot._pose[2]) - odom_deg_0
        fused_deg = robot.get_fused_orientation() - fused_deg_0
        rec.append(elapsed, x, y, odom_deg, fused_deg)

        robot.set_velocity(0.0, ANGULAR_DEG_S)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

    robot.stop()
    print("[orientation_fusion_test] In-place spin complete — stopping.")


# =============================================================================
# Plotting
# =============================================================================

def _plot_results(rec: _Record) -> None:
    t, x, y, odom_deg, fused_deg = rec.arrays()

    # Unwrap both heading traces before differencing so the correction is
    # continuous even when either trace crosses the ±180° boundary.
    correction = np.degrees(np.unwrap(np.radians(fused_deg))) \
               - np.degrees(np.unwrap(np.radians(odom_deg)))

    try:
        import matplotlib
        matplotlib.use("TkAgg")
    except Exception:
        pass  # stay with Agg if no display

    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    fig.suptitle(
        f"Complementary-filter fusion test  "
        f"(α={ORIENTATION_FUSION_ALPHA}, {SPIN_ANGLE_DEG:.0f}° in-place spin)",
        fontsize=12,
    )

    # ── Panel 1: Heading over time ────────────────────────────────────────────
    ax = axes[0]
    ax.plot(t, odom_deg,  lw=1.2, label="Odometry θ", color="steelblue")
    ax.plot(t, fused_deg, lw=1.5, label="Fused θ (CF)",  color="tomato",  linestyle="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("heading (°)")
    ax.set_title("Heading: odometry vs fused")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── Panel 2: Fusion correction over time ──────────────────────────────────
    ax = axes[1]
    ax.axhline(0, color="gray", lw=0.8, linestyle=":")
    ax.fill_between(t, correction, alpha=0.3, color="tomato")
    ax.plot(t, correction, lw=1.2, color="tomato", label="fused − odom")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("correction (°)")
    ax.set_title("Fusion correction (fused − odometry)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    try:
        plt.savefig(_PLOT_PATH, dpi=150)
        print(f"[orientation_fusion_test] Plot saved → {_PLOT_PATH}")
    except Exception as exc:
        print(f"[orientation_fusion_test] Could not save plot: {exc}")

    try:
        plt.show()
    except Exception:
        pass  # headless — plot already saved


# =============================================================================
# Entry point
# =============================================================================

def run(robot: Robot) -> None:
    robot.set_orientation_fusion_alpha(ORIENTATION_FUSION_ALPHA)

    rec = _Record()
    _spin_in_place(robot, rec)
    _plot_results(rec)


# =============================================================================
# ROS2 node entry point (mirrors robot_node.py, but calls this module's run())
# =============================================================================

def main(args=None) -> None:
    import signal
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    # Reuse the Robot wrapper via a minimal node
    from robot.robot import Robot as _Robot

    class _TestNode(Node):
        def __init__(self) -> None:
            super().__init__("orientation_fusion_test")
            self.robot = _Robot(self)

    node = _TestNode()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    def _raise_keyboard_interrupt(signum, frame):
        raise KeyboardInterrupt()

    old_sigint  = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT,  _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        run(node.robot)
    except KeyboardInterrupt:
        node.get_logger().info("test interrupted; shutting down")
    finally:
        try:
            node.robot.shutdown()
        except Exception:
            pass
        signal.signal(signal.SIGINT,  old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
