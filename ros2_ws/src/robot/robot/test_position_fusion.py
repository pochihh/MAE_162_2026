"""
test_position_fusion.py — GPS-anchored position fusion alpha-tuning test
=========================================================================
Purpose
-------
Helps students tune ``POSITION_FUSION_ALPHA`` by driving the robot in a straight
line along the robot's initial forward direction between two marked spots on
the floor.

The test does not assume the robot starts at the GPS/world origin. Instead,
when GPS is first acquired it records the current odometry position and uses
that as the translation offset for the GPS frame. After that, the fused
position behaves in the robot's odometry/world frame and students can tune
``POSITION_FUSION_ALPHA`` directly against the marked travel distance.

Setup
-----
1. Mark two spots on the floor separated by ``DRIVE_DISTANCE_MM`` along the
   +Y axis.  Place the robot on the first spot facing forward (+Y).
2. Mark the destination spot with an X.
3. Run the test.  The robot drives to the second spot and stops.
4. Check the plots:
   - **Forward progress over time** — how closely the fused estimate tracks the
     target distance. If fused progress drifts away from the target line the
     alpha is too low (GPS not pulling enough) or GPS coverage has a gap.
   - **Lateral drift over time** — ideally stays near zero.  A large lateral
     drift means the robot veered; fused drift should correct back toward zero
     when GPS is active.
   - **2D bird's-eye trajectory** — the ideal path is a straight line from the
     start marker to the end marker. The fused path should stay tighter to the
     ideal than raw odometry when GPS is active.
   - **Fused − odometry error** — shaded green when GPS is active; error
     magnitude should rise when GPS drops out and shrink when it returns.

Tuning guide
------------
* If the fused path overshoots or oscillates around the corrected position → lower alpha.
* If the fused path lags odometry and never fully corrects to the marker → raise alpha.
* A good starting point is alpha = 0.3–0.5 for 10 Hz GPS; alpha = 1.0 snaps
  directly to GPS each kinematics tick (useful to verify GPS accuracy in isolation).

Usage:
    ros2 launch robot test_position_fusion.launch.py
"""

from __future__ import annotations

import math
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from robot.robot import FirmwareState, Robot
from robot.hardware_map import DEFAULT_FSM_HZ

# ── Tunable parameters ────────────────────────────────────────────────────────

DRIVE_DISTANCE_MM     = 1000.0   # mm — distance to drive once inside GPS range
DRIVE_SPEED_MM_S      = 100.0   # mm/s forward speed
POSITION_FUSION_ALPHA      = 1     # GPS weight for complementary filter (0–1); tune this
GPS_SEARCH_EXTRA_MM   = 2000.0  # mm — extra distance to drive past DRIVE_DISTANCE_MM
                                 #       searching for GPS if not yet acquired; set to 0
                                 #       to disable the extension and stop at DRIVE_DISTANCE_MM
                                 #       regardless of GPS status.

# ── Plot output path ──────────────────────────────────────────────────────────
# /host_home is the host $HOME bind-mounted in docker-compose.rpi.yml.
# Fall back to the container's own home if the mount is absent (e.g. vm target).
_HOST_HOME = "/host_home"
_PLOT_PATH = (
    os.path.join(_HOST_HOME, "position_fusion_test_result.png")
    if os.path.isdir(_HOST_HOME)
    else os.path.expanduser("~/position_fusion_test_result.png")
)


# =============================================================================
# Data collection
# =============================================================================

class _Record:
    def __init__(self) -> None:
        self.t:          list[float] = []
        self.odom_x:     list[float] = []
        self.odom_y:     list[float] = []
        self.fused_x:    list[float] = []
        self.fused_y:    list[float] = []
        self.gps_active: list[bool]  = []

    def append(self, t, odom_x, odom_y, fused_x, fused_y, gps_active) -> None:
        self.t.append(t)
        self.odom_x.append(odom_x)
        self.odom_y.append(odom_y)
        self.fused_x.append(fused_x)
        self.fused_y.append(fused_y)
        self.gps_active.append(gps_active)

    def arrays(self):
        return (
            np.array(self.t),
            np.array(self.odom_x),
            np.array(self.odom_y),
            np.array(self.fused_x),
            np.array(self.fused_y),
            np.array(self.gps_active, dtype=bool),
        )


# =============================================================================
# Straight-line drive
# =============================================================================

def _drive_straight(robot: Robot, rec: _Record) -> None:
    """Drive forward, recording every tick.

    Stopping logic
    --------------
    * Normal case (GPS acquired before or during the drive): stop once odometry
      reports that the robot has travelled DRIVE_DISTANCE_MM *since GPS was first
      acquired*.  This guarantees at least DRIVE_DISTANCE_MM of fused-vs-odometry
      comparison data in the plot.
    * Robot starts outside GPS range: if GPS has not been acquired by the time
      the robot has driven DRIVE_DISTANCE_MM from the start, keep driving up to
      GPS_SEARCH_EXTRA_MM more millimetres while looking for GPS coverage.  As
      soon as GPS is found the DRIVE_DISTANCE_MM countdown restarts from that
      point.
    * Safety cap: stop unconditionally at DRIVE_DISTANCE_MM + GPS_SEARCH_EXTRA_MM
      from the start with a clear warning when GPS was never found.
    """
    period = 1.0 / float(DEFAULT_FSM_HZ)

    # ── Prerequisite: bridge must be running ─────────────────────────────────
    _SERVICE_WAIT_S = 10.0
    if not robot._set_state_client.wait_for_service(timeout_sec=_SERVICE_WAIT_S):
        raise RuntimeError(
            f"[position_test] /set_firmware_state service not available after "
            f"{_SERVICE_WAIT_S:.0f}s. Start the bridge first:\n"
            "  ros2 run bridge bridge"
        )

    # ── Wait for firmware to leave INIT (state 0) ────────────────────────────
    _IDLE_WAIT_S = 10.0
    _t0 = time.monotonic()
    while robot.get_state() == 0:   # 0 = INIT, not in FirmwareState enum
        if time.monotonic() - _t0 > _IDLE_WAIT_S:
            raise RuntimeError(
                f"[position_test] Firmware still in INIT after {_IDLE_WAIT_S:.0f}s."
            )
        time.sleep(0.1)

    # ── If ERROR or ESTOP, reset to IDLE first ───────────────────────────────
    if robot.get_state() not in (int(FirmwareState.IDLE), int(FirmwareState.RUNNING)):
        if not robot.set_state(FirmwareState.IDLE):
            raise RuntimeError(
                f"[position_test] Could not reset firmware to IDLE "
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
                    f"[position_test] Firmware did not reach RUNNING "
                    f"(state: {robot.get_state()})."
                )
            time.sleep(0.05)
    robot.reset_odometry()

    # Poll until the odometry echo comes back near zero, confirming the reset
    # has fully round-tripped through UART.  Two fixed wait_for_pose_update
    # calls are not reliable when the previous run left the firmware at a large
    # odometry value — the reset packet and the stale pose echo can race.
    _RESET_SETTLE_MM  = 10.0   # accept once within 10 mm of origin
    _RESET_TIMEOUT_S  = 3.0
    _t_reset = time.monotonic()
    while True:
        robot.wait_for_pose_update(timeout=0.5)
        with robot._lock:
            _ox, _oy, _ = robot._pose
        if math.hypot(_ox, _oy) < _RESET_SETTLE_MM:
            break
        if time.monotonic() - _t_reset > _RESET_TIMEOUT_S:
            print(
                f"[pos_fusion_test] WARNING — odometry did not settle to zero after "
                f"{_RESET_TIMEOUT_S:.0f}s (last value: {_ox:.1f}, {_oy:.1f} mm). "
                f"Proceeding with current position as reference origin."
            )
            break

    with robot._lock:
        odom_x0, odom_y0, _ = robot._pose
    fused_x0, fused_y0, _ = robot._get_pose_mm()

    t_start = time.monotonic()
    next_tick = t_start
    odom_x = odom_x0
    odom_y = odom_y0
    gps_aligned = False

    # Odometry position at the moment GPS was first acquired.
    gps_acq_odom_x = odom_x0
    gps_acq_odom_y = odom_y0
    warned_no_gps = False
    max_dist = DRIVE_DISTANCE_MM + GPS_SEARCH_EXTRA_MM
    # X plot references — reset on the first recording tick after GPS aligns so
    # the X panel shows drift *since GPS activation*, not the GPS-frame
    # translation offset.  Y is already relative (robot drives in +Y from
    # the start, so odom_y0 / fused_y0 captured at start are the correct
    # references).  X stays near 0 throughout, so the GPS X-frame offset
    # would otherwise dominate the lateral-drift panel.
    plot_odom_x0  = odom_x0
    plot_fused_x0 = fused_x0
    gps_x_ref_set = False

    print(
        f"[pos_fusion_test] Driving in world-Y direction for {DRIVE_DISTANCE_MM:.0f} mm "
        f"at {DRIVE_SPEED_MM_S:.0f} mm/s  alpha={POSITION_FUSION_ALPHA}\n"
        f"[pos_fusion_test] GPS frame will be translated to match odometry "
        f"at first acquisition.\n"
        f"[pos_fusion_test] If GPS is not acquired within {DRIVE_DISTANCE_MM:.0f} mm, "
        f"the robot will continue up to {GPS_SEARCH_EXTRA_MM:.0f} mm further to find "
        f"GPS coverage (total cap: {max_dist:.0f} mm)."
        if GPS_SEARCH_EXTRA_MM > 0 else
        f"[pos_fusion_test] Driving in world-Y direction for {DRIVE_DISTANCE_MM:.0f} mm "
        f"at {DRIVE_SPEED_MM_S:.0f} mm/s  alpha={POSITION_FUSION_ALPHA}"
    )

    while True:
        robot.wait_for_pose_update(timeout=period * 2)

        # Raw odometry (private, accessed here for diagnostic purposes only)
        with robot._lock:
            odom_x, odom_y, _ = robot._pose
            gps_x_mm = robot._gps_x_mm
            gps_y_mm = robot._gps_y_mm

        gps_on = robot.is_gps_active() # for information purposes
        if gps_on and not gps_aligned:
        #    # Align the GPS frame translation to the odometry/world frame
        #    # the moment GPS first becomes available.
        #    learned_offset_x = odom_x - gps_x_mm
        #    learned_offset_y = odom_y - gps_y_mm
        #    robot.set_gps_offset(learned_offset_x, learned_offset_y)
            gps_aligned = True
            gps_acq_odom_x = odom_x
            gps_acq_odom_y = odom_y
        #    dist_at_acq = math.hypot(odom_x - odom_x0, odom_y - odom_y0)
        #    print(
        #        f"[pos_fusion_test] GPS acquired at {dist_at_acq:.0f} mm — learned offset "
        #        f"({learned_offset_x:.1f}, {learned_offset_y:.1f}) mm. "
        #        f"Driving {DRIVE_DISTANCE_MM:.0f} mm more inside GPS range."
        #    )
        #    # Wait for the next tag update so the new offset is reflected in
        #    # the internal GPS state before recording/using fused position.
        #    continue


        # algorithm:
        # 1. if entered GPS region, use the gps reading (with the gps offset already in place, no need to add another gps offset)
        # 2. start using the GPS x and y, and use the entrypoint gps location as the offset to add to the odometer

        fused_x_mm, fused_y_mm, _ = robot._get_pose_mm()

        # On the first recording tick after GPS aligns, re-zero the X references
        # so the lateral-drift panel shows drift relative to the GPS-activation
        # point, not the GPS-frame X translation.
        if gps_aligned and not gps_x_ref_set:
            plot_odom_x0  = odom_x
            plot_fused_x0 = fused_x_mm
            gps_x_ref_set = True
            print(
                f"[pos_fusion_test] GPS X reference set: "
                f"odom_x={odom_x:.1f} mm, fused_x={fused_x_mm:.1f} mm "
                f"(GPS X offset absorbed = {fused_x_mm - odom_x:.1f} mm)"
            )

        elapsed = time.monotonic() - t_start
        # Record X relative to the GPS-activation reference (removes GPS-frame
        # X translation); record Y relative to the test-start position (captures
        # the full forward progress from rest).
        rec.append(
            elapsed,
            odom_x - plot_odom_x0, odom_y - odom_y0,
            fused_x_mm - plot_fused_x0, fused_y_mm - fused_y0,
            gps_on,
        )

        dist_traveled = math.hypot(odom_x - odom_x0, odom_y - odom_y0)

        # Distance driven since GPS was first acquired (reset when GPS is found
        # mid-drive so that we always record DRIVE_DISTANCE_MM of comparison data).
        dist_since_gps = math.hypot(
            odom_x - gps_acq_odom_x, odom_y - gps_acq_odom_y
        )

        # ── Stopping conditions ───────────────────────────────────────────────
        if gps_aligned and dist_since_gps >= DRIVE_DISTANCE_MM:
            # Driven DRIVE_DISTANCE_MM inside GPS range — primary stop.
            break

        if dist_traveled >= max_dist:
            # Safety cap: hit total distance limit before completing the run.
            if gps_aligned:
                print(
                    f"[pos_fusion_test] WARNING — GPS coverage ended before "
                    f"{DRIVE_DISTANCE_MM:.0f} mm of fused data could be collected. "
                    f"Increase GPS_SEARCH_EXTRA_MM or reposition the robot closer to the tag."
                )
            else:
                print(
                    f"[pos_fusion_test] WARNING — GPS was never acquired during the "
                    f"{max_dist:.0f} mm drive. The fused position equals raw odometry "
                    f"throughout; no fusion comparison is available in the plot."
                )
            break

        # Warn once when the primary distance is passed without GPS.
        if (
            GPS_SEARCH_EXTRA_MM > 0
            and not gps_aligned
            and not warned_no_gps
            and dist_traveled >= DRIVE_DISTANCE_MM
        ):
            print(
                f"[pos_fusion_test] GPS not yet acquired at {dist_traveled:.0f} mm. "
                f"Continuing up to {GPS_SEARCH_EXTRA_MM:.0f} mm more to find GPS "
                f"coverage…"
            )
            warned_no_gps = True

        robot.set_velocity(DRIVE_SPEED_MM_S, 0.0)

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

    t_end = time.monotonic()
    robot.stop()
    print(
        f"[pos_fusion_test] Done — travelled {dist_traveled:.1f} mm "
        f"in {t_end - t_start:.1f} s"
    )


# =============================================================================
# Plotting
# =============================================================================

def _shade_gps_regions(ax, t: np.ndarray, gps_active: np.ndarray) -> None:
    """Shade time periods when GPS was active green."""
    in_region = False
    t_start = 0.0
    for i, active in enumerate(gps_active):
        if active and not in_region:
            t_start = t[i]
            in_region = True
        elif not active and in_region:
            ax.axvspan(t_start, t[i], alpha=0.15, color="green", label="_nolegend_")
            in_region = False
    if in_region:
        ax.axvspan(t_start, t[-1], alpha=0.15, color="green", label="_nolegend_")


def _plot_results(rec: _Record) -> None:
    t, odom_x, odom_y, fused_x, fused_y, gps_active = rec.arrays()

    error_mm = np.hypot(fused_x - odom_x, fused_y - odom_y)

    try:
        import matplotlib
        matplotlib.use("TkAgg")
    except Exception:
        pass

    # Layout: Y-progress (tall, top-left), 2D bird's-eye (top-right),
    #         X-drift (bottom-left), error magnitude (bottom-right).
    fig = plt.figure(figsize=(14, 9))
    gs  = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.32)
    ax_y    = fig.add_subplot(gs[0, 0])
    ax_traj = fig.add_subplot(gs[0, 1])
    ax_x    = fig.add_subplot(gs[1, 0])
    ax_err  = fig.add_subplot(gs[1, 1])

    total_dist = t[-1] * DRIVE_SPEED_MM_S if len(t) > 1 else DRIVE_DISTANCE_MM
    fig.suptitle(
        f"Position fusion — world-Y drive  "
        f"(α={POSITION_FUSION_ALPHA}, GPS acquired {'at start' if gps_active[0] else 'mid-drive' if gps_active.any() else 'never'})",
        fontsize=12,
    )
    gps_patch = mpatches.Patch(color="green", alpha=0.3, label="GPS active")

    # ── Panel 1 (top-left): Forward progress — primary tuning view ───────────
    _shade_gps_regions(ax_y, t, gps_active)
    ax_y.plot(t, odom_y,  lw=1.2, color="steelblue", label="Odometry Y")
    ax_y.plot(t, fused_y, lw=1.5, color="tomato",    linestyle="--", label="Fused Y")
    ax_y.axhline(DRIVE_DISTANCE_MM, color="black", lw=1.0, linestyle=":",
                 label=f"GPS-range target ({DRIVE_DISTANCE_MM:.0f} mm)")
    ax_y.set_xlabel("time (s)")
    ax_y.set_ylabel("Forward progress (mm)")
    ax_y.set_title("Forward progress toward target (primary)")
    ax_y.legend(handles=[*ax_y.get_legend_handles_labels()[0], gps_patch], fontsize=8)
    ax_y.grid(True, alpha=0.3)

    # ── Panel 2 (top-right): 2D bird's-eye trajectory ─────────────────────────
    # Ideal path is a straight line from the start marker to the end marker.
    x_range = max(np.abs(odom_x).max(), np.abs(fused_x).max(), 50.0)
    ax_traj.axvline(0.0, color="black", lw=1.0, linestyle=":", label="Ideal path (X=0)")
    ax_traj.plot(odom_x,  odom_y,  lw=1.2, color="steelblue", label="Odometry path")
    ax_traj.plot(fused_x, fused_y, lw=1.5, color="tomato",    linestyle="--", label="Fused path")
    ax_traj.plot(0.0, 0.0,               "go", markersize=8, label="Start marker")
    ax_traj.plot(0.0, DRIVE_DISTANCE_MM, "kx", markersize=10, markeredgewidth=2,
                 label=f"GPS-range target ({DRIVE_DISTANCE_MM:.0f} mm)")
    ax_traj.plot(odom_x[-1],  odom_y[-1],  "bs", markersize=7, label="End (odom)")
    ax_traj.plot(fused_x[-1], fused_y[-1], "r^", markersize=7, label="End (fused)")
    ax_traj.set_xlim(-x_range * 1.5, x_range * 1.5)
    ax_traj.set_xlabel("X (mm)  [should stay ≈ 0]")
    ax_traj.set_ylabel("Y (mm)")
    ax_traj.set_title("2D trajectory (bird's-eye)")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.legend(fontsize=7)
    ax_traj.grid(True, alpha=0.3)

    # ── Panel 3 (bottom-left): Lateral drift — should stay near zero ─────────
    _shade_gps_regions(ax_x, t, gps_active)
    ax_x.axhline(0.0, color="black", lw=0.8, linestyle=":")
    ax_x.plot(t, odom_x,  lw=1.2, color="steelblue", label="Odometry X")
    ax_x.plot(t, fused_x, lw=1.5, color="tomato",    linestyle="--", label="Fused X")
    ax_x.set_xlabel("time (s)")
    ax_x.set_ylabel("Lateral drift (mm)")
    ax_x.set_title("Lateral drift (should stay ≈ 0)")
    ax_x.legend(handles=[*ax_x.get_legend_handles_labels()[0][:2], gps_patch], fontsize=8)
    ax_x.grid(True, alpha=0.3)

    # ── Panel 4 (bottom-right): Fused − odometry error magnitude ─────────────
    _shade_gps_regions(ax_err, t, gps_active)
    ax_err.fill_between(t, error_mm, alpha=0.3, color="darkorange")
    ax_err.plot(t, error_mm, lw=1.2, color="darkorange", label="|fused − odom|")
    ax_err.set_xlabel("time (s)")
    ax_err.set_ylabel("error (mm)")
    ax_err.set_title("Fused − odometry error magnitude")
    ax_err.legend(handles=[*ax_err.get_legend_handles_labels()[0][:1], gps_patch], fontsize=8)
    ax_err.grid(True, alpha=0.3)

    try:
        plt.savefig(_PLOT_PATH, dpi=150)
        print(f"[pos_fusion_test] Plot saved → {_PLOT_PATH}")
    except Exception as exc:
        print(f"[pos_fusion_test] Could not save plot: {exc}")

    try:
        plt.show()
    except Exception:
        pass


# =============================================================================
# Entry point
# =============================================================================

def run(robot: Robot) -> None:
    # Start with zero translation; the test learns the GPS→odom offset when
    # the robot first enters GPS view.
    robot.set_gps_offset(0.0, 0.0)
    robot.set_position_fusion_alpha(POSITION_FUSION_ALPHA)

    rec = _Record()
    _drive_straight(robot, rec)
    _plot_results(rec)


# =============================================================================
# ROS2 node entry point
# =============================================================================

def main(args=None) -> None:
    import signal
    import threading

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    from robot.robot import Robot as _Robot

    class _TestNode(Node):
        def __init__(self) -> None:
            super().__init__("position_fusion_test")
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
