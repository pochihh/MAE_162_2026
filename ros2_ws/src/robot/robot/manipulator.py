"""
manipulator.py — cylinder detection and grab sequence for the 3-DOF arm.

Joint mapping (per hardware):
  Yaw   — stepper 1         (swivel) — DISABLED, mechanically out of service.
                               Robot must be physically pre-aligned so the
                               cylinder is directly ahead (Y ≈ 0) before
                               calling grab_cylinder().
  J2    — servo 1           (shoulder/elevator: 180°=retracted, 0°=extended)
  J3    — servo 2           (elbow/extension:     0°=retracted, 180°=extended)
  Grip  — servo 3           (180°=open, ~80°=closed)

Sensor reframing (LiDAR origin → yaw-joint origin, mm):
  x_arm = x_lidar + 0
  y_arm = y_lidar - 115
  z_arm = z_lidar + 93.5   (LiDAR is 93.5 mm below the yaw joint)
"""

from __future__ import annotations

import math
import time

import numpy as np

from robot.robot import Robot

# ---------------------------------------------------------------------------
# Arm geometry — tune to physical robot (mm)
# ---------------------------------------------------------------------------

ARM_L2_MM = 210.0   # shoulder pivot → elbow pivot
ARM_L3_MM = 345.0   # elbow pivot → end-effector claw midpoint

# ---------------------------------------------------------------------------
# Servo angle conventions
# ---------------------------------------------------------------------------

GRIPPER_OPEN_DEG  = 180.0
GRIPPER_CLOSE_DEG =  110.0   # tune to actual gripper geometry for a firm grip

# Starting (fully retracted) servo angles — arm is stowed.
SHOULDER_RETRACTED_DEG = 180.0   # servo 1 retracted position
ELBOW_RETRACTED_DEG    =   0.0   # servo 2 retracted position

# ---------------------------------------------------------------------------
# Motion speed limits
# ---------------------------------------------------------------------------

# Maximum rate at which any servo is allowed to move (degrees per second).
# Lower = smoother / slower.  Raise if the arm needs to move faster.
SERVO_SPEED_DEG_S = 10

# Update interval for the servo sweep loop (seconds).
SERVO_STEP_INTERVAL_S = 0.02    # 50 Hz — matches typical servo PWM refresh

# Number of interpolation steps for the straight-line approach trajectory.
# More steps = smoother Cartesian path.
APPROACH_STEPS = 40

# Minimum angle change (degrees) before a new set_servo command is sent.
# Per-step deltas at SERVO_SPEED_DEG_S * SERVO_STEP_INTERVAL_S can fall below
# a hobby servo's pulse resolution, causing it to hunt/jitter instead of
# moving smoothly. Sub-threshold deltas are accumulated and flushed once they
# cross this deadband.
SERVO_POSITION_DEADBAND_DEG = 0.5

# ---------------------------------------------------------------------------
# Sensor reframing offsets (slide values, mm)
# ---------------------------------------------------------------------------

LIDAR_TO_ARM_X_MM =   0.0
LIDAR_TO_ARM_Y_MM = -115.0
LIDAR_TO_ARM_Z_MM =  93.5    # positive = yaw joint is above the LiDAR plane

# ---------------------------------------------------------------------------
# Yaw stepper conversion and motion config
# ---------------------------------------------------------------------------
#
# DISABLED — stepper 1 (yaw/swivel) is mechanically out of service.
# The robot is now expected to be physically pre-aligned so the cylinder
# sits directly ahead of the manipulator (Y ≈ 0) before grab_cylinder() is
# called. All stepper-related setup and motion below is commented out.
#
# # steps/rev = motor_steps_per_rev × microstep_divisor × gear_ratio
# YAW_STEPS_PER_REV = 200 * 16   # tune: 200-step motor, 1/16 microstepping, no gearing
# YAW_STEPS_PER_DEG = YAW_STEPS_PER_REV / 360.0
#
# BASE_STEPPER_MAX_VEL   = 800    # steps/s — tune for desired yaw speed
# BASE_STEPPER_ACCEL     = 400    # steps/s² — tune for desired yaw ramp rate
#
# # Tracks cumulative stepper position (steps from boot zero) so successive
# # grab_cylinder() calls issue correctly delta-relative moves instead of
# # always treating the current position as zero.
# _current_base_steps: int = 0

# ---------------------------------------------------------------------------
# Cylinder detection parameters
# ---------------------------------------------------------------------------

CYLINDER_CLUSTER_MIN_PTS    = 5
CYLINDER_CLUSTER_MAX_RAD_MM = 80.0

# Aspect-ratio filter: reject clusters whose PCA major/minor axis ratio
# exceeds this threshold.  A flat wall segment is long and thin (high ratio);
# a cylinder cross-section is roughly square (ratio ≈ 1–2).
CYLINDER_MAX_ASPECT_RATIO = 3.0

# Circle-fit filter: expected physical radius range of the target cylinder (mm).
# Reject clusters whose best-fit circle radius falls outside this window.
CYLINDER_RADIUS_MIN_MM =  20.0
CYLINDER_RADIUS_MAX_MM = 120.0

# Maximum allowed mean residual from the circle fit (mm).
# Points on a true circular arc sit close to the fitted circle; wall points
# produce a much higher residual because they are collinear, not circular.
CYLINDER_FIT_RESIDUAL_MAX_MM = 15.0

# Lift height after gripping (mm)
LIFT_HEIGHT_MM = 30.0


# ---------------------------------------------------------------------------
# Motion helpers
# ---------------------------------------------------------------------------

def _servo_sweep(
    robot: Robot,
    channel: int,
    start_deg: float,
    end_deg: float,
    speed_deg_s: float = SERVO_SPEED_DEG_S,
    step_interval_s: float = SERVO_STEP_INTERVAL_S,
) -> None:
    """
    Move a single servo from start_deg to end_deg at a controlled rate.

    Sends incremental angle commands at `step_interval_s` intervals so the
    servo ramps smoothly instead of snapping to the target in one jump.
    """
    delta = end_deg - start_deg
    if abs(delta) < 0.1:
        return

    deg_per_step = speed_deg_s * step_interval_s
    n_steps = max(1, int(abs(delta) / deg_per_step))
    for i in range(1, n_steps + 1):
        angle = start_deg + delta * (i / n_steps)
        robot.set_servo(channel, angle)
        time.sleep(step_interval_s)


def _servos_sweep_simultaneous(
    robot: Robot,
    starts: list[tuple[int, float]],
    ends: list[tuple[int, float]],
    speed_deg_s: float = SERVO_SPEED_DEG_S,
    step_interval_s: float = SERVO_STEP_INTERVAL_S,
) -> None:
    """
    Move multiple servos simultaneously from their start angles to their end
    angles, stepping all channels together at each tick.

    `starts` and `ends` are lists of (channel, angle_deg) in matching order.
    The number of steps is determined by whichever servo has the largest travel
    so that all servos arrive at the same time (coordinated motion).
    """
    deltas = [e[1] - s[1] for s, e in zip(starts, ends)]
    max_travel = max(abs(d) for d in deltas) if deltas else 0.0
    if max_travel < 0.1:
        return

    deg_per_step = speed_deg_s * step_interval_s
    n_steps = max(1, int(max_travel / deg_per_step))

    for i in range(1, n_steps + 1):
        t = i / n_steps
        for (ch, s_ang), delta in zip(starts, deltas):
            robot.set_servo(ch, s_ang + delta * t)
        time.sleep(step_interval_s)


# ---------------------------------------------------------------------------
# IK helpers
# ---------------------------------------------------------------------------

def _aspect_ratio(cluster: np.ndarray) -> float:
    """
    Return the PCA aspect ratio (major axis / minor axis) of a point cluster.

    A ratio close to 1 means the cluster is roughly circular (cylinder).
    A very high ratio means the cluster is elongated (wall segment).
    """
    centred = cluster - cluster.mean(axis=0)
    # 2×2 covariance matrix; eigenvalues give variance along each principal axis.
    cov = np.cov(centred.T)
    eigvals = np.linalg.eigvalsh(cov)          # sorted ascending
    minor, major = float(eigvals[0]), float(eigvals[1])
    if minor < 1e-6:
        return float("inf")
    return math.sqrt(major / minor)            # ratio of std-devs, not variances


def _circle_fit(cluster: np.ndarray) -> tuple[float, float, float, float]:
    """
    Algebraic least-squares circle fit (Kasa method).

    Solves the linear system:
      [x  y  1] [D]   [-(x²+y²)]
                [E] =
                [F]

    where the circle is  x²+y² + Dx + Ey + F = 0,
    giving centre  (-D/2, -E/2)  and  radius  sqrt(D²/4 + E²/4 − F).

    Returns (cx, cy, radius, mean_residual) all in the same units as cluster.
    """
    x = cluster[:, 0]
    y = cluster[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = -(x**2 + y**2)
    result, *_ = np.linalg.lstsq(A, b, rcond=None)
    D, E, F = result
    cx = -D / 2.0
    cy = -E / 2.0
    r2 = cx**2 + cy**2 - F
    radius = math.sqrt(max(0.0, r2))
    residuals = np.abs(np.sqrt((x - cx)**2 + (y - cy)**2) - radius)
    return cx, cy, radius, float(residuals.mean())


def _find_cylinder(robot: Robot) -> tuple[float, float, float] | None:
    """
    Return (X, Y, Z) of the nearest cylindrical cluster in the manipulator
    frame, or None if no suitable object is found.

    Detection pipeline:
      1. Forward-facing filter  — discard all points behind the robot.
      2. Radius clustering      — group nearby points into candidate clusters.
      3. Aspect-ratio filter    — reject elongated clusters (walls, edges).
      4. Circle-fit filter      — reject clusters that don't fit a circle with
                                  a plausible cylinder radius and low residual.
      5. Nearest survivor       — pick the closest passing cluster.
    """
    obstacles = robot._get_obstacles_mm()
    print(f"[manipulator] raw lidar points: {len(obstacles)}")
    if len(obstacles) < CYLINDER_CLUSTER_MIN_PTS:
        print(f"[manipulator] not enough raw points (< {CYLINDER_CLUSTER_MIN_PTS}) — is the lidar enabled / publishing?")
        return None

    pts = np.array(obstacles, dtype=float)   # shape (N, 2)

    # 1. Keep only forward-facing points (robot +x direction).
    pts = pts[pts[:, 0] > 0.0]
    print(f"[manipulator] forward-facing points (x>0): {len(pts)}")
    if len(pts) < CYLINDER_CLUSTER_MIN_PTS:
        print(f"[manipulator] not enough forward points (< {CYLINDER_CLUSTER_MIN_PTS})")
        return None

    # 2. Greedy radius clustering — group points within CYLINDER_CLUSTER_MAX_RAD_MM.
    visited  = np.zeros(len(pts), dtype=bool)
    clusters: list[np.ndarray] = []

    for i in range(len(pts)):
        if visited[i]:
            continue
        dists   = np.linalg.norm(pts - pts[i], axis=1)
        members = np.where(dists <= CYLINDER_CLUSTER_MAX_RAD_MM)[0]
        if len(members) >= CYLINDER_CLUSTER_MIN_PTS:
            visited[members] = True
            clusters.append(pts[members])

    print(f"[manipulator] clusters formed: {len(clusters)}")
    for cluster in clusters:
        centroid = cluster.mean(axis=0)
        print(f"[manipulator]   cluster: {len(cluster)} pts, centroid=({centroid[0]:.0f}, {centroid[1]:.0f}) mm")

    if not clusters:
        print("[manipulator] no clusters — try increasing CYLINDER_CLUSTER_MAX_RAD_MM or decreasing CYLINDER_CLUSTER_MIN_PTS")
        return None

    # 3 & 4. Geometry filters — keep only clusters that look like a cylinder.
    passing: list[tuple[float, np.ndarray]] = []   # (distance_to_centroid, cluster)

    for cluster in clusters:
        centroid = cluster.mean(axis=0)
        dist     = math.hypot(centroid[0], centroid[1])

        # Aspect-ratio check: reject wall-like elongated clusters.
        aspect = _aspect_ratio(cluster)
        if aspect > CYLINDER_MAX_ASPECT_RATIO:
            print(f"[manipulator] cluster at {dist:.0f} mm rejected — aspect ratio {aspect:.1f} > {CYLINDER_MAX_ASPECT_RATIO}")
            continue

        # Circle-fit check: residual and radius must match a real cylinder.
        _, _, fit_radius, mean_residual = _circle_fit(cluster)
        if not (CYLINDER_RADIUS_MIN_MM <= fit_radius <= CYLINDER_RADIUS_MAX_MM):
            print(f"[manipulator] cluster at {dist:.0f} mm rejected — fit radius {fit_radius:.1f} mm out of range")
            continue
        if mean_residual > CYLINDER_FIT_RESIDUAL_MAX_MM:
            print(f"[manipulator] cluster at {dist:.0f} mm rejected — fit residual {mean_residual:.1f} mm > {CYLINDER_FIT_RESIDUAL_MAX_MM}")
            continue

        print(f"[manipulator] cluster at {dist:.0f} mm passed — aspect {aspect:.1f}, radius {fit_radius:.1f} mm, residual {mean_residual:.1f} mm")
        passing.append((dist, cluster))

    if not passing:
        return None

    # 5. Pick the closest passing cluster.
    passing.sort(key=lambda t: t[0])
    best_cluster = passing[0][1]
    cx_body, cy_body = best_cluster.mean(axis=0)

    x_arm = cx_body + LIDAR_TO_ARM_X_MM
    y_arm = cy_body + LIDAR_TO_ARM_Y_MM
    z_arm = 0.0     + LIDAR_TO_ARM_Z_MM

    return (x_arm, y_arm, z_arm)


def _ik_planar(r: float, z_drop: float) -> tuple[float, float]:
    """
    2-D IK for the shoulder + elbow given planar reach `r` and vertical drop
    `z_drop` (positive = target is below the yaw joint).

    Uses the atan2 formulation (teammate's method) which handles the full
    ±180° range cleanly and avoids the sign ambiguity of acos near workspace
    boundaries.

    Returns (theta2_deg, theta3_deg) using the hardware conventions:
      theta2_deg — servo 1 (180°=retracted / 0°=extended)
      theta3_deg — servo 2 (  0°=retracted / 180°=extended)
    """
    d = math.hypot(r, z_drop)
    max_reach = ARM_L2_MM + ARM_L3_MM
    min_reach = abs(ARM_L2_MM - ARM_L3_MM)
    if not (min_reach < d < max_reach):
        d = max(min_reach + 1.0, min(d, max_reach - 1.0))

    # atan2 IK: D is the cosine of the elbow angle via Law of Cosines.
    D = (r**2 + z_drop**2 - ARM_L2_MM**2 - ARM_L3_MM**2) / (2.0 * ARM_L2_MM * ARM_L3_MM)
    D = max(-1.0, min(1.0, D))

    # Elbow-down solution (negative sqrt keeps the arm above the target).
    theta3_rad = math.atan2(-math.sqrt(1.0 - D**2), D)

    theta2_rad = math.atan2(z_drop, r) - math.atan2(
        ARM_L3_MM * math.sin(theta3_rad),
        ARM_L2_MM + ARM_L3_MM * math.cos(theta3_rad),
    )

    # Map to servo conventions: servo 1 inverted (180°=retracted), servo 2 direct.
    theta2_deg = max(0.0, min(180.0, 180.0 - math.degrees(theta2_rad)))
    theta3_deg = max(0.0, min(180.0,         math.degrees(theta3_rad) + 180.0))

    return theta2_deg, theta3_deg


def _ik(X: float, Y: float, Z: float) -> tuple[float, float]:
    """
    Planar IK — returns (theta2_deg, theta3_deg).

    Yaw is no longer computed: stepper 1 is mechanically disabled and the
    robot is assumed to be physically pre-aligned so the cylinder lies
    directly ahead (Y is ignored). Reach `r` is taken from X only.

    `Z` follows the file's documented convention (positive = target is
    below the yaw joint), but `_ik_planar`'s internal frame is standard
    math convention (positive = above), so the sign is flipped here.
    """
    r = X
    return _ik_planar(r, -Z)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def grab_cylinder(robot: Robot) -> bool:
    """
    Detect the nearest cylinder in front of the robot, position the arm over
    it using inverse kinematics, grip it, and lift it slightly off the ground.

    Sequence:
      1. Open gripper.
      2. (Stepper yaw — disabled. Robot must be pre-aligned so the cylinder
          is directly ahead of the manipulator before calling this function.)
      3. Shoulder + elbow sweep simultaneously along a straight Cartesian path
         from the retracted pose to the target pose.
      4. Close gripper.
      5. Raise shoulder slightly to lift the object.

    Returns True on success, False if no suitable object is detected.
    """
    print("[manipulator] scanning for cylinder...")

    target = _find_cylinder(robot)
    if target is None:
        print("[manipulator] no cylindrical object detected")
        return False

    X, Y, Z = target
    print(f"[manipulator] target at arm-frame (X={X:.1f} Y={Y:.1f} Z={Z:.1f}) mm")

    theta2_target, theta3_target = _ik(X, Y, Z)
    print(f"[manipulator] IK  θ2={theta2_target:.1f}°  θ3={theta3_target:.1f}°")

    # ------------------------------------------------------------------
    # 1. Open gripper (smooth sweep from wherever it is to fully open).
    # ------------------------------------------------------------------
    robot.enable_servo(3)
    _servo_sweep(robot, 3, GRIPPER_CLOSE_DEG, GRIPPER_OPEN_DEG)

    # ------------------------------------------------------------------
    # 2. Yaw stepper — DISABLED (stepper 1 mechanically out of service).
    #    The robot must be physically pre-aligned so the cylinder is
    #    directly ahead of the manipulator (Y ≈ 0) before this is called.
    # ------------------------------------------------------------------
    # global _current_base_steps
    # robot.step_set_config(1, max_velocity=BASE_STEPPER_MAX_VEL, acceleration=BASE_STEPPER_ACCEL)
    # robot.step_enable(1)
    #
    # target_base_steps = int(round(theta1_deg * YAW_STEPS_PER_DEG))
    # steps_to_move = target_base_steps - _current_base_steps
    #
    # if steps_to_move != 0:
    #     print(f"[manipulator] yaw {theta1_deg:.1f}° — moving {steps_to_move:+d} steps (absolute target: {target_base_steps})")
    #     ok = robot.step_move(1, steps_to_move, blocking=True, timeout=10.0)
    #     if not ok:
    #         print("[manipulator] stepper timed out — aborting")
    #         return False
    #     _current_base_steps = target_base_steps
    # else:
    #     print("[manipulator] yaw already at target — no stepper move needed")

    # ------------------------------------------------------------------
    # 3. Straight-line Cartesian approach.
    #
    # The arm starts at its retracted pose (r=0 conceptually — just the
    # stowed joint angles).  We interpolate `r` from 0 to the target
    # reach while keeping the end-effector height constant (z_drop fixed),
    # which produces a horizontal straight-line path in the sagittal plane.
    # At each step both servos are commanded together.
    # ------------------------------------------------------------------
    robot.enable_servo(1)
    robot.enable_servo(2)

    # Reach is taken directly from X — the robot is pre-aligned so the
    # cylinder lies in the manipulator's sagittal plane (Y ≈ 0).
    # Z follows the "positive = below yaw joint" convention; _ik_planar's
    # internal frame is "positive = above", so flip the sign (see _ik).
    r_target  = X
    z_drop    = -Z

    # Current (retracted) servo angles — used as the interpolation start.
    shoulder_now = SHOULDER_RETRACTED_DEG
    elbow_now    = ELBOW_RETRACTED_DEG

    print("[manipulator] beginning straight-line approach...")

    deg_per_step = SERVO_SPEED_DEG_S * SERVO_STEP_INTERVAL_S
    # Steps needed for the servo with the longest travel.
    shoulder_travel = abs(theta2_target - shoulder_now)
    elbow_travel    = abs(theta3_target - elbow_now)
    n_steps = max(APPROACH_STEPS,
                  int(max(shoulder_travel, elbow_travel) / deg_per_step))

    # Accumulated (rate-limited) target angles — used to rate-limit each step
    # so the IK solution's discontinuity near the workspace boundary (small
    # r_i) can't snap the servos straight to a far-away angle.
    shoulder_accum = shoulder_now
    elbow_accum    = elbow_now

    # Angles actually last sent to the servos — used for the deadband below.
    shoulder_sent = shoulder_now
    elbow_sent    = elbow_now

    for i in range(1, n_steps + 1):
        t = i / n_steps
        # Interpolate reach linearly — straight horizontal line in Cartesian space.
        r_i = r_target * t
        # Recompute IK at each intermediate reach to follow the line exactly.
        s_deg, e_deg = _ik_planar(r_i, z_drop)

        # Rate-limit: never move a servo more than deg_per_step from its
        # accumulated target.
        s_delta = max(-deg_per_step, min(deg_per_step, s_deg - shoulder_accum))
        e_delta = max(-deg_per_step, min(deg_per_step, e_deg - elbow_accum))
        shoulder_accum += s_delta
        elbow_accum    += e_delta

        # Deadband: only issue a new command once the accumulated target has
        # moved far enough from the last commanded angle to be a real step
        # for the servo, avoiding sub-resolution hunting/jitter.
        if abs(shoulder_accum - shoulder_sent) >= SERVO_POSITION_DEADBAND_DEG:
            shoulder_sent = shoulder_accum
            robot.set_servo(1, shoulder_sent)
        if abs(elbow_accum - elbow_sent) >= SERVO_POSITION_DEADBAND_DEG:
            elbow_sent = elbow_accum
            robot.set_servo(2, elbow_sent)

        time.sleep(SERVO_STEP_INTERVAL_S)

    # Ensure the final target is always reached exactly, even if the last
    # accumulated delta was below the deadband.
    if shoulder_sent != shoulder_accum:
        shoulder_sent = shoulder_accum
        robot.set_servo(1, shoulder_sent)
    if elbow_sent != elbow_accum:
        elbow_sent = elbow_accum
        robot.set_servo(2, elbow_sent)

    print("[manipulator] approach complete")

    # ------------------------------------------------------------------
    # 4. Close gripper slowly.
    # ------------------------------------------------------------------
    print("[manipulator] closing gripper")
    _servo_sweep(robot, 3, GRIPPER_OPEN_DEG, GRIPPER_CLOSE_DEG)

    # ------------------------------------------------------------------
    # 5. Lift — raise the shoulder slightly while keeping elbow fixed,
    #    commanding both servos together for a smooth upward arc.
    # ------------------------------------------------------------------
    lift_delta_deg = math.degrees(math.atan2(LIFT_HEIGHT_MM, r_target))
    theta2_lifted  = max(0.0, min(180.0, theta2_target - lift_delta_deg))
    theta3_lifted  = theta3_target   # elbow stays; shoulder carries the load

    print(f"[manipulator] lifting  shoulder {shoulder_sent:.1f}° → {theta2_lifted:.1f}°")
    _servos_sweep_simultaneous(
        robot,
        starts=[(1, shoulder_sent), (2, elbow_sent)],
        ends  =[(1, theta2_lifted), (2, theta3_lifted)],
    )

    print("[manipulator] cylinder gripped and lifted")
    return True
