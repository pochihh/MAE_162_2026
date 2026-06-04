from __future__ import annotations
import math
import os
import subprocess
import threading
import time

import cv2
import numpy as np

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import (
    Button, DCMotorMode, DEFAULT_FSM_HZ, LED, Motor,
    LIDAR_FOV_DEG, LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM, LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM, LIDAR_RANGE_MIN_MM,
)
from robot.util import densify_polyline
from robot.examples.lidar_viz import LidarViz

# ---------------------------------------------------------------------------
# Robot hardware configuration
# ---------------------------------------------------------------------------

POSITION_UNIT        = Unit.MM
WHEEL_DIAMETER       = 78.0
WHEEL_BASE           = 317.0
INITIAL_THETA_DEG    = 90.0

LEFT_WHEEL_MOTOR         = Motor.DC_M2
LEFT_WHEEL_DIR_INVERTED  = False
RIGHT_WHEEL_MOTOR        = Motor.DC_M1
RIGHT_WHEEL_DIR_INVERTED = True

# ---------------------------------------------------------------------------
# Pure Pursuit parameters (segments 1 and 3 — no cones)
# ---------------------------------------------------------------------------

PP_LOOKAHEAD_MM   = 100.0
PP_MAX_LINEAR     = 140.0
PP_MAX_ANGULAR    = 1.5
PP_GOAL_TOL       = 20.0
PP_ALPHA_LD       = 0.7
PP_X_L            = 300.0
PP_OFFSET         = 0.0
PP_LANE_WIDTH     = 500.0

# ---------------------------------------------------------------------------
# LAPF parameters — cone avoidance segment (1220, 305) → (1220, 3350)
# ---------------------------------------------------------------------------

LAPF_GOAL_X_MM       = 1525.0
LAPF_GOAL_Y_MM       = 3350.0
LAPF_VELOCITY_MM_S   = 120.0
LAPF_TOLERANCE_MM    = 300.0
LAPF_MAX_ANGULAR     = 1.2
LAPF_LEASH_MM        = 250.0
LAPF_HALF_ANGLE_DEG  = 85.0    # <90° keeps virtual target in forward hemisphere; was 120° (caused backward spin)
LAPF_REPULSION_MM    = 430.0   # cone surface → start of gradient (inflation + 215mm reaction zone)
LAPF_INFLATION_MM    = 165.0   # robot half-width (165) + 50mm safety margin
LAPF_TARGET_SPD_MM_S = 200.0
LAPF_REPULSION_GAIN  = 200.0   # 6 simultaneous cones × 5.3 each was 32× attraction; halved to reduce stack-up
LAPF_ATTRACTION_GAIN = 3.0     # was 1.0; 3× boost to help pull through initial all-ahead-cone phase
LAPF_EMA_ALPHA       = 0.35
LAPF_MAX_S           = 55.0    # fallback: leave SEG2 after this many seconds even if goal not reached

# ---------------------------------------------------------------------------
# FSM timing and start triggers
# ---------------------------------------------------------------------------

STATUS_INTERVAL_S    = 0.5
BTN3_HOLD_TICKS      = 10   # ~0.2 s at 50 Hz — debounce glitches shorter than this
GREEN_HOLD_TICKS     = 10   # same hold required before green-light trigger fires
GREEN_MIN_CONFIDENCE = 0.30 # must match vision_node confidence_threshold in run_robot.sh

# ---------------------------------------------------------------------------
# SEG3 wall-following parameters — L-channel (1525,3350)→(2440,3350)→(2440,330)
# East leg: left (north) wall only — ignores right side so cone gaps don't attract.
# South leg: both walls solid — centres between them.
# ---------------------------------------------------------------------------

SEG3_WALL_RANGE_MM  = 900.0   # max lidar range for wall detection
SEG3_WALL_MIN_PTS   = 4       # minimum points to trust a wall reading
SEG3_STOP_MM        = 200.0   # full-stop if forward obstacle < this
SEG3_CRUISE_MM      = 450.0   # full speed when forward clear > this
SEG3_MAX_SPEED      = 140.0   # mm/s
SEG3_MAX_ANG_DEG    = 30.0    # deg/s cap — prevents spiralling during wall convergence

SEG3_K_HEADING           = 0.8    # deg/s per deg of wall slope angle
SEG3_K_LATERAL           = 0.08   # deg/s per mm of lateral error from target

SEG3_LEFT_TARGET_MM      = 280.0  # 560mm corridor / 2 = centre
SEG3_CORNER_MM           = 280.0  # turn south when forward obstacle < this (end of east leg)
SEG3_EAST_MIN_TRAVEL_MM  = 700.0  # must travel this far east before corner detection is allowed
SEG3_SOUTH_DIST_MM       = 3100.0 # total odometry distance in south leg before done
SEG3_SOUTH_WALL_MM       = 2300.0 # walls end here — coast straight for the remaining distance

# ---------------------------------------------------------------------------
# GPS position fusion
# ---------------------------------------------------------------------------

POSITION_FUSION_ALPHA = 0.0   # GPS weight for complementary filter (0–1)
GPS_TAG_ID            = 13     # ArUco tag ID to track (-1 = accept any tag)

# ---------------------------------------------------------------------------
# Camera tracking (SEG4) — constants
# ---------------------------------------------------------------------------

_CAM_DEVICE  = "/dev/video10"
_CAM_WIDTH   = 640
_CAM_HEIGHT  = 480
_CAM_FPS     = 30
_FRAME_BYTES = _CAM_WIDTH * _CAM_HEIGHT * 3

_HSV_RED_LOW1  = np.array([159, 131,  99], dtype=np.uint8)
_HSV_RED_HIGH1 = np.array([179, 255, 255], dtype=np.uint8)
_HSV_RED_LOW2  = np.array([  0, 131,  99], dtype=np.uint8)
_HSV_RED_HIGH2 = np.array([ 11, 255, 255], dtype=np.uint8)
_MIN_BLOB_AREA = 500

SHOOT_CHANNEL    = 15
SHOOT_A_DEG      = 95.0
SHOOT_B_DEG      = 170.0
SHOOT_SETTLE_S   = 0.5
SHOOT_COOLDOWN_S = 2.0

PAN_CHANNEL    = 16
PAN_CENTER_DEG = 90.0
PAN_MIN_DEG    = 0.0
PAN_MAX_DEG    = 180.0
PAN_SCAN_STEP  = 2.0

STEP_INTERVAL_S  = 0.8
COARSE_THRESH_PX = 100
PAN_STEP_COARSE  = 5.0
PAN_STEP_FINE    = 0.9
CENTER_TOL_PX    = 15
AIM_OFFSET_PX    = 2

_US_PER_DEG          = 1000.0 / 180.0
PAN_DITHER_THRESH_PX = 15
PAN_DITHER_FWD_DEG   = 13.5 / _US_PER_DEG
PAN_DITHER_BACK_DEG  = 11.7 / _US_PER_DEG
PAN_DITHER_REST_S    = 0.6
PAN_LOCK_CHECK_S     = 0.5

PITCH_MOTOR            = 3
PITCH_PWM              = 200
PITCH_STEP_INTERVAL_S  = 0.8
PITCH_COARSE_THRESH_PX = 50
PITCH_PULSE_COARSE_S   = 0.18
PITCH_PULSE_FINE_S     = 0.012
PITCH_SCAN_PWM         = 80
PITCH_SCAN_HALF_S      = 1.5

CAM_ALPHA   = 1.2
CAM_BETA    = 10
_TARGET_EMA = 0.35

CAMERA_TRACK_WAIT_S = 10.0   # pause after SEG3_SOUTH before tracking begins

_frame_lock   = threading.Lock()
_latest_frame: np.ndarray | None = None

# ---------------------------------------------------------------------------
# Map path — split at cone corridor boundaries
# ---------------------------------------------------------------------------

# Segment 1: start → entry of cone corridor

PATH_SEG1_CTRL = [
    (   0.0,    0.0),
    (   0.0, 3350.0),
    ( 580.0, 3350.0),
    ( 580.0,  450.0),
    (1525.0,  450.0),
]


# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def _camera_thread() -> None:
    global _latest_frame
    cmd = [
        "ffmpeg", "-loglevel", "quiet",
        "-f", "v4l2", "-input_format", "yuyv422",
        "-video_size", f"{_CAM_WIDTH}x{_CAM_HEIGHT}",
        "-framerate", str(_CAM_FPS),
        "-i", _CAM_DEVICE,
        "-f", "rawvideo", "-pix_fmt", "bgr24", "-",
    ]
    while True:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        print("[cam] ffmpeg started")
        while True:
            raw = proc.stdout.read(_FRAME_BYTES)
            if len(raw) < _FRAME_BYTES:
                print("[cam] stream ended — restarting…")
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                (_CAM_HEIGHT, _CAM_WIDTH, 3)
            ).copy()
            frame = cv2.convertScaleAbs(frame, alpha=CAM_ALPHA, beta=CAM_BETA)
            with _frame_lock:
                _latest_frame = frame


def _detect_target(frame: np.ndarray) -> list[tuple[int, int, int]]:
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, _HSV_RED_LOW1, _HSV_RED_HIGH1),
        cv2.inRange(hsv, _HSV_RED_LOW2, _HSV_RED_HIGH2),
    )
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area > _MIN_BLOB_AREA:
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            blobs.append((int(cx), int(cy), int(radius), area))
    blobs.sort(key=lambda b: b[3], reverse=True)
    return [(cx, cy, r) for cx, cy, r, _ in blobs]


# ---------------------------------------------------------------------------
# SEG3 wall-following core
# ---------------------------------------------------------------------------

def _seg3_measure_wall(
    robot_pts: list[tuple[float, float]],
) -> tuple[float | None, float, float]:
    """Return (perp_dist_mm, wall_slope_deg, fwd_min_mm) from left-wall lidar points.
    perp_dist_mm: perpendicular distance to left wall, or None if too few points.
    wall_slope_deg: +ve = wall diverging ahead (steer left), -ve = converging (steer right).
    fwd_min_mm: nearest obstacle in forward ±25° cone, used to throttle speed.
    """
    left_lo  = math.radians(50.0)
    left_hi  = math.radians(130.0)
    fwd_half = math.radians(25.0)

    left_xs: list[float] = []
    left_ys: list[float] = []
    fwd_min = SEG3_WALL_RANGE_MM

    for x, y in robot_pts:
        r = math.hypot(x, y)
        if r < 80 or r > SEG3_WALL_RANGE_MM:
            continue
        a = math.atan2(y, x)
        if left_lo <= a <= left_hi:
            left_xs.append(x)
            left_ys.append(y)
        if abs(a) < fwd_half:
            fwd_min = min(fwd_min, r)

    if len(left_xs) < SEG3_WALL_MIN_PTS:
        return None, 0.0, fwd_min

    xs_arr = np.array(left_xs)
    ys_arr = np.array(left_ys)

    slope = 0.0
    intercept = float(np.median(left_ys))
    if xs_arr.std() > 20.0:
        slope, intercept = np.polyfit(xs_arr, ys_arr, 1)
        slope = float(slope)
        intercept = float(intercept)

    perp_dist = abs(intercept) / math.sqrt(slope ** 2 + 1.0)

    return perp_dist, math.degrees(math.atan(slope)), fwd_min


# ---------------------------------------------------------------------------
# Setup and navigation helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.enable_lidar()
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=LIDAR_RANGE_MAX_MM,
        fov_deg=LIDAR_FOV_DEG,
    )
    robot.enable_gps()
    robot.set_position_fusion_alpha(POSITION_FUSION_ALPHA)
    robot.set_tracked_tag_id(GPS_TAG_ID)


def init_pp(robot: Robot, ctrl_points: list, spacing: float = 20.0) -> None:
    robot._nav_follow_pp_path(
        lookahead_distance=PP_LOOKAHEAD_MM,
        max_linear_speed=PP_MAX_LINEAR,
        max_angular_speed=PP_MAX_ANGULAR,
        goal_tolerance=PP_GOAL_TOL,
        obstacles_range=450.0,
        view_angle=math.radians(70.0),
        safe_dist=250.0,
        avoidance_delay=150,
        alpha_Ld=PP_ALPHA_LD,
        offset=PP_OFFSET,
        lane_width=PP_LANE_WIDTH,
        obstacle_avoidance=False,
        x_L=PP_X_L,
    )
    path = densify_polyline(ctrl_points, spacing=spacing)
    robot.planner.set_path(path)


def start_lapf(robot: Robot):
    return robot.lapf_to_goal(
        LAPF_GOAL_X_MM,
        LAPF_GOAL_Y_MM,
        velocity=LAPF_VELOCITY_MM_S,
        tolerance=LAPF_TOLERANCE_MM,
        leash_length_mm=LAPF_LEASH_MM,
        repulsion_range_mm=LAPF_REPULSION_MM,
        target_speed_mm_s=LAPF_TARGET_SPD_MM_S,
        max_angular_rad_s=LAPF_MAX_ANGULAR,
        repulsion_gain=LAPF_REPULSION_GAIN,
        attraction_gain=LAPF_ATTRACTION_GAIN,
        force_ema_alpha=LAPF_EMA_ALPHA,
        inflation_margin_mm=LAPF_INFLATION_MM,
        leash_half_angle_deg=LAPF_HALF_ANGLE_DEG,
        blocking=False,
    )


def _is_green_light(robot: Robot) -> bool:
    """Return True if the vision node reports a confirmed green traffic light."""
    if not robot.is_vision_active(timeout_s=3.0):
        return False
    color = robot.get_detection_attribute(
        "traffic light", "color", min_confidence=GREEN_MIN_CONFIDENCE
    )
    return color == "green"


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def print_status(robot: Robot, label: str = "") -> None:
    x, y, theta = robot.get_pose()
    confirmed = robot.get_obstacle_tracks(include_unconfirmed=False)
    raw_pts   = robot.get_obstacles()
    pose_src  = "fused" if robot.has_fused_pose() else "odom"
    gps_str   = "GPS:active" if robot.is_gps_active() else "GPS:stale"
    tag = f"[{label}]" if label else ""
    print(f"{tag}  pose=({x:.0f}, {y:.0f}) mm [{pose_src}]  θ={theta:.1f}°"
          f"  {gps_str}  raw_pts={len(raw_pts)}  confirmed={len(confirmed)}")


# ---------------------------------------------------------------------------
# FSM entry point
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    try:
        _run(robot)
    finally:
        robot.stop()
        show_idle_leds(robot)
        print("[FSM] motors stopped")


def _run(robot: Robot) -> None:
    # Firmware is in IDLE (run_robot.sh sent RESET but NOT START).
    # SYS_ODOM_PARAM_SET is only accepted in IDLE state (firmware allowConfig gate).
    # Set params first, then transition to RUNNING.

    configure_robot(robot)

    # Mandatory confirmation loop — must get firmware echo before going to RUNNING.
    for _attempt in range(5):
        ok = robot.set_odometry_parameters(
            wheel_diameter=WHEEL_DIAMETER,
            wheel_base=WHEEL_BASE,
            initial_theta_deg=INITIAL_THETA_DEG,
            left_motor_id=LEFT_WHEEL_MOTOR,
            left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
            right_motor_id=RIGHT_WHEEL_MOTOR,
            right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
            timeout=2.0,
        )
        if ok:
            p = robot.get_odometry_parameters()
            print(f"[FSM] odom params confirmed (attempt {_attempt + 1}): "
                  f"L=M{p['left_motor_number']} inv={p['left_motor_dir_inverted']}  "
                  f"R=M{p['right_motor_number']} inv={p['right_motor_dir_inverted']}")
            break
        print(f"[FSM] odom params not confirmed (attempt {_attempt + 1}), retrying…")
        time.sleep(0.2)
    else:
        print("[FSM] FATAL: odom params never confirmed — aborting.")
        return

    # Transition firmware IDLE → RUNNING.
    for attempt in range(5):
        ok = robot.set_state(FirmwareState.RUNNING, timeout=10.0)
        if ok:
            print(f"[FSM] firmware → RUNNING (attempt {attempt + 1})")
            break
        print(f"[FSM] set_state RUNNING failed (attempt {attempt + 1}), retrying…")
        time.sleep(1.0)
    else:
        print("[FSM] WARNING: could not confirm RUNNING state — continuing anyway")

    time.sleep(0.5)          # let firmware stabilise before odometry reset
    robot.reset_odometry()
    robot.wait_for_odometry_reset(timeout=3.0)
    x, y, theta = robot.get_pose()
    print(f"[FSM] odometry reset confirmed  pose=({x:.0f},{y:.0f}) θ={theta:.1f}°")
    if abs(theta - INITIAL_THETA_DEG) > 5.0:
        print(f"[FSM] WARNING: theta={theta:.1f}° expected {INITIAL_THETA_DEG}° — resetting again")
        robot.reset_odometry()
        robot.wait_for_odometry_reset(timeout=3.0)
        x, y, theta = robot.get_pose()
        print(f"[FSM] re-reset  pose=({x:.0f},{y:.0f}) θ={theta:.1f}°")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    last_status_at = time.monotonic()

    # Pre-build segment 1 path so IDLE state is ready immediately
    init_pp(robot, PATH_SEG1_CTRL)

    print("=" * 60)
    print("MAP RUN — PP × LAPF")
    print("  Seg1: (0,0) → (1220,305)   [Pure Pursuit]")
    print("  Seg2: (1220,305) → (1220,3350)  [LAPF cone avoidance]")
    print("  Seg3: (1220,3350) → (2745,0)  [Pure Pursuit]")
    print("  BTN_3 = start   BTN_2 = stop")
    print("=" * 60)

    _VALID_START_STATES = {
        "IDLE", "PP_SEG1", "LAPF_SEG2", "SEG3_EAST", "SEG3_SOUTH",
        "CAMERA_WAIT", "CAMERA_TRACK",
    }
    _start_state = os.environ.get("ROBOT_START_STATE", "").strip().upper() or "IDLE"
    if _start_state not in _VALID_START_STATES:
        print(f"[FSM] WARNING: unknown ROBOT_START_STATE={_start_state!r} — defaulting to IDLE")
        _start_state = "IDLE"
    if _start_state != "IDLE":
        print(f"[FSM] *** skipping to state: {_start_state} ***")

    state = _start_state
    lapf_handle      = None
    lapf_start_time  = None
    viz              = None
    seg3_east_start    = (0.0, 0.0)
    seg3_south_start   = (0.0, 0.0)
    seg3_coast_heading: float | None = None

    # Camera tracking state (initialised when CAMERA_TRACK is first entered)
    camera_track_initialized = False
    camera_wait_start        = 0.0
    cam_pan_deg              = PAN_CENTER_DEG
    cam_pitch_pwm            = 0
    cam_scan_dir             = 1
    cam_pitch_scan_dir       = 1
    cam_pitch_scan_start     = 0.0
    cam_last_pitch_step_at   = 0.0
    cam_pitch_pulse_end_at   = 0.0
    cam_smooth_tx: float | None  = None
    cam_smooth_ty: float | None  = None
    cam_last_step_at         = 0.0
    cam_last_shot_at         = 0.0
    cam_shooting             = False
    cam_pan_dither_phase     = False
    cam_pan_dither_dir       = 0.0
    cam_pan_dither_next_at   = 0.0
    cam_pan_lock_check_until = 0.0
    cam_locked_pos: tuple[int, int] | None = None
    cam_centred_since: float | None        = None

    # When jumping into a camera state, seed the wait timer and LEDs correctly
    if _start_state == "CAMERA_WAIT":
        camera_wait_start = time.monotonic()
        show_idle_leds(robot)
    elif _start_state == "CAMERA_TRACK":
        show_moving_leds(robot)
    else:
        show_idle_leds(robot)

    # BTN_3 start logic: require button seen released, then held for BTN3_HOLD_TICKS
    # consecutive ticks before accepting — guards against firmware-init glitches.
    btn3_was_released = False
    btn3_hold_count   = 0
    green_hold_count  = 0

    while True:
        now = time.monotonic()

        # ── IDLE: wait for green light or BTN3 hold ───────────────────────
        if state == "IDLE":
            show_idle_leds(robot)

            # ── Trigger 1: BTN_3 held ────────────────────────────────────────
            btn3_now = robot.get_button(Button.BTN_3)
            if not btn3_now:
                btn3_was_released = True
                btn3_hold_count   = 0
            elif btn3_was_released:
                btn3_hold_count += 1
                if btn3_hold_count == 1:
                    print(f"[IDLE] BTN3 held — waiting {BTN3_HOLD_TICKS} ticks")

            # ── Trigger 2: green traffic light ──────────────────────────────
            if _is_green_light(robot):
                green_hold_count += 1
                if green_hold_count == 1:
                    print(f"[IDLE] green light seen — waiting {GREEN_HOLD_TICKS} ticks")
            else:
                green_hold_count = 0

            # ── Periodic stoplight status ────────────────────────────────────
            if now - last_status_at >= STATUS_INTERVAL_S:
                last_status_at = now
                dets = robot.get_detections("traffic light")
                if not robot.is_vision_active(timeout_s=3.0):
                    print("[IDLE] vision_node not active")
                elif dets:
                    best = max(dets, key=lambda d: float(d["confidence"]))
                    color = best.get("attributes", {}).get("color", {}).get("value", "?")
                    conf  = float(best["confidence"])
                    print(f"[IDLE] traffic light detected  color={color}  conf={conf:.0%}")
                else:
                    print("[IDLE] vision active — no traffic light detected")

            # ── OR: either trigger fires the start ───────────────────────────
            btn3_fire  = btn3_was_released and btn3_hold_count  >= BTN3_HOLD_TICKS
            green_fire = green_hold_count >= GREEN_HOLD_TICKS
            if btn3_fire or green_fire:
                trigger = "BTN3" if btn3_fire else "green light"
                print(f"[FSM] IDLE → PP_SEG1  (trigger: {trigger})")
                robot.reset_odometry()
                robot.wait_for_odometry_reset(timeout=2.0)
                x, y, theta = robot.get_pose()
                print(f"[FSM] start reset  pose=({x:.0f},{y:.0f}) θ={theta:.1f}°")
                init_pp(robot, PATH_SEG1_CTRL)
                show_moving_leds(robot)
                btn3_hold_count  = 0
                green_hold_count = 0
                state = "PP_SEG1"

        # ── PP_SEG1: pure pursuit to cone corridor entry ───────────────────
        elif state == "PP_SEG1":
            if now - last_status_at >= STATUS_INTERVAL_S:
                print_status(robot, "PP_SEG1")
                last_status_at = now
            result = robot._nav_follow_pp_path_loop()
            if result == "IDLE":
                x, y, theta = robot.get_pose()
                print(f"[FSM] PP_SEG1 done  pose=({x:.0f},{y:.0f}) θ={theta:.1f}°"
                      f" — turning to 90°")

                robot.turn_to(90.0, blocking=True, tolerance_deg=3.0, timeout=10.0)
                x, y, theta = robot.get_pose()
                print(f"[FSM] turn done  θ={theta:.1f}° → starting LAPF")
                lapf_handle     = start_lapf(robot)
                lapf_start_time = time.monotonic()
                viz = LidarViz(robot, goal=(LAPF_GOAL_X_MM, LAPF_GOAL_Y_MM))
                viz.start()
                print("[FSM] visualiser → ros2_ws/runtime_output/lidar_viz.png")
                state = "LAPF_SEG2"

        # ── LAPF_SEG2: potential-field cone avoidance ─────────────────────
        elif state == "LAPF_SEG2":
            if now - last_status_at >= STATUS_INTERVAL_S:
                x, y, theta = robot.get_pose()
                vt = robot.get_virtual_target()
                confirmed   = robot.get_obstacle_tracks(include_unconfirmed=False)
                unconfirmed = robot.get_obstacle_tracks(include_unconfirmed=True)
                raw_pts     = robot.get_obstacles()
                vt_str    = f"vt=({vt[0]:.0f},{vt[1]:.0f})" if vt is not None else "vt=none"
                pose_src  = "fused" if robot.has_fused_pose() else "odom"
                gps_str   = "GPS:on" if robot.is_gps_active() else "GPS:off"
                remaining = ((LAPF_GOAL_X_MM - x) ** 2 + (LAPF_GOAL_Y_MM - y) ** 2) ** 0.5
                if confirmed:
                    nearest = min(confirmed, key=lambda o: math.hypot(o["x"] - x, o["y"] - y))
                    nd = math.hypot(nearest["x"] - x, nearest["y"] - y)
                    near_str = f"nearest=({nearest['x']:.0f},{nearest['y']:.0f}) d={nd:.0f}mm"
                else:
                    near_str = "nearest=none"
                print(f"[LAPF_SEG2]  pose=({x:.0f},{y:.0f}) [{pose_src}] θ={theta:.1f}°"
                      f"  rem={remaining:.0f} mm  {vt_str}  {gps_str}"
                      f"  raw={len(raw_pts)}  unc={len(unconfirmed)}  conf={len(confirmed)}"
                      f"  {near_str}")
                last_status_at = now
            lapf_done = lapf_handle is not None and lapf_handle.is_finished()
            lapf_timed_out = (lapf_start_time is not None and
                              time.monotonic() - lapf_start_time > LAPF_MAX_S)
            if lapf_done or lapf_timed_out:
                if lapf_timed_out and not lapf_done:
                    print("[FSM] LAPF_SEG2 timeout — forcing transition to SEG3")
                    if lapf_handle is not None:
                        lapf_handle.cancel()
                robot.stop()
                x, y, theta = robot.get_pose()
                print(f"[FSM] LAPF_SEG2 done  pose=({x:.0f},{y:.0f}) θ={theta:.1f}° → turning east")
                robot.turn_to(0.0, blocking=True, tolerance_deg=5.0, timeout=8.0)
                x, y, theta = robot.get_pose()
                seg3_east_start = (x, y)
                print(f"[FSM] turned east  θ={theta:.1f}° → SEG3_EAST")
                show_moving_leds(robot)
                state = "SEG3_EAST"

        # ── SEG3_EAST: wall-follow east until corner ──────────────────────
        elif state == "SEG3_EAST":
            raw_pts = robot.get_obstacles()
            perp_dist, wall_slope_deg, fwd_min = _seg3_measure_wall(raw_pts)
            x, y, theta_deg = robot.get_pose()
            east_travel = math.hypot(x - seg3_east_start[0], y - seg3_east_start[1])

            corner_ready = east_travel >= SEG3_EAST_MIN_TRAVEL_MM and fwd_min < SEG3_CORNER_MM
            if corner_ready:
                robot.stop()
                print(f"[FSM] SEG3_EAST: east wall at {fwd_min:.0f}mm after {east_travel:.0f}mm → turning south")
                robot.turn_to(270.0, blocking=True, tolerance_deg=5.0, timeout=8.0)
                x, y, _ = robot.get_pose()
                seg3_south_start   = (x, y)
                seg3_coast_heading = None
                print(f"[FSM] turned south  south-start=({x:.0f},{y:.0f}) → SEG3_SOUTH")
                show_moving_leds(robot)
                state = "SEG3_SOUTH"
            else:
                lat_err = (perp_dist - SEG3_LEFT_TARGET_MM) if perp_dist is not None else 0.0
                angular = SEG3_K_HEADING * wall_slope_deg + SEG3_K_LATERAL * lat_err
                angular = max(-SEG3_MAX_ANG_DEG, min(SEG3_MAX_ANG_DEG, angular))
                t = (fwd_min - SEG3_STOP_MM) / max(1.0, SEG3_CRUISE_MM - SEG3_STOP_MM)
                robot.set_velocity(SEG3_MAX_SPEED * max(0.0, min(1.0, t)), angular)

            if now - last_status_at >= STATUS_INTERVAL_S:
                pd_str = f"{perp_dist:.0f}" if perp_dist is not None else "N/A"
                print(f"[SEG3_EAST]  pose=({x:.0f},{y:.0f}) θ={theta_deg:.1f}°"
                      f"  wall={pd_str}mm  slope={wall_slope_deg:+.1f}°  fwd={fwd_min:.0f}mm"
                      f"  east={east_travel:.0f}mm (need {SEG3_EAST_MIN_TRAVEL_MM:.0f} before corner)")
                last_status_at = now

        # ── SEG3_SOUTH: wall-follow south to finish ───────────────────────
        elif state == "SEG3_SOUTH":
            x, y, theta_deg = robot.get_pose()
            dist_traveled = math.hypot(x - seg3_south_start[0], y - seg3_south_start[1])

            if dist_traveled >= SEG3_SOUTH_DIST_MM:
                robot.stop()
                if viz is not None:
                    viz.stop()
                target_deg = theta_deg + 45.0
                print(f"[FSM] SEG3_SOUTH done at {dist_traveled:.0f}mm — rotating 45° left to {target_deg:.1f}°")
                robot.turn_to(target_deg, blocking=True, tolerance_deg=3.0, timeout=8.0)
                x, y, theta_deg = robot.get_pose()
                print(f"[FSM] post-south turn done  θ={theta_deg:.1f}°")
                show_idle_leds(robot)
                print(f"[FSM] → camera track in {CAMERA_TRACK_WAIT_S:.0f}s")
                print_status(robot, "SEG3_DONE")
                camera_wait_start = now
                state = "CAMERA_WAIT"

            elif dist_traveled < SEG3_SOUTH_WALL_MM:
                raw_pts = robot.get_obstacles()
                perp_dist, wall_slope_deg, fwd_min = _seg3_measure_wall(raw_pts)
                if fwd_min < SEG3_STOP_MM:
                    robot.set_velocity(0.0, 0.0)
                else:
                    lat_err = (perp_dist - SEG3_LEFT_TARGET_MM) if perp_dist is not None else 0.0
                    angular = SEG3_K_HEADING * wall_slope_deg + SEG3_K_LATERAL * lat_err
                    angular = max(-SEG3_MAX_ANG_DEG, min(SEG3_MAX_ANG_DEG, angular))
                    t = (fwd_min - SEG3_STOP_MM) / max(1.0, SEG3_CRUISE_MM - SEG3_STOP_MM)
                    robot.set_velocity(SEG3_MAX_SPEED * max(0.0, min(1.0, t)), angular)
                if now - last_status_at >= STATUS_INTERVAL_S:
                    pd_str = f"{perp_dist:.0f}" if perp_dist is not None else "N/A"
                    print(f"[SEG3_SOUTH]  pose=({x:.0f},{y:.0f}) θ={theta_deg:.1f}°"
                          f"  dist={dist_traveled:.0f}/{SEG3_SOUTH_DIST_MM:.0f}mm"
                          f"  wall={pd_str}mm  slope={wall_slope_deg:+.1f}°  fwd={fwd_min:.0f}mm")
                    last_status_at = now
            else:
                if seg3_coast_heading is None:
                    seg3_coast_heading = theta_deg
                heading_err = seg3_coast_heading - theta_deg
                angular = max(-SEG3_MAX_ANG_DEG, min(SEG3_MAX_ANG_DEG, 1.5 * heading_err))
                robot.set_velocity(SEG3_MAX_SPEED, angular)
                if now - last_status_at >= STATUS_INTERVAL_S:
                    print(f"[SEG3_SOUTH]  pose=({x:.0f},{y:.0f}) θ={theta_deg:.1f}°"
                          f"  dist={dist_traveled:.0f}/{SEG3_SOUTH_DIST_MM:.0f}mm"
                          f"  [coasting  target={seg3_coast_heading:.1f}°  herr={heading_err:+.1f}°]")
                    last_status_at = now

        # ── CAMERA_WAIT: 10-second pause after SEG3 ──────────────────────
        elif state == "CAMERA_WAIT":
            if now - last_status_at >= STATUS_INTERVAL_S:
                remaining = max(0.0, CAMERA_TRACK_WAIT_S - (now - camera_wait_start))
                print(f"[CAMERA_WAIT] camera track starts in {remaining:.0f}s")
                last_status_at = now
            if now - camera_wait_start >= CAMERA_TRACK_WAIT_S:
                print("[FSM] CAMERA_WAIT done → CAMERA_TRACK")
                show_moving_leds(robot)
                state = "CAMERA_TRACK"

        # ── CAMERA_TRACK: red-target tracking with pan servo + pitch motor ─
        elif state == "CAMERA_TRACK":
            if not camera_track_initialized:
                # vision_node holds /dev/video10 — kill it so ffmpeg can open the device
                print("[track] stopping vision_node to free camera device…")
                subprocess.run(["pkill", "-f", "vision_node"], check=False)
                time.sleep(0.5)
                threading.Thread(target=_camera_thread, daemon=True).start()
                _bridge_deadline = time.monotonic() + 10.0
                print("[track] waiting for hardware bridge…")
                while robot.get_dc_state() is None:
                    if time.monotonic() > _bridge_deadline:
                        print("[track] WARNING: bridge not responding — enabling anyway")
                        break
                    time.sleep(0.05)
                robot.enable_servo(PAN_CHANNEL)
                robot.enable_servo(SHOOT_CHANNEL)
                _m3_deadline = time.monotonic() + 8.0
                print(f"[track] enabling M{PITCH_MOTOR} (PWM)…")
                while True:
                    robot.enable_motor(PITCH_MOTOR, DCMotorMode.PWM)
                    time.sleep(0.1)
                    dc = robot.get_dc_state()
                    if dc is not None and dc.motors[PITCH_MOTOR - 1].mode == int(DCMotorMode.PWM):
                        print(f"[track] M{PITCH_MOTOR} enabled")
                        break
                    if time.monotonic() > _m3_deadline:
                        print(f"[track] WARNING: M{PITCH_MOTOR} enable not confirmed — proceeding anyway")
                        break
                cam_pan_deg              = PAN_CENTER_DEG
                cam_pitch_pwm            = 0
                cam_scan_dir             = 1
                cam_pitch_scan_dir       = 1
                cam_pitch_scan_start     = now
                cam_last_pitch_step_at   = 0.0
                cam_pitch_pulse_end_at   = 0.0
                cam_smooth_tx            = None
                cam_smooth_ty            = None
                cam_last_step_at         = 0.0
                cam_last_shot_at         = now - SHOOT_COOLDOWN_S
                cam_shooting             = False
                cam_pan_dither_phase     = False
                cam_pan_dither_dir       = 0.0
                cam_pan_dither_next_at   = 0.0
                cam_pan_lock_check_until = 0.0
                cam_locked_pos           = None
                cam_centred_since        = None
                robot.set_servo(PAN_CHANNEL, cam_pan_deg)
                robot.set_servo(SHOOT_CHANNEL, SHOOT_A_DEG)
                robot.set_motor_pwm(PITCH_MOTOR, 0)
                print(f"[track] scanning for red target  pan={cam_pan_deg:.0f}°")
                camera_track_initialized = True
                next_tick = time.monotonic()

            with _frame_lock:
                frame = _latest_frame

            if frame is not None:
                blobs = _detect_target(frame)
                detection: tuple[int, int, int] | None = None

                if blobs:
                    if cam_locked_pos is None:
                        detection = blobs[0]
                    else:
                        lx, ly = cam_locked_pos
                        detection = min(blobs, key=lambda b: (b[0]-lx)**2 + (b[1]-ly)**2)
                    cam_locked_pos = (detection[0], detection[1])
                else:
                    cam_locked_pos = None

                if detection:
                    tx, ty, _ = detection
                    cam_pitch_scan_start = now

                    if cam_smooth_tx is None:
                        cam_smooth_tx, cam_smooth_ty = float(tx), float(ty)
                    else:
                        cam_smooth_tx += _TARGET_EMA * (tx - cam_smooth_tx)
                        cam_smooth_ty += _TARGET_EMA * (ty - cam_smooth_ty)

                    pan_err   = cam_smooth_tx - (_CAM_WIDTH  // 2)
                    pitch_err = (cam_smooth_ty - AIM_OFFSET_PX) - (_CAM_HEIGHT // 2)

                    centred = abs(pan_err) <= CENTER_TOL_PX and abs(pitch_err) <= CENTER_TOL_PX
                    if centred and not cam_shooting:
                        if cam_centred_since is None:
                            cam_centred_since = now
                        elif now - cam_centred_since >= 0.5 and now - cam_last_shot_at >= SHOOT_COOLDOWN_S:
                            cam_centred_since = None
                            cam_shooting      = True
                            cam_last_shot_at  = now
                            print("[shoot] firing CH15")
                            def _shoot():
                                nonlocal cam_shooting
                                robot.set_servo(SHOOT_CHANNEL, SHOOT_B_DEG)
                                time.sleep(SHOOT_SETTLE_S)
                                robot.set_servo(SHOOT_CHANNEL, SHOOT_A_DEG)
                                cam_shooting = False
                                print("[shoot] reset CH15")
                            threading.Thread(target=_shoot, daemon=True).start()
                    else:
                        cam_centred_since = None

                    if not cam_shooting and now >= cam_pan_lock_check_until:
                        if abs(pan_err) > CENTER_TOL_PX:
                            if abs(pan_err) > PAN_DITHER_THRESH_PX:
                                if now - cam_last_step_at >= STEP_INTERVAL_S:
                                    step = PAN_STEP_COARSE if abs(pan_err) > COARSE_THRESH_PX else PAN_STEP_FINE
                                    correction = -math.copysign(step, pan_err)
                                    cam_pan_dither_phase = False
                                    cam_pan_deg = float(np.clip(
                                        cam_pan_deg + correction, PAN_MIN_DEG, PAN_MAX_DEG,
                                    ))
                                    robot.set_servo(PAN_CHANNEL, cam_pan_deg)
                                    cam_last_step_at = now
                            else:
                                if now >= cam_pan_dither_next_at:
                                    if not cam_pan_dither_phase:
                                        cam_pan_dither_dir = math.copysign(1.0, pan_err)
                                        correction = -cam_pan_dither_dir * PAN_DITHER_FWD_DEG
                                    else:
                                        correction = (
                                            0.0 if abs(pan_err) <= CENTER_TOL_PX
                                            else cam_pan_dither_dir * PAN_DITHER_BACK_DEG
                                        )
                                    cam_pan_dither_phase = not cam_pan_dither_phase
                                    cam_pan_dither_next_at = now + PAN_DITHER_REST_S
                                    cam_pan_deg = float(np.clip(
                                        cam_pan_deg + correction, PAN_MIN_DEG, PAN_MAX_DEG,
                                    ))
                                    robot.set_servo(PAN_CHANNEL, cam_pan_deg)
                        else:
                            cam_pan_dither_phase     = False
                            cam_pan_lock_check_until = now + PAN_LOCK_CHECK_S

                    if not cam_shooting and abs(pitch_err) > CENTER_TOL_PX:
                        if now < cam_pitch_pulse_end_at:
                            cam_pitch_pwm = int(math.copysign(PITCH_PWM, pitch_err))
                        elif now - cam_last_pitch_step_at >= PITCH_STEP_INTERVAL_S:
                            dur = PITCH_PULSE_COARSE_S if abs(pitch_err) > PITCH_COARSE_THRESH_PX else PITCH_PULSE_FINE_S
                            cam_pitch_pulse_end_at = now + dur
                            cam_last_pitch_step_at = now
                            cam_pitch_pwm = int(math.copysign(PITCH_PWM, pitch_err))
                        else:
                            cam_pitch_pwm = 0
                    else:
                        cam_pitch_pwm = 0
                    robot.set_motor_pwm(PITCH_MOTOR, cam_pitch_pwm)

                else:
                    cam_smooth_tx          = None
                    cam_smooth_ty          = None
                    cam_pitch_pulse_end_at = 0.0
                    cam_locked_pos         = None

                    if now - cam_last_step_at >= STEP_INTERVAL_S:
                        cam_pan_deg += cam_scan_dir * PAN_SCAN_STEP
                        if cam_pan_deg >= PAN_MAX_DEG:
                            cam_pan_deg  = PAN_MAX_DEG
                            cam_scan_dir = -1
                        elif cam_pan_deg <= PAN_MIN_DEG:
                            cam_pan_deg  = PAN_MIN_DEG
                            cam_scan_dir = 1
                        robot.set_servo(PAN_CHANNEL, cam_pan_deg)
                        cam_last_step_at = now

                    if now - cam_pitch_scan_start >= PITCH_SCAN_HALF_S:
                        cam_pitch_scan_dir   = -cam_pitch_scan_dir
                        cam_pitch_scan_start = now
                    cam_pitch_pwm = cam_pitch_scan_dir * PITCH_SCAN_PWM
                    robot.set_motor_pwm(PITCH_MOTOR, cam_pitch_pwm)

        # ── BTN_2: emergency stop (checked every tick) ────────────────────
        if robot.get_button(Button.BTN_2):
            robot.stop()
            if viz is not None:
                viz.stop()
            show_idle_leds(robot)
            print("[FSM] BTN_2 — aborted")
            return

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
