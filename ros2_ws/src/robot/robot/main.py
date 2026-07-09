# A.R.M.O.R. Full Script - Integrated Mobility & Manipulation
# MAE 162E - Capstone Project
# Group 2, Spring 2026

from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline
from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    ServoChannel,
    Stepper,
    StepMoveType,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    POSITION_UNIT,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    LEFT_WHEEL_MOTOR,
    LEFT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    RIGHT_WHEEL_DIR_INVERTED,
)

# ---------------------------------------------------------------------------
# Sensor & Logic Toggles
# ---------------------------------------------------------------------------
ENABLE_LIDAR     = True
ENABLE_GPS       = False
ENABLE_VISION    = True
ENABLE_STOP_SIGN = True  

LIDAR_MOUNTED_ON_ARM = False  
INVERT_YAW_DIRECTION = True  

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50  
MIN_STOP_SIGN_CONFIDENCE = 0.50  

# ---------------------------------------------------------------------------
# Pure Pursuit & GPS Configuration (Phase 1)
# ---------------------------------------------------------------------------
TAG_ID = 13                 
GPS_POSITION_ALPHA = 0.01   

PP_VELOCITY_MM_S = 225.0  
PP_ANGULAR_VEL = 1.0      
PP_LOOKAHEAD_MM = 60.0                                     
PP_TOLERANCE_MM = 20.0

# ---------------------------------------------------------------------------
# DWA Obstacle Avoidance Configuration (Phase 2)
# ---------------------------------------------------------------------------
DWA_MAX_VEL_MM_S = 160.0  
DWA_MAX_ACC_MM_S2 = 500.0  
DWA_MAX_ANGULAR_RAD_S = 1.5
DWA_MAX_ANGULAR_ACC_RAD_S2 = 5.0

DWA_PREDICT_TIME_S = 4.0        
DWA_VELOCITY_SAMPLES = [7, 15]  
DWA_COST_GAINS = [1.0, 1.5, 2.0]
DWA_LOOKAHEAD_MM = 300.0        
DWA_TOLERANCE_MM = 50.0        
DWA_OBSTACLES_RANGE_MM = 600.0  

# Kinematic Footprint
ROBOT_FRONT_MM = 380.0  
ROBOT_REAR_MM = 50.0         
ROBOT_HALF_WIDTH_MM = 190.0  

# ---------------------------------------------------------------------------
# Manipulator & Perception Configuration
# ---------------------------------------------------------------------------
Z_LIDAR = 120.0 
Z_LTS = 85.0    
Z_SS = 8.5
L_1 = 54.2
L_J2J3 = 201.0
L_J3EE = 330.0        

X_0 = -115.0  
Y_0 = 0.0     

Z_0 = Z_LIDAR + Z_LTS + Z_SS
SHOULDER_OFFSET = Z_0 + L_1  

SHOULDER_SERVO = ServoChannel.CH_1
ELBOW_SERVO = ServoChannel.CH_2
GRIPPER_SERVO = ServoChannel.CH_3

SHOULDER_MIN = 2.0
SHOULDER_MAX = 200.0 
ELBOW_MIN = 2.0
ELBOW_MAX = 178.0
SHOULDER_MOUNT_OFFSET = -10.0  

SHOULDER_REST_DEG = 180.0   
ELBOW_REST_DEG = 10.0        
SHOULDER_MAST_DEG = 90.0     
ELBOW_MAST_DEG = 178.0       
GRIPPER_OPEN_DEG = 180.0    
GRIPPER_CLOSE_DEG = 95.0        

SCAN_FOV_DEG = 60.0
DEBRIS_MIN_DIST_MM = 100.0
DEBRIS_MAX_DIST_MM = 600.0
CLUSTER_TOLERANCE_MM = 80.0  
ISOLATION_RADIUS_MM = 100.0  
PICK_Z_HEIGHT_MM = 300.0        

BASE_STEPPER = Stepper.STEPPER_1
STEPS_PER_REV = 200.0          
BASE_STEPPER_VEL = 50                
BASE_STEPPER_ACCEL = 400        
BACKLASH_STEPS = 35             
DISCARD_ADDITIONAL_TURN_DEG = 90.0

# ---------------------------------------------------------------------------
# PATH CONFIGURATIONS
# ---------------------------------------------------------------------------
PP_PATH_CONTROL_POINTS_1 = [    
    (0.0, 0.0),           
    (0.0, 3630.0),        
    (510.0, 3670.0),      
    (530.0, 3670.0),      
    (540.0, 870.0),       
    (1370.0, 870.0),      
    (1400.0, 945.0),      
]
PP_DENSE_PATH_1 = densify_polyline(PP_PATH_CONTROL_POINTS_1, spacing=20.0)

DWA_PATH_CONTROL_POINTS = [
    (1400.0, 945.0),      
    (1400.0, 3650.0),
    (2200.0, 3650.0),
    (2200.0, 820.0), 
]
DWA_DENSE_PATH = densify_polyline(DWA_PATH_CONTROL_POINTS, spacing=20.0)

GPS_ZONE_X_MIN, GPS_ZONE_X_MAX = 300.0, 1000.0
GPS_ZONE_Y_MIN, GPS_ZONE_Y_MAX = 1370.0, 3370.0

DWA_ZONE_X_MIN, DWA_ZONE_X_MAX = 1100.0, 2100.0
DWA_ZONE_Y_MIN = 920.0

STOP_SIGN_PAUSE_SEC = 3.0
BLIND_DRIVE_VELOCITY_MM_S = 150.0
BLIND_DRIVE_DURATION_SEC = 3.5

# ---------------------------------------------------------------------------
# Helpers (Kinematics & Control)
# ---------------------------------------------------------------------------

class ArmKinematics3D:
    def calculate_ik(self, r_target_mm: float, z_target_mm: float) -> tuple[float, float]:
        r_arm = r_target_mm               
        z_arm = z_target_mm - SHOULDER_OFFSET   

        d = math.hypot(r_arm, z_arm)
        max_reach = L_J2J3 + L_J3EE
        if d > max_reach or d < abs(L_J2J3 - L_J3EE):
            raise ValueError(f"Target unreachable. Distance from shoulder is {d:.1f}mm (Max: {max_reach}mm)")
       
        D = (r_arm**2 + z_arm**2 - L_J2J3**2 - L_J3EE**2) / (2 * L_J2J3 * L_J3EE)
        D = max(min(D, 1.0), -1.0)
        
        theta3_rad = math.atan2(-math.sqrt(1 - D**2), D)
        theta2_rad = math.atan2(z_arm, r_arm) - math.atan2(
            L_J3EE * math.sin(theta3_rad), 
            L_J2J3 + L_J3EE * math.cos(theta3_rad)
        )

        return math.degrees(theta2_rad), math.degrees(theta3_rad)

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=425.0, 
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_LIDAR:
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
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_gps_offset(-20.7, -201.8) 
        robot.set_position_fusion_alpha(0.0)
        print(f"[sensor] GPS enabled — Tracking ArUco tag {TAG_ID} (Fusion starting OFF)")
        
    if ENABLE_VISION:
        robot.enable_vision()
        print("[sensor] Vision enabled")

def load_dwa_profile(robot: Robot, period: float):
    print("[CFG] Loading DWA Profile (Phase 2)")
    robot._init_dwa_planner(
        max_vel_mm=DWA_MAX_VEL_MM_S, max_acc_mm=DWA_MAX_ACC_MM_S2,
        max_angular_rad=DWA_MAX_ANGULAR_RAD_S, max_angular_acc_rad=DWA_MAX_ANGULAR_ACC_RAD_S2,
        lookahead_mm=DWA_LOOKAHEAD_MM, robot_front_mm=ROBOT_FRONT_MM,
        robot_rear_mm=ROBOT_REAR_MM, robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
        tolerance_mm=DWA_TOLERANCE_MM, gains_of_costs=DWA_COST_GAINS,
        period=period, predict_time=DWA_PREDICT_TIME_S,
        predict_velocity_samples_resolution=DWA_VELOCITY_SAMPLES,
        obstacles_range_mm=DWA_OBSTACLES_RANGE_MM, ttc_weight=0.0
    )

def start_pp_path_1(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=PP_DENSE_PATH_1, velocity=PP_VELOCITY_MM_S,
        lookahead=PP_LOOKAHEAD_MM, tolerance=PP_TOLERANCE_MM,
        advance_radius=PP_LOOKAHEAD_MM, max_angular_rad_s=PP_ANGULAR_VEL,
        blocking=False, 
    )

def command_shoulder_physical(robot: Robot, logical_deg: float) -> None:
    physical_deg = logical_deg + SHOULDER_MOUNT_OFFSET
    safe_phys = max(SHOULDER_MIN, min(SHOULDER_MAX, physical_deg))
    robot.set_servo_270(SHOULDER_SERVO, safe_phys)

def stow_arm(robot: Robot) -> None:
    command_shoulder_physical(robot, SHOULDER_REST_DEG)
    safe_e = max(ELBOW_MIN, min(ELBOW_MAX, ELBOW_REST_DEG))
    robot.set_servo_180_wide(ELBOW_SERVO, safe_e)

def move_servos_slowly(robot: Robot, curr_s: float, curr_e: float, target_s: float, target_e: float, step_deg=2.0, delay_s=0.02) -> None:
    safe_target_e = max(ELBOW_MIN, min(ELBOW_MAX, target_e))
    diff_s = target_s - curr_s
    diff_e = safe_target_e - curr_e
    
    max_diff = max(abs(diff_s), abs(diff_e))
    num_steps = int(math.ceil(max_diff / step_deg))
    if num_steps <= 0: return

    for i in range(1, num_steps + 1):
        fraction = i / float(num_steps)
        logical_s = curr_s + (diff_s * fraction)
        logical_e = curr_e + (diff_e * fraction)
        
        command_shoulder_physical(robot, logical_s)
        robot.set_servo_180_wide(ELBOW_SERVO, logical_e)
        time.sleep(delay_s)

def move_gripper_slowly(robot: Robot, curr_g: float, target_g: float, step_deg=1.0, delay_s=0.02) -> None:
    diff_g = target_g - curr_g
    num_steps = int(math.ceil(abs(diff_g) / step_deg))
    if num_steps <= 0: return
    for i in range(1, num_steps + 1):
        fraction = i / float(num_steps)
        pos = curr_g + (diff_g * fraction)
        robot.set_servo(GRIPPER_SERVO, pos)
        time.sleep(delay_s)

def command_yaw_stepper(robot: Robot, steps: int) -> None:
    if steps == 0: return
    robot.step_set_config(BASE_STEPPER, max_velocity=BASE_STEPPER_VEL, acceleration=BASE_STEPPER_ACCEL)
    robot.step_move(BASE_STEPPER, steps, StepMoveType.RELATIVE, blocking=True)

def find_traffic_light_color(robot: Robot) -> str | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC): return None
    best_color, best_confidence = None, -1.0
    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE: continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color not in ("red", "green"): continue
        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)
    return best_color

def is_stop_sign_detected(robot: Robot) -> bool:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC): return False
    for sign in robot.get_detections("stop sign"):
        if float(sign["confidence"]) >= MIN_STOP_SIGN_CONFIDENCE: return True
    return False

def scan_for_debris(robot: Robot, fov_limit_deg: float) -> tuple[float, float] | None:
    obstacles = robot.get_obstacles()
    if not obstacles: return None

    valid_points = []
    fov_rad = math.radians(fov_limit_deg)
    
    for ox, oy in obstacles:
        dist = math.hypot(ox, oy)
        angle = math.atan2(oy, ox)
        if DEBRIS_MIN_DIST_MM < dist < DEBRIS_MAX_DIST_MM and abs(angle) <= fov_rad:
            valid_points.append((ox, oy))

    if not valid_points: return None
    clusters = []
    for pt in valid_points:
        placed = False
        for cluster in clusters:
            if any(math.hypot(pt[0] - cp[0], pt[1] - cp[1]) < CLUSTER_TOLERANCE_MM for cp in cluster):
                cluster.append(pt)
                placed = True
                break
        if not placed:
            clusters.append([pt])

    centroids = [(sum(p[0] for p in c) / len(c), sum(p[1] for p in c) / len(c)) for c in clusters]
    best_target, min_dist = None, float('inf')

    for i, (cx1, cy1) in enumerate(centroids):
        isolated = True
        for j, (cx2, cy2) in enumerate(centroids):
            if i != j and math.hypot(cx2 - cx1, cy2 - cy1) < ISOLATION_RADIUS_MM:
                isolated = False
                break
        if isolated:
            dist = math.hypot(cx1, cy1)
            if dist < min_dist:
                min_dist = dist
                best_target = (cx1, cy1)

    return best_target

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR): robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)

# ---------------------------------------------------------------------------
# Main FSM Loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    time.sleep(0.5) 
    configure_robot(robot)

    state = "INIT"
    previous_state = ""
    recovery_ticks = 0
    
    drive_handle = None
    is_gps_fusion_active = False

    ik_solver = ArmKinematics3D()
    curr_shoulder = SHOULDER_REST_DEG
    curr_elbow = ELBOW_REST_DEG
    curr_gripper = GRIPPER_OPEN_DEG
    current_yaw_deg = 0.0
    last_move_direction = 1   
    target_dist_mm = 0.0
    target_yaw_deg = 0.0
    target_shoulder_deg = 0.0
    target_elbow_deg = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    
    next_tick = time.monotonic()
    last_log_time = time.monotonic()

    while True:
        now = time.monotonic()

        # Print Odometry logging at 0.05s intervals for mobility phases only
        if state in ["MOVING_PHASE_1", "MOVING_PHASE_2", "RECOVERY"] and (now - last_log_time >= 0.05):
            pose = robot.get_odometry_pose()
            _raw_odom = (pose[0], pose[1])
            class DummyMsg: pass
            msg = DummyMsg()
            msg.theta = pose[2]  
            print(f"odom=({_raw_odom[0]:.1f}, {_raw_odom[1]:.1f}) mm, θ ={float(math.degrees(msg.theta)):5.1f}°")
            last_log_time = now

        if state == "INIT":
            start_robot(robot)
            
            robot.enable_servo(SHOULDER_SERVO)
            robot.enable_servo(ELBOW_SERVO)
            robot.enable_servo(GRIPPER_SERVO)
            robot.step_enable(BASE_STEPPER)
            
            stow_arm(robot)
            robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
            
            print("[FSM] INIT (odometry reset, arm stowed)")
            print("[FSM] IDLE - Press BTN_1 or show GREEN LIGHT to begin Phase 1.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            
            button_pressed = robot.get_button(Button.BTN_1)
            green_light_seen = ENABLE_VISION and find_traffic_light_color(robot) == "green"
            
            if button_pressed or green_light_seen:
                start_robot(robot) 
                show_running_leds(robot)
                if green_light_seen:
                    print("[VISION] Green light detected! Start Moving via Pure Pursuit (Phase 1)!")
                else:
                    print("[UI] BTN_1 pressed. Start Moving via Pure Pursuit (Phase 1)!")
                    
                drive_handle = start_pp_path_1(robot)
                state = "MOVING_PHASE_1"
            
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot.")
                robot.shutdown()

        # ==============================================================================
        # PHASE 1: PURE PURSUIT
        # ==============================================================================
        elif state == "MOVING_PHASE_1":
            show_running_leds(robot)
            
            pose = robot.get_fused_pose() if (ENABLE_GPS and robot.has_fused_pose()) else robot.get_odometry_pose()
            current_x, current_y = pose[0], pose[1]
            
            in_gps_zone = (GPS_ZONE_X_MIN < current_x < GPS_ZONE_X_MAX) and (GPS_ZONE_Y_MIN < current_y < GPS_ZONE_Y_MAX)
            if in_gps_zone and not is_gps_fusion_active and ENABLE_GPS:
                robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
                is_gps_fusion_active = True
                print(f"[GPS] Entering GPS Zone. Fusion ENABLED (alpha={GPS_POSITION_ALPHA})")
            elif not in_gps_zone and is_gps_fusion_active and ENABLE_GPS:
                robot.set_position_fusion_alpha(0.0)
                is_gps_fusion_active = False
                print("[GPS] Leaving GPS Zone. Fusion DISABLED")
                
            if (DWA_ZONE_X_MIN < current_x < DWA_ZONE_X_MAX) and (current_y > DWA_ZONE_Y_MIN):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                
                print("\n[FSM] OBSTACLE PORTION REACHED: Terminating Phase 1 and Engaging DWA Planner...")
                load_dwa_profile(robot, period)
                state = "MOVING_PHASE_2"
                continue
            
            if robot.get_button(Button.BTN_2):
                if drive_handle is not None: drive_handle.cancel()
                robot.stop()
                state = "IDLE"
            elif drive_handle is not None and drive_handle.is_finished():
                if current_x > DWA_ZONE_X_MIN:
                    load_dwa_profile(robot, period)
                    state = "MOVING_PHASE_2"
                else:
                    robot.stop()
                    state = "IDLE"

        # ==============================================================================
        # PHASE 2: DWA (Obstacles) -> STOP SIGN SEQUENCE
        # ==============================================================================
        elif state == "MOVING_PHASE_2":
            show_running_leds(robot)

            button_pressed = robot.was_button_pressed(Button.BTN_2)
            stop_sign_seen = is_stop_sign_detected(robot) if (ENABLE_VISION and ENABLE_STOP_SIGN) else False
            
            if button_pressed:
                robot.stop()
                state = "IDLE"
                
            elif stop_sign_seen:
                robot.stop()
                show_idle_leds(robot)
                
                print(f"\n[VISION] Stop sign detected! Pausing for {STOP_SIGN_PAUSE_SEC} seconds.")
                time.sleep(STOP_SIGN_PAUSE_SEC)
                
                print(f"[FSM] Pause complete. Executing blind forward drive at {BLIND_DRIVE_VELOCITY_MM_S} mm/s for {BLIND_DRIVE_DURATION_SEC}s.")
                robot._send_body_velocity_mm(BLIND_DRIVE_VELOCITY_MM_S, 0.0)
                time.sleep(BLIND_DRIVE_DURATION_SEC)
                robot.stop()
                
                print("\n[FSM] ----------------------------------------------------")
                print("[FSM] Waiting 10 seconds for user to set up manipulator obstacles...")
                print("[FSM] ----------------------------------------------------\n")
                time.sleep(10.0)
                
                print("[FSM] Proceeding to Manipulation Task.")
                state = "SCAN_FOR_DEBRIS"
                
            else:
                status = robot._nav_follow_path_loop(DWA_DENSE_PATH, period)
                if status == "IDLE":
                    robot.stop()
                    state = "IDLE"
                elif status == "COLLISION":
                    previous_state = state  
                    state = "RECOVERY"

        # ==============================================================================
        # COLLISION RECOVERY
        # ==============================================================================
        elif state == "RECOVERY":
            show_running_leds(robot)
            robot._send_body_velocity_mm(-100.0, 0.0)
            recovery_ticks += 1
            max_recovery_ticks = int(1.5 * float(DEFAULT_FSM_HZ))
            
            if recovery_ticks >= max_recovery_ticks:
                robot.stop()
                recovery_ticks = 0  
                if previous_state == "MOVING_PHASE_1":
                    drive_handle = start_pp_path_1(robot)
                state = previous_state

            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                recovery_ticks = 0
                state = "IDLE"

        # ==============================================================================
        # MANIPULATION SEQUENCE (Post-Stop Sign)
        # ==============================================================================
        elif state == "SCAN_FOR_DEBRIS":
            target = scan_for_debris(robot, fov_limit_deg=SCAN_FOV_DEG)
            if target:
                tx_raw, ty_raw = target
                stepper_fwd = tx_raw - X_0 
                stepper_lat = ty_raw - Y_0
                raw_yaw_deg = math.degrees(math.atan2(stepper_lat, stepper_fwd))
                
                target_yaw_deg = -raw_yaw_deg if INVERT_YAW_DIRECTION else raw_yaw_deg
                target_dist_mm = math.hypot(stepper_fwd, stepper_lat)
                
                print(f"[PERCEPTION] Debris found. LiDAR Fwd:{tx_raw:.1f}, Lat:{ty_raw:.1f}")
                try:
                    t2_raw, t3_raw = ik_solver.calculate_ik(r_target_mm=target_dist_mm, z_target_mm=PICK_Z_HEIGHT_MM)
                    target_shoulder_deg = t2_raw
                    target_elbow_deg = t3_raw + 180.0
                    state = "ALIGN_BASE"
                except ValueError as e:
                    print(f"[WARNING] Debris spotted but unreachable: {e}. Rescanning...")
                    time.sleep(1.0)
            else:
                time.sleep(0.1)

        elif state == "ALIGN_BASE":
            if LIDAR_MOUNTED_ON_ARM:
                delta_deg = target_yaw_deg
                current_yaw_deg += delta_deg
            else:
                delta_deg = target_yaw_deg - current_yaw_deg
                current_yaw_deg = target_yaw_deg
                
            steps_to_move = int(delta_deg * (STEPS_PER_REV / 360.0))
            move_direction = 1 if steps_to_move >= 0 else -1
            
            if move_direction != last_move_direction and steps_to_move != 0:
                steps_to_command = steps_to_move + (move_direction * BACKLASH_STEPS)
            else:
                steps_to_command = steps_to_move
                
            last_move_direction = move_direction
            print(f"[STEPPER] Swiveling {delta_deg:.1f}°...")
            command_yaw_stepper(robot, steps_to_command)
            state = "REACH_ARM"

        elif state == "REACH_ARM":
            print(f"[ACTUATE] Reaching arm...")
            move_servos_slowly(robot, curr_shoulder, curr_elbow, target_shoulder_deg, target_elbow_deg)
            curr_shoulder, curr_elbow = target_shoulder_deg, target_elbow_deg
            state = "GRAB_AND_MAST"

        elif state == "GRAB_AND_MAST":
            print("[ACTUATE] Gripping object slowly...")
            move_gripper_slowly(robot, curr_gripper, GRIPPER_CLOSE_DEG)
            curr_gripper = GRIPPER_CLOSE_DEG
            time.sleep(0.5)
            
            print(f"[ACTUATE] Pulling into vertical MAST position...")
            move_servos_slowly(robot, curr_shoulder, curr_elbow, SHOULDER_MAST_DEG, ELBOW_MAST_DEG)
            curr_shoulder, curr_elbow = SHOULDER_MAST_DEG, ELBOW_MAST_DEG
            state = "DISCARD_AND_STOW"

        elif state == "DISCARD_AND_STOW":
            discard_delta = DISCARD_ADDITIONAL_TURN_DEG * last_move_direction
            discard_steps = int(discard_delta * (STEPS_PER_REV / 360.0))
            
            print(f"[STEPPER] Swinging {discard_delta}° further to discard...")
            command_yaw_stepper(robot, discard_steps)
            if LIDAR_MOUNTED_ON_ARM: current_yaw_deg += discard_delta
            
            move_servos_slowly(robot, curr_shoulder, curr_elbow, target_shoulder_deg, target_elbow_deg)
            curr_shoulder, curr_elbow = target_shoulder_deg, target_elbow_deg

            move_gripper_slowly(robot, curr_gripper, GRIPPER_OPEN_DEG)
            curr_gripper = GRIPPER_OPEN_DEG
            time.sleep(0.5)
            
            move_servos_slowly(robot, curr_shoulder, curr_elbow, SHOULDER_MAST_DEG, ELBOW_MAST_DEG)
            curr_shoulder, curr_elbow = SHOULDER_MAST_DEG, ELBOW_MAST_DEG

            home_delta = 0.0 - current_yaw_deg
            home_steps = int(home_delta * (STEPS_PER_REV / 360.0))
            home_direction = 1 if home_steps >= 0 else -1
            
            if home_direction != last_move_direction and home_steps != 0:
                home_command = home_steps + (home_direction * BACKLASH_STEPS)
            else:
                home_command = home_steps
                
            last_move_direction = home_direction
            current_yaw_deg = 0.0
            
            command_yaw_stepper(robot, home_command)
            move_servos_slowly(robot, curr_shoulder, curr_elbow, SHOULDER_REST_DEG, ELBOW_REST_DEG)
            curr_shoulder, curr_elbow = SHOULDER_REST_DEG, ELBOW_REST_DEG
            
            print("\n[FSM] Manipulation sequence complete. Continuing mobility run...")
            state = "FINAL_DRIVE"

        # ==============================================================================
        # FINAL MOBILITY SEQUENCE
        # ==============================================================================
        elif state == "FINAL_DRIVE":
            print("[FSM] Final Drive: Proceeding forward at 150mm/s for 4 seconds...")
            robot._send_body_velocity_mm(150.0, 0.0)
            time.sleep(4.0)
            robot.stop()
            
            print("[FSM] Full capstone sequence complete! Returning to IDLE.")
            show_idle_leds(robot)
            state = "IDLE"

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()