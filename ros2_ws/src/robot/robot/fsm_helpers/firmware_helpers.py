"""
User-created methods concerning robot firmware
"""
import time
import robot.hardware_map as hm
import robot.fsm_helpers.burgerbot_parameters as bp
from robot.robot import FirmwareState, Robot

ENABLE_LIDAR = True
ENABLE_GPS = False
ENABLE_VISION = True

def configure_robot(robot: Robot) -> None:
    robot.set_unit(hm.POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=hm.WHEEL_DIAMETER,
        wheel_base=hm.WHEEL_BASE,
        initial_theta_deg=hm.INITIAL_THETA_DEG,
        left_motor_id=hm.LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=hm.LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=hm.RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=hm.RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=hm.LIDAR_MOUNT_X_MM,
            y_mm=hm.LIDAR_MOUNT_Y_MM,
            theta_deg=hm.LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=hm.LIDAR_RANGE_MIN_MM,
            range_max_mm=hm.LIDAR_RANGE_MAX_MM,
            fov_deg=hm.LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(hm.TAG_ID)
        robot.set_tag_body_offset(hm.TAG_BODY_OFFSET_X_MM, hm.TAG_BODY_OFFSET_Y_MM)
        print(f"[sensor] GPS enabled — tracking ArUco tag {hm.TAG_ID}")

    if ENABLE_VISION:
        robot.enable_vision()
        print("[sensor] vision enabled — subscribing to /vision/detections")

    robot.step_set_config(
        stepper_id=hm.LIFT_STEPPER_ID,
        max_velocity=hm.LIFT_VELOCITY,
        acceleration=hm.LIFT_ACCELERATION
    )


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    kp, ki, kd = hm.WHEEL_VEL_PID
    robot.set_pid_gains(
        hm.Motor.DC_M1, hm.DCPidLoop.VELOCITY, kp, ki, kd,
        deadband_static=hm.DB_STATIC,
        deadband_kinetic=hm.DB_KINETIC,
        stop_threshold=hm.STOP_THRESHOLD
    )
    robot.set_pid_gains(
        hm.Motor.DC_M2, hm.DCPidLoop.VELOCITY, kp, ki, kd,
        deadband_static=hm.DB_STATIC,
        deadband_kinetic=hm.DB_KINETIC,
        stop_threshold=hm.STOP_THRESHOLD
    )


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)
        x, y, theta = robot.get_pose()
        print(f"POSE: ({x:.1f}, {y:.1f}, {theta:.1f})")


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(hm.LED.ORANGE, 200)
    robot.set_led(hm.LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(hm.LED.ORANGE, 0)
    robot.set_led(hm.LED.GREEN, 200)


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


def get_servo_angle(robot: Robot, servo_id: int) -> float:
    """Helper to fetch current servo angle from state (0..180)."""
    state = robot.get_servo_state()
    if not state:
        return 0.0
    for channel in state.channels:
        if channel.channel_number == servo_id:
            pulse_us = channel.pulse_us
            if pulse_us < hm.SERVO_MIN_US:
                return 0.0
            return (pulse_us - hm.SERVO_MIN_US) * 180.0 / (hm.SERVO_MAX_US - hm.SERVO_MIN_US)
    return 0.0


class FsmHandle:
    """Base class for non-blocking FSM handles."""
    def __init__(self):
        self._finished = False

    def is_done(self) -> bool:
        return self._finished

    def cancel(self) -> None:
        self._finished = True

    def wait(self, timeout: float = None) -> bool:
        """Stub for polling-based handles."""
        return self._finished


class ApproachHandle(FsmHandle):
    """
    Handle to track non-blocking approach to ingredient table.
    Completion triggered by a limit switch
    """
    def __init__(self, robot: Robot, limit_id: int):
        super().__init__()
        self._robot = robot
        self._limit_id = limit_id
        self._drive_on = False

    def is_done(self) -> bool:
        if self._finished:
            return True
        if self._robot.get_limit(self._limit_id):
            print(f"[APPROACH] — Table reached: limit {self._limit_id} triggered.")
            self._robot.stop()
            self._finished = True
        elif not self._drive_on:
            self._robot.set_velocity(bp.APPROACH_VEL_MM_S, 0.0)
            self._drive_on = True
        return self._finished

    def cancel(self) -> None:
        self._robot.stop()
        super().cancel()


class ServoHandle(FsmHandle):
    """
    Handle to track non-blocking motion progress.
    Mimics the interface of MotionHandle.
    """
    def __init__(self, robot: Robot, servo_id: int, goal_angle: float, speed: float):
        super().__init__()
        self._robot = robot
        self._servo_id = servo_id
        self._speed = speed
        self._goal_angle = max(0.0, min(180.0, goal_angle))
        self._time_last_cmd = time.monotonic()
        self._hz_cmd = 10
        
        self._current_angle = get_servo_angle(robot, servo_id)
        
        # Determine direction
        if self._goal_angle > self._current_angle:
            self._sign = 1
        elif self._goal_angle < self._current_angle:
            self._sign = -1
        else:
            self._sign = 0
            self._finished = True

        if not self._finished:
            self._robot.set_servo(self._servo_id, self._current_angle)

    def is_done(self) -> bool:
        """Returns True once goal angle deg reached."""
        if self._finished:
            return True
            
        now = time.monotonic()
        dt = now - self._time_last_cmd
        if dt >= 1.0/self._hz_cmd:
            step = self._sign * self._speed * dt
            self._current_angle += step
            self._time_last_cmd = now
            
            # Check for completion/overshoot
            if (self._sign > 0 and self._current_angle >= self._goal_angle) or \
               (self._sign < 0 and self._current_angle <= self._goal_angle) or \
               (self._sign == 0):
                self._current_angle = self._goal_angle
                self._finished = True
                
            self._robot.set_servo(self._servo_id, self._current_angle)
            
        return self._finished


class StepMoveHandle(FsmHandle):
    """
    Handle to track non-blocking stepper motion or homing progress.
    Mimics the interface of MotionHandle.
    """
    _ACTIVE_WAIT_S = 0.25  # Minimum seconds to wait before trusting an IDLE state

    def __init__(self, robot: Robot, stepper_id: int, disable_on_done: bool = True, target_position: int | None = None):
        super().__init__()
        self._robot = robot
        self._id = stepper_id
        self._disable_on_done = disable_on_done
        self._saw_active = False
        self._created_time = time.monotonic()

        if target_position is not None:
            state = self._robot.get_step_state()
            if state is not None:
                current_pos = state.steppers[self._id - 1].count
                if current_pos == target_position:
                    if self._disable_on_done:
                        self._robot.step_disable(self._id)
                    self._finished = True

    def is_done(self) -> bool:
        """Returns True once motion starts and then returns to IDLE."""
        if self._finished:
            return True

        state = self._robot.get_step_state()
        if state is None:
            return False

        # Index is 0-based
        motion_state = state.steppers[self._id - 1].motion_state

        # 1. Detect that it has actually started moving
        if motion_state != 0:  # 0 is IDLE
            self._saw_active = True

        # 2. If we never saw active, wait at least _ACTIVE_WAIT_S before
        #    trusting IDLE — guards against polling before firmware starts moving
        if not self._saw_active:
            if time.monotonic() - self._created_time >= self._ACTIVE_WAIT_S:
                self._saw_active = True  # Assume motion completed within poll gap
            else:
                return False

        # 3. Return True only if it was moving (or assumed done) and is now IDLE
        if self._saw_active and motion_state == 0:
            if self._disable_on_done:
                self._robot.step_disable(self._id)  # Safe to disable now
            self._finished = True
        return self._finished

    def cancel(self) -> None:
        """Abort motion."""
        self._robot.step_disable(self._id)
        super().cancel()


class StepHomingHandle(StepMoveHandle):
    """
    Handle to track non-blocking stepper homing progress.
    Completion triggered by returning to IDLE.
    """
    def __init__(self, robot: Robot, stepper_id: int, limit_id: int, disable_on_done: bool = True, target_position: int | None = None):
        super().__init__(robot, stepper_id, disable_on_done, target_position=target_position)
        self._limit_id = limit_id

    def is_done(self) -> bool:
        """Returns True once motion returns to IDLE."""
        if self._finished:
            return True

        # Check for completion via returning to IDLE (handled by base class).
        # We MUST NOT return True just because the limit switch is triggered,
        # as there may be a post-homing position move (backoff or target_position).
        done = super().is_done()
        if done:
            print(f"[HOMING] — Stepper homing complete.")
        return done


class ServoHomingHandle(ServoHandle):
    """
    Handle to track non-blocking homing progress.
    Mimics the interface of MotionHandle.
    """
    def __init__(self, robot: Robot, servo_id: int, limit_id: int, home_velocity: float):
        # We home to 0.0 deg
        super().__init__(robot, servo_id, goal_angle=0.0, speed=home_velocity)
        self._limit_id = limit_id
        
        # Override frequency for homing if desired (optional, 10Hz is fine)
        self._hz_cmd = hm.DEFAULT_FSM_HZ # Use faster polling for homing accuracy
        
        # Logic bug fix: if current angle is 0.0 (likely uninitialized), 
        # ServoHandle would set finished=True immediately.
        # We want to force it to move until it hits the limit switch.
        if self._current_angle == 0.0 and not self._robot.get_limit(self._limit_id):
            self._current_angle = hm.GRIPPER_HOME_DEG
            self._sign = -1
            self._finished = False
            self._robot.set_servo(self._servo_id, self._current_angle)

    def is_done(self) -> bool:
        """Returns True once limit switch is triggered or 0 deg reached."""
        if self._finished:
            return True

        if self._robot.get_limit(self._limit_id):
            print(f"[HOMING] — Gripper homing complete: limit {self._limit_id} triggered.")
            self._finished = True
            return True

        # Leverage ServoHandle's velocity-stepped movement
        was_finished = self._finished
        done = super().is_done()
        
        if done and not was_finished:
            print("[HOMING] — Gripper homing complete: 0 deg reached.")
             
        return done


def move_lift(robot: Robot, position: float, move_type: int, disable_on_done: bool = False) -> StepMoveHandle:
    """
    Starts a non-blocking stepper move for the lift.
    Returns a StepMoveHandle to poll for completion.
    """
    robot.step_move(hm.LIFT_STEPPER_ID, position, move_type, blocking=False)
    
    target_pos = int(position) if move_type == hm.StepMoveType.ABSOLUTE else None
    return StepMoveHandle(robot, hm.LIFT_STEPPER_ID, disable_on_done=disable_on_done, target_position=target_pos)


def home_lift(robot: Robot, post_position: int | None = None) -> StepHomingHandle:
    """
    Starts the non-blocking homing sequence for the lift.
    Returns a StepHomingHandle to poll for completion.
    """
    print("[HOMING] — Starting lift homing...")
    robot.step_enable(hm.LIFT_STEPPER_ID)
    robot.step_home(
        hm.LIFT_STEPPER_ID,
        direction=1,
        home_velocity=hm.LIFT_VELOCITY,
        blocking=False
    )
    return StepHomingHandle(robot, hm.LIFT_STEPPER_ID, limit_id=hm.Limit.LIM_1, disable_on_done=True, target_position=post_position)


def home_gripper(robot: Robot) -> ServoHomingHandle:
    """
    Starts the non-blocking homing sequence for the gripper.
    Returns a ServoHomingHandle to poll for completion.
    """
    print("[HOMING] — Starting gripper homing...")
    robot.enable_servo(hm.GRIPPER_SERVO_ID)
    return ServoHomingHandle(
        robot=robot,
        servo_id=hm.GRIPPER_SERVO_ID,
        limit_id=hm.GRIPPER_LIMIT,
        home_velocity=hm.GRIPPER_SPEED
    )


def set_gripper(robot: Robot, angle: float) -> ServoHandle:
    """
    Starts the non-blocking, velocity-limited gripper motion
    Returns a ServoHandle to poll for completion.
    """
    return ServoHandle(
        robot=robot,
        servo_id=hm.GRIPPER_SERVO_ID,
        goal_angle=angle,
        speed=hm.GRIPPER_SPEED
    )


def approach_ingredient_table(robot: Robot) -> ApproachHandle:
    """
    Starts the non-blocking approach to the ingredient table.
    Returns an ApproachHandle to poll for completion.
    """
    return ApproachHandle(
        robot=robot,
        limit_id=hm.TABLE_LIMIT
    )