from __future__ import annotations

from collections.abc import Callable
import math
import time
import threading

from rclpy.node import Node

import time as _time

from robot.sensor_fusion import GpsTangentOrientationFusion, OrientationComplementaryFilter, PositionComplementaryFilter, SensorFusion
try:
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except (ImportError, ModuleNotFoundError):
    class ReliabilityPolicy:
        BEST_EFFORT = "best_effort"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    class QoSProfile:
        def __init__(self, reliability=None, history=None, depth=1) -> None:
            self.reliability = reliability
            self.history = history
            self.depth = depth

import numpy as np

from bridge_interfaces.msg import (
    DCEnable,
    DCHome,
    DCPid,
    DCPidReq,
    DCPidSet,
    DCResetPosition,
    DCSetPosition,
    DCSetPwm,
    DCSetVelocity,
    DCStateAll,
    IOSetLed,
    IOSetNeopixel,
    IOOutputState,
    SensorKinematics,
    ServoEnable,
    ServoSet,
    ServoStateAll,
    StepConfig,
    StepConfigReq,
    StepConfigSet,
    StepEnable,
    StepHome,
    StepMove,
    StepStateAll,
    SysOdomParamReq,
    SysOdomParamRsp,
    SysOdomParamSet,
    SysOdomReset,
    SystemConfig,
    SystemDiag,
    SystemInfo,
    SystemPower,
    SystemState,
    IOInputState,
    FusedPose,
    LidarWorldPoints,
    SensorImu,
    TagDetectionArray,
    TrackedObstacleArray,
    VirtualTarget,
)
try:
    from bridge_interfaces.msg import VisionDetectionArray
except (ImportError, ModuleNotFoundError):
    class VisionDetectionArray:  # test-stub fallback
        detections = ()
        image_width = 0
        image_height = 0

from bridge_interfaces.srv import SetFirmwareState
try:
    from sensor_msgs.msg import LaserScan
except (ImportError, ModuleNotFoundError):
    class LaserScan:  # test-stub fallback
        ranges = ()
        angle_min = 0.0
        angle_max = 0.0
        angle_increment = 0.0
        range_min = 0.0
        range_max = 0.0

from robot.hardware_map import (
    BUTTON_COUNT,
    DCMotorMode,
    DCPidLoop,
    DEFAULT_NAV_HZ,
    LEDMode,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LIMIT_COUNT,
    INITIAL_THETA_DEG as HW_INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED as HW_LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR as HW_LEFT_WHEEL_MOTOR,
    Motor,
    RIGHT_WHEEL_DIR_INVERTED as HW_RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR as HW_RIGHT_WHEEL_MOTOR,
    StepMoveType,
    StepperMotionState,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    Unit,
    WHEEL_BASE as HW_WHEEL_BASE,
    WHEEL_DIAMETER as HW_WHEEL_DIAMETER,
)

from robot.robot_impl.hardware import FirmwareState, HardwareMixin
from robot.obstacle_tracking import ObstacleTracker
from robot.robot_impl.sensors import SensorsMixin
from robot.robot_impl.navigation import MotionHandle, NavigationMixin
from robot.robot_impl.legacy import LegacyMixin


# =============================================================================
# Robot
# =============================================================================

class Robot(HardwareMixin, SensorsMixin, NavigationMixin, LegacyMixin):
    """
    Layer-1 abstraction over all bridge ROS topics and services.

    Construct this inside a running ROS node and pass the node reference in.
    All length/velocity inputs and outputs respect the unit system set at
    construction time (default: mm). Angles are always in degrees for the
    public API; internally radians are used.

    Motor IDs are 1-based (1–4). Stepper IDs are 1-based (1–4).
    Servo channels are 1-based (1–16).
    Button IDs are 1-based (1–10). Limit IDs are 1-based (1–8).

    Sensor subscriptions are opt-in — call the relevant enable_*() method
    before using the corresponding API:
        robot.enable_lidar()   — subscribe to /scan, populate get_obstacles()
        robot.enable_gps()     — subscribe to /tag_detections, populate GPS fusion
        robot.enable_imu()     — subscribe to /sensor_imu, populate orientation fusion
        robot.enable_vision()  — subscribe to /vision/detections, populate get_detections()

    Hardware defaults match robot/hardware_map.py and firmware/arduino/src/config.h:
        WHEEL_DIAMETER_MM = 74.0
        WHEEL_BASE_MM     = 333.0
        ENCODER_PPR       = 1440  (4× mode)
        DEFAULT_LEFT_WHEEL_MOTOR  = 1
        DEFAULT_RIGHT_WHEEL_MOTOR = 2
        INITIAL_THETA_DEG = 90.0
        IMU_Z_DOWN        = False  (Z-axis points up)
    """

    WHEEL_DIAMETER_MM: float = HW_WHEEL_DIAMETER
    WHEEL_BASE_MM:     float = HW_WHEEL_BASE
    ENCODER_PPR:       int   = 1440
    INITIAL_THETA_DEG: float = HW_INITIAL_THETA_DEG
    IMU_Z_DOWN:        bool  = False
    DEFAULT_LEFT_WHEEL_MOTOR:  int = int(HW_LEFT_WHEEL_MOTOR)
    DEFAULT_RIGHT_WHEEL_MOTOR: int = int(HW_RIGHT_WHEEL_MOTOR)
    DEFAULT_LEFT_WHEEL_DIR_INVERTED:  bool = HW_LEFT_WHEEL_DIR_INVERTED
    DEFAULT_RIGHT_WHEEL_DIR_INVERTED: bool = HW_RIGHT_WHEEL_DIR_INVERTED
    POSITION_ALPHA    = 0.05   # complementary filter GPS weight for position fusion
    ORIENTATION_ALPHA = 0.0    # complementary filter IMU weight for orientation fusion
    TAG_X_OFFSET_MM   = TAG_BODY_OFFSET_X_MM   # ArUco tag x in robot body frame (mm, +x = forward)
    TAG_Y_OFFSET_MM   = TAG_BODY_OFFSET_Y_MM   # ArUco tag y in robot body frame (mm, +y = left)

    LIDAR_MOUNT_X_MM:      float = LIDAR_MOUNT_X_MM      # lidar x in robot body frame (mm, +x = forward)
    LIDAR_MOUNT_Y_MM:      float = LIDAR_MOUNT_Y_MM      # lidar y in robot body frame (mm, +y = left)
    LIDAR_MOUNT_THETA_DEG: float = LIDAR_MOUNT_THETA_DEG # lidar heading offset relative to robot forward (CCW+)

    LIDAR_RANGE_MIN_MM:  float = LIDAR_RANGE_MIN_MM  # discard lidar returns closer than this (mm)
    LIDAR_RANGE_MAX_MM:  float = LIDAR_RANGE_MAX_MM  # discard lidar returns farther than this (mm)
    LIDAR_FOV_MIN_DEG:   float = LIDAR_FOV_DEG[0]    # FOV window min (deg, 0° = robot +x forward, CCW+)
    LIDAR_FOV_MAX_DEG:   float = LIDAR_FOV_DEG[1]    # FOV window max

    OBSTACLE_TRACK_INPUT_RANGE_MM: float = 1200.0
    OBSTACLE_TRACK_MAX_INPUT_POINTS: int = 250
    OBSTACLE_TRACK_CLUSTER_NEIGHBOR_MM: float = 90.0
    OBSTACLE_TRACK_CLUSTER_MIN_POINTS: int = 3
    OBSTACLE_TRACK_MAX_DISK_RADIUS_MM: float = 75.0
    OBSTACLE_TRACK_RADIUS_MARGIN_MM: float = 20.0
    OBSTACLE_TRACK_ASSOCIATION_DIST_MM: float = 180.0
    OBSTACLE_TRACK_TTL_S: float = 1.5
    OBSTACLE_TRACK_MIN_HITS_TO_CONFIRM: int = 2
    OBSTACLE_TRACK_MAX_TRACKS: int = 12
    APF_MAX_PLANNER_TRACKS: int = 6
    APF_TRACK_INPUT_MARGIN_MM: float = 150.0
    LAPF_MAX_PLANNER_TRACKS: int = 6
    LAPF_LEASH_LENGTH_MM: float = 400.0
    LAPF_LEASH_HALF_ANGLE_DEG: float = 60.0
    LAPF_TARGET_SPEED_MM_S: float = 200.0
    LAPF_REPULSION_RANGE_MM: float = 700.0
    LAPF_REPULSION_GAIN: float = 800.0
    LAPF_ATTRACTION_GAIN: float = 1.0
    LAPF_FORCE_EMA_ALPHA: float = 0.35
    LAPF_INFLATION_MARGIN_MM: float = 200.0

    _SERVO_MIN_US: int = 1000
    _SERVO_MAX_US: int = 2000
    _SHUTDOWN_SETTLE_S: float = 0.10

    def __init__(
        self,
        node: Node,
        unit: Unit = Unit.MM,
        wheel_diameter_mm: float = WHEEL_DIAMETER_MM,
        wheel_base_mm: float = WHEEL_BASE_MM,
        encoder_ppr: int = ENCODER_PPR,
    ) -> None:
        self._node             = node
        self._unit             = unit
        self._wheel_diameter   = wheel_diameter_mm
        self._wheel_base       = wheel_base_mm
        self._encoder_ppr      = encoder_ppr
        self._ticks_per_mm     = encoder_ppr / (math.pi * wheel_diameter_mm)
        self._initial_theta_deg = self.INITIAL_THETA_DEG
        self._left_wheel_motor  = self.DEFAULT_LEFT_WHEEL_MOTOR
        self._right_wheel_motor = self.DEFAULT_RIGHT_WHEEL_MOTOR
        self._left_wheel_dir_inverted  = self.DEFAULT_LEFT_WHEEL_DIR_INVERTED
        self._right_wheel_dir_inverted = self.DEFAULT_RIGHT_WHEEL_DIR_INVERTED
        self._odom_user_configured = False
        self._odom_confirm_event = threading.Event()
        self._lock             = threading.Lock()

        # ── Cached firmware state ─────────────────────────────────────────────
        self._sys_state:  int            = 0
        self._sys_power:  SystemPower    = None
        self._sys_info:   SystemInfo     = None
        self._sys_config: SystemConfig   = None
        self._sys_diag:   SystemDiag     = None
        self._dc_state:   DCStateAll     = None
        self._dc_pid_cache: dict[tuple[int, int], DCPid] = {}
        self._step_state: StepStateAll   = None
        self._step_config_cache: dict[int, StepConfig] = {}
        self._servo_state: ServoStateAll = None
        self._io_output_state: IOOutputState = None
        self._imu:        SensorImu      = None
        self._imu_z_down:         bool        = self.IMU_Z_DOWN
        self._ahrs_heading:       float | None = None
        self._odom_reset_pending: bool       = False
        self._fused_theta:        float      = math.radians(self.INITIAL_THETA_DEG)
        self._orientation_fusion: OrientationComplementaryFilter = OrientationComplementaryFilter(alpha=self.ORIENTATION_ALPHA)
        self._fusion: SensorFusion = self._orientation_fusion
        self._pose:    tuple = (0.0, 0.0, 0.0)   # x_mm, y_mm, theta_rad (raw odometry)

        # ── GPS position fusion ───────────────────────────────────────────────
        self._tracked_tag_id:      int   = -1
        self._gps_x_mm:            float = 0.0
        self._gps_y_mm:            float = 0.0
        self._gps_last_time:       float = 0.0
        self._gps_timeout_s:       float = 1.0
        self._gps_offset_x_mm:     float = 0.0
        self._gps_offset_y_mm:     float = 0.0
        self._tag_body_offset_x_mm: float = self.TAG_X_OFFSET_MM
        self._tag_body_offset_y_mm: float = self.TAG_Y_OFFSET_MM
        self._gps_paused:          bool  = False
        self._gps_subscribed:      bool  = False

        # ── Lidar ─────────────────────────────────────────────────────────────
        self._lidar_mount_x_mm:      float = self.LIDAR_MOUNT_X_MM
        self._lidar_mount_y_mm:      float = self.LIDAR_MOUNT_Y_MM
        self._lidar_mount_theta_rad: float = math.radians(self.LIDAR_MOUNT_THETA_DEG)
        self._lidar_range_min_mm:    float = self.LIDAR_RANGE_MIN_MM
        self._lidar_range_max_mm:    float = self.LIDAR_RANGE_MAX_MM
        self._lidar_fov_min_rad:     float = math.radians(self.LIDAR_FOV_MIN_DEG)
        self._lidar_fov_max_rad:     float = math.radians(self.LIDAR_FOV_MAX_DEG)
        self._lidar_world_pub        = None   # set by start_lidar_world_publisher
        self._lidar_world_timer      = None   # ROS timer, fires on spin thread
        self._obstacle_tracker = ObstacleTracker(
            cluster_neighbor_mm=self.OBSTACLE_TRACK_CLUSTER_NEIGHBOR_MM,
            cluster_min_points=self.OBSTACLE_TRACK_CLUSTER_MIN_POINTS,
            max_disk_radius_mm=self.OBSTACLE_TRACK_MAX_DISK_RADIUS_MM,
            disk_radius_margin_mm=self.OBSTACLE_TRACK_RADIUS_MARGIN_MM,
            association_dist_mm=self.OBSTACLE_TRACK_ASSOCIATION_DIST_MM,
            ttl_s=self.OBSTACLE_TRACK_TTL_S,
            min_hits_to_confirm=self.OBSTACLE_TRACK_MIN_HITS_TO_CONFIRM,
            max_tracks=self.OBSTACLE_TRACK_MAX_TRACKS,
        )
        self._obstacle_tracks = []

        # ── Fused position ────────────────────────────────────────────────────
        self._fused_x_mm:    float = 0.0
        self._fused_y_mm:    float = 0.0
        self._fused_pose_available: bool = False
        self._pos_fusion:    PositionComplementaryFilter = PositionComplementaryFilter(alpha=self.POSITION_ALPHA)
        self._vel:     tuple = (0.0, 0.0, 0.0)   # vx_mm_s, vy_mm_s, vtheta_rad_s

        # ── IO ────────────────────────────────────────────────────────────────
        self._buttons: int  = 0
        self._limits:  int  = 0
        self._button_edges: int  = 0
        self._limit_edges:  int  = 0
        self._have_io_input: bool = False

        # ── Obstacles / vision ────────────────────────────────────────────────
        self._obstacles_mm: np.ndarray = np.float64([])
        self._obstacle_provider: Callable[[], list[tuple[float, float]]] | None = None
        self._obstacle_avoidance_planner = None   # legacy
        self._virtual_target_mm: tuple[float, float] | None = None
        self._vision_detections: list[dict[str, object]] = []
        self._vision_image_size: tuple[int, int] = (0, 0)
        self._vision_last_time: float = 0.0

        # ── Trajectory ────────────────────────────────────────────────────────
        self._odom_traj:  list[tuple[float, float]] = []
        self._fused_traj: list[tuple[float, float]] = []

        # ── Events ────────────────────────────────────────────────────────────
        self._pose_event:       threading.Event = threading.Event()
        self._odom_reset_event: threading.Event = threading.Event()
        self._button_events: dict[int, threading.Event] = {}
        self._limit_events:  dict[int, threading.Event] = {}

        # ── Navigation ────────────────────────────────────────────────────────
        self._nav_lock:   threading.Lock   = threading.Lock()
        self._nav_thread: threading.Thread = None
        self._nav_cancel: threading.Event  = threading.Event()
        self._nav_done:   threading.Event  = threading.Event()

        # ── Publishers ────────────────────────────────────────────────────────
        self._dc_vel_pub   = node.create_publisher(DCSetVelocity,  '/dc_set_velocity',  10)
        self._dc_pwm_pub   = node.create_publisher(DCSetPwm,       '/dc_set_pwm',       10)
        self._dc_pos_pub   = node.create_publisher(DCSetPosition,  '/dc_set_position',  10)
        self._dc_en_pub    = node.create_publisher(DCEnable,       '/dc_enable',        10)
        self._dc_home_pub  = node.create_publisher(DCHome,         '/dc_home',          10)
        self._dc_rst_pub   = node.create_publisher(DCResetPosition,'/dc_reset_position',10)
        self._dc_pid_set   = node.create_publisher(DCPidSet,       '/dc_pid_set',       10)
        self._dc_pid_req   = node.create_publisher(DCPidReq,       '/dc_pid_req',       10)
        self._step_en_pub  = node.create_publisher(StepEnable,     '/step_enable',      10)
        self._step_mv_pub  = node.create_publisher(StepMove,       '/step_move',        10)
        self._step_hm_pub  = node.create_publisher(StepHome,       '/step_home',        10)
        self._step_cfg_req_pub = node.create_publisher(StepConfigReq, '/step_config_req', 10)
        self._step_cfg_pub = node.create_publisher(StepConfigSet,  '/step_config_set',  10)
        self._srv_en_pub   = node.create_publisher(ServoEnable,    '/servo_enable',     10)
        self._srv_set_pub  = node.create_publisher(ServoSet,       '/servo_set',        10)
        self._led_pub      = node.create_publisher(IOSetLed,       '/io_set_led',       10)
        self._neo_pub      = node.create_publisher(IOSetNeopixel,  '/io_set_neopixel',  10)
        self._odom_param_req_pub = node.create_publisher(SysOdomParamReq, '/sys_odom_param_req', 10)
        self._odom_param_pub     = node.create_publisher(SysOdomParamSet, '/sys_odom_param_set', 10)
        self._odom_pub     = node.create_publisher(SysOdomReset,   '/sys_odom_reset',   10)
        self._pub_fused_pose = node.create_publisher(FusedPose,    '/fused_pose',       10)
        self._pub_obstacle_tracks = node.create_publisher(TrackedObstacleArray, '/obstacle_tracks', 10)
        self._pub_virtual_target = node.create_publisher(VirtualTarget, '/virtual_target', 10)

        # ── Always-on subscriptions ───────────────────────────────────────────
        node.create_subscription(SystemState,      '/sys_state',          self._on_sys_state,      10)
        node.create_subscription(SystemPower,      '/sys_power',          self._on_sys_power,      10)
        node.create_subscription(SystemInfo,       '/sys_info_rsp',       self._on_sys_info,       10)
        node.create_subscription(SystemConfig,     '/sys_config_rsp',     self._on_sys_config,     10)
        node.create_subscription(SystemDiag,       '/sys_diag_rsp',       self._on_sys_diag,       10)
        node.create_subscription(DCPid,            '/dc_pid_rsp',         self._on_dc_pid,         10)
        node.create_subscription(DCStateAll,       '/dc_state_all',       self._on_dc_state,       10)
        node.create_subscription(StepConfig,       '/step_config_rsp',    self._on_step_config,    10)
        node.create_subscription(StepStateAll,     '/step_state_all',     self._on_step_state,     10)
        node.create_subscription(ServoStateAll,    '/servo_state_all',    self._on_servo_state,    10)
        node.create_subscription(SensorKinematics, '/sensor_kinematics',  self._on_kinematics,     10)
        node.create_subscription(IOInputState,     '/io_input_state',     self._on_io_input,       10)
        node.create_subscription(IOOutputState,    '/io_output_state',    self._on_io_output,      10)
        node.create_subscription(SysOdomParamRsp,  '/sys_odom_param_rsp', self._on_odom_param_rsp, 10)

        # Sensor subscriptions are opt-in — call enable_lidar(), enable_gps(),
        # enable_imu(), or enable_vision() to activate them.

        # ── Service clients ───────────────────────────────────────────────────
        self._set_state_client = node.create_client(SetFirmwareState, '/set_firmware_state')

        # Sync local odometry cache with live firmware snapshot.
        self.request_odometry_parameters()

    # =========================================================================
    # Subscription callbacks  (ROS spin thread)
    # =========================================================================

    def _on_sys_state(self, msg: SystemState) -> None:
        with self._lock:
            self._sys_state = msg.state

    def _on_sys_power(self, msg: SystemPower) -> None:
        with self._lock:
            self._sys_power = msg

    def _on_sys_info(self, msg: SystemInfo) -> None:
        with self._lock:
            self._sys_info = msg

    def _on_sys_config(self, msg: SystemConfig) -> None:
        with self._lock:
            self._sys_config = msg

    def _on_sys_diag(self, msg: SystemDiag) -> None:
        with self._lock:
            self._sys_diag = msg

    def _on_dc_pid(self, msg: DCPid) -> None:
        with self._lock:
            self._dc_pid_cache[(int(msg.motor_number), int(msg.loop_type))] = msg

    def _on_dc_state(self, msg: DCStateAll) -> None:
        with self._lock:
            self._dc_state = msg

    def _on_step_config(self, msg: StepConfig) -> None:
        with self._lock:
            self._step_config_cache[int(msg.stepper_number)] = msg

    def _on_step_state(self, msg: StepStateAll) -> None:
        with self._lock:
            self._step_state = msg

    def _on_servo_state(self, msg: ServoStateAll) -> None:
        with self._lock:
            self._servo_state = msg

    def _on_imu(self, msg: SensorImu) -> None:
        with self._lock:
            self._imu = msg
            if msg.mag_calibrated:
                # TODO: use the gravity vector from the AMRS to allow mouting of the IMU in any orientation, not just flat with Z up or down.  This would also allow auto-detection of the Z-down configuration without needing a user-set flag.
                
                # Use the Madgwick AHRS quaternion (accel + gyro + mag, 9-axis).
                # The firmware runs the Fusion library AHRS which integrates all
                # three sensor axes and applies gyroscope bias correction; the
                # resulting yaw is a tilt-compensated, drift-corrected heading.
                # Convention: NWU (North-West-Up), yaw via ZYX Euler extraction.
                qw = float(msg.quat_w)
                qx = float(msg.quat_x)
                qy = float(msg.quat_y)
                qz = float(msg.quat_z)
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz),
                )
                # When the sensor is mounted upside-down (Z toward ground) the
                # AHRS converges to an inverted-attitude quaternion, making the
                # extracted yaw the negative of the physical heading.
                self._ahrs_heading = -yaw if self._imu_z_down else yaw
            else:
                # Calibration lost or not yet acquired — clear the heading so
                # _on_kinematics falls back to pure odometry rather than
                # continuing to fuse against a stale absolute reference.
                # Also clear _ahrs_prev_wrapped so that when calibration is
                # regained the accumulator re-seeds from odometry rather than
                # computing a delta from a pre-loss value (which may have
                # jumped when the Madgwick filter reinitialised).
                self._ahrs_heading = None

    def _on_kinematics(self, msg: SensorKinematics) -> None:
        with self._lock:
            self._pose = (msg.x, msg.y, msg.theta)
            self._vel  = (msg.vx, msg.vy, msg.v_theta)

            # Orientation fusion: delegate to the active strategy.
            _initial_theta_rad = math.radians(self._initial_theta_deg)
            if self._ahrs_heading is not None:
                if self._odom_reset_pending:
                    if abs(math.atan2(
                        math.sin(msg.theta - _initial_theta_rad),
                        math.cos(msg.theta - _initial_theta_rad),
                    )) < math.radians(5.0):
                        self._odom_reset_pending = False
                        self._odom_reset_event.set()
                    relative_ahrs = None
                else:
                    relative_ahrs = self._ahrs_heading
            else:
                if self._odom_reset_pending:
                    if abs(math.atan2(
                        math.sin(msg.theta - _initial_theta_rad),
                        math.cos(msg.theta - _initial_theta_rad),
                    )) < math.radians(5.0):
                        self._odom_reset_pending = False
                        self._odom_reset_event.set()
                relative_ahrs = None
            linear_vel  = math.hypot(float(msg.vx), float(msg.vy))
            angular_vel = float(msg.v_theta)
            self._fused_theta = self._orientation_fusion.update(
                odom_theta  = msg.theta,
                mag_heading = relative_ahrs,
                linear_vel  = linear_vel,
                angular_vel = angular_vel,
            )

            # Position fusion: complementary filter blending odometry with GPS.
            # When GPS is available, anchor the absolute position and update the
            # dead-reckoning baseline. When GPS is stale, project forward from the
            # last anchor using the odometry delta to avoid raw-odometry drift.
            gps_fresh = (
                self._gps_last_time > 0.0
                and (_time.monotonic() - self._gps_last_time) < self._gps_timeout_s
            )
            gps_x = self._gps_x_mm if gps_fresh else None
            gps_y = self._gps_y_mm if gps_fresh else None
            self._fused_x_mm, self._fused_y_mm = self._pos_fusion.update(
                msg.x, msg.y, gps_x, gps_y
            )
            _raw_odom = (float(msg.x), float(msg.y))
            _raw_fused = (self._fused_x_mm, self._fused_y_mm)

        self._odom_traj.append(_raw_odom)
        self._fused_traj.append(_raw_fused)
        # self._node.get_logger().info(
        #     f"odom=({_raw_odom[0]:.1f}, {_raw_odom[1]:.1f}) mm, θ ={float(math.degrees(msg.theta)):5.1f}°",
        #     # f"fused=({_raw_fused[0]:.1f}, {_raw_fused[1]:.1f}) mm",
        #     throttle_duration_sec=0.5,
        # )
        self._pose_event.set()
        self._pose_event.clear()

    def _on_tag_detections(self, msg: TagDetectionArray) -> None:
        """Cache GPS position from the tracked ArUco tag (metres → mm, arena frame)."""
        for det in msg.detections:
            if self._tracked_tag_id == -1 or det.tag_id == self._tracked_tag_id:
                with self._lock:
                    warn_offset = (
                        self._gps_offset_x_mm == 0.0 and self._gps_offset_y_mm == 0.0
                    )
                    # Update GPS state unconditionally — the warning is advisory only
                    # and must not gate the position update.
                    tag_x = float(det.x) * 1000.0 + self._gps_offset_x_mm
                    tag_y = float(det.y) * 1000.0 + self._gps_offset_y_mm
                    # Correct for tag not being at the robot body origin.
                    # Rotate the body-frame tag offset into arena frame and subtract
                    # so _gps_x/y_mm reflect the robot centre, not the tag centre.
                    self._gps_x_mm      = tag_x - self._tag_body_offset_x_mm 
                    self._gps_y_mm      = tag_y - self._tag_body_offset_y_mm
                    self._gps_last_time = _time.monotonic()
                # Log outside the lock so a slow or failing logger call cannot
                # block kinematics callbacks that also acquire the lock.
                if warn_offset:
                    self._node.get_logger().warn(
                        'GPS offset is (0, 0) — arena-frame correction has not been '
                        'configured. Call set_gps_offset() with the measured offset '
                        'between the GPS frame and the arena corner origin.',
                        throttle_duration_sec=10.0,
                    )
                break

    def _on_io_input(self, msg: IOInputState) -> None:
        with self._lock:
            prev_btn = self._buttons if self._have_io_input else msg.button_mask
            prev_lim = self._limits if self._have_io_input else msg.limit_mask
            self._buttons = msg.button_mask
            self._limits  = msg.limit_mask
            self._button_edges |= msg.button_mask & ~prev_btn
            self._limit_edges |= msg.limit_mask & ~prev_lim
            self._have_io_input = True

        # Fire events on rising edges (outside lock to avoid deadlock)
        for bit in range(16):
            bid = bit + 1
            if ((msg.button_mask >> bit) & 1) and not ((prev_btn >> bit) & 1):
                ev = self._button_events.get(bid)
                if ev:
                    ev.set()
            if ((msg.limit_mask >> bit) & 1) and not ((prev_lim >> bit) & 1):
                ev = self._limit_events.get(bid)
                if ev:
                    ev.set()

    def _on_io_output(self, msg: IOOutputState) -> None:
        with self._lock:
            self._io_output_state = msg

    def _on_odom_param_rsp(self, msg: SysOdomParamRsp) -> None:
        self._apply_odom_param_snapshot(
            float(msg.wheel_diameter_mm),
            float(msg.wheel_base_mm),
            float(msg.initial_theta_deg),
            int(msg.left_motor_number),
            bool(msg.left_motor_dir_inverted),
            int(msg.right_motor_number),
            bool(msg.right_motor_dir_inverted),
        )

    def _on_lidar(self, msg: LaserScan) -> None:
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert polar to Cartesian (robot frame, metres -> mm)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        with self._lock:
            self._obstacles_mm = np.float64(
                np.concatenate([x[:, np.newaxis], y[:, np.newaxis]], axis=1)
            ) * 1000.0

    # =========================================================================
    # System
    # =========================================================================

    def get_state(self) -> int:
        """Return cached firmware state. Compare to FirmwareState enum."""
        with self._lock:
            return self._sys_state

    def set_state(self, state: FirmwareState, timeout: float = 5.0) -> bool:
        """
        Request a firmware state transition via the /set_firmware_state service.
        Blocks until the transition completes or timeout expires.
        Safe to call from any thread.
        """
        if not self._set_state_client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().error('set_firmware_state service not available')
            return False
        req = SetFirmwareState.Request()
        req.target_state = int(state)
        req.timeout_sec  = float(timeout)
        future = self._set_state_client.call_async(req)

        # ROS spins in a background thread; wait with a threading.Event
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(timeout=timeout + 1.0)
        if not future.done():
            return False
        result = future.result()
        return result is not None and result.success

    def estop(self) -> bool:
        """Immediately transition firmware to ESTOP."""
        return self.set_state(FirmwareState.ESTOP)

    def reset_estop(self) -> bool:
        """Clear ESTOP and return to IDLE."""
        return self.set_state(FirmwareState.IDLE)

    def reset_odometry(self) -> None:
        """Reset firmware odometry pose to (0, 0, initial_theta).

        Captures the current AHRS heading as the new zero reference and
        initialises the relative-AHRS accumulator to match the firmware's
        initial_theta so that the fusion filter sees no spurious discrepancy
        between odometry and AHRS heading immediately after reset.

        After this call, get_fused_orientation() and get_pose()[2] will both
        report initial_theta (default 90°), not 0°.
        """
        with self._lock:
            # Do NOT capture _ahrs_heading_offset here.  The firmware will
            # only process this reset after UART round-trip latency (~5-40 ms).
            # Capturing the AHRS heading now and blending it with the
            # not-yet-reset odometry produces a misaligned zero reference.
            #
            # Instead, set _odom_reset_pending = True so that _on_kinematics
            # captures the AHRS offset on the first tick where msg.theta has
            # already snapped back to initial_theta (i.e. firmware confirmed
            # the reset).  That way both zero references share exactly the
            # same physical timestamp.
            self._odom_reset_pending = True
            self._odom_reset_event.clear()
            self._pos_fusion.reset()
        msg = SysOdomReset()
        msg.flags = 0
        self._odom_pub.publish(msg)

    def set_wheel_diameter_mm(self, wheel_diameter_mm: float) -> None:
        """Update wheel diameter in explicit millimeters."""
        self._update_odometry_params(wheel_diameter_mm=wheel_diameter_mm)

    def set_wheel_base_mm(self, wheel_base_mm: float) -> None:
        """Update wheel base in explicit millimeters."""
        self._update_odometry_params(wheel_base_mm=wheel_base_mm)

    def set_initial_theta(self, theta_deg: float) -> None:
        """
        Update the heading used by future odometry resets.

        This does not overwrite the current live pose. Call reset_odometry()
        after setting it if you want the new heading applied immediately.
        """
        self._update_odometry_params(initial_theta_deg=theta_deg)

    def set_odom_left_motor(self, motor_id: int) -> None:
        """Select which DC motor is treated as the left wheel for odometry and body motion."""
        self._update_odometry_params(left_wheel_motor=motor_id)

    def set_odom_right_motor(self, motor_id: int) -> None:
        """Select which DC motor is treated as the right wheel for odometry and body motion."""
        self._update_odometry_params(right_wheel_motor=motor_id)

    def set_odom_motors(self, left_motor_id: int, right_motor_id: int) -> None:
        """Atomically configure both odometry wheel motors."""
        self._update_odometry_params(
            left_wheel_motor=left_motor_id,
            right_wheel_motor=right_motor_id,
        )

    def set_odom_left_motor_dir_inverted(self, inverted: bool) -> None:
        """Configure whether positive left-wheel odometry counts mean reverse motion."""
        self._update_odometry_params(left_wheel_dir_inverted=inverted)

    def set_odom_right_motor_dir_inverted(self, inverted: bool) -> None:
        """Configure whether positive right-wheel odometry counts mean reverse motion."""
        self._update_odometry_params(right_wheel_dir_inverted=inverted)

    def set_odometry_parameters(
        self,
        *,
        wheel_diameter: float | None = None,
        wheel_base: float | None = None,
        initial_theta_deg: float | None = None,
        left_motor_id: int | None = None,
        left_motor_dir_inverted: bool | None = None,
        right_motor_id: int | None = None,
        right_motor_dir_inverted: bool | None = None,
    ) -> None:
        """
        Publish a full odometry-parameter snapshot to firmware in one call.

        wheel_diameter and wheel_base are in the current user unit system.
        Use set_wheel_diameter_mm() / set_wheel_base_mm() when you need to
        work in explicit raw millimeters instead.
        """
        scale = self._unit.value
        self._update_odometry_params(
            wheel_diameter_mm=None if wheel_diameter is None else float(wheel_diameter) * scale,
            wheel_base_mm=None if wheel_base is None else float(wheel_base) * scale,
            initial_theta_deg=initial_theta_deg,
            left_wheel_motor=left_motor_id,
            left_wheel_dir_inverted=left_motor_dir_inverted,
            right_wheel_motor=right_motor_id,
            right_wheel_dir_inverted=right_motor_dir_inverted,
        )

    def request_odometry_parameters(self) -> None:
        """Request the current firmware odometry parameter snapshot."""
        msg = SysOdomParamReq()
        msg.target = 0xFF
        self._odom_param_req_pub.publish(msg)

    def get_odometry_parameters(self) -> dict[str, float | int | bool]:
        """Return the latest local odometry parameter snapshot in firmware-native millimeters."""
        with self._lock:
            return {
                "wheel_diameter_mm": self._wheel_diameter,
                "wheel_base_mm": self._wheel_base,
                "initial_theta_deg": self._initial_theta_deg,
                "left_motor_number": self._left_wheel_motor,
                "left_motor_dir_inverted": self._left_wheel_dir_inverted,
                "right_motor_number": self._right_wheel_motor,
                "right_motor_dir_inverted": self._right_wheel_dir_inverted,
            }

    def get_power(self) -> SystemPower:
        """Return cached power state (battery_mv, rail_5v_mv, servo_rail_mv)."""
        with self._lock:
            return self._sys_power

    def get_system_info(self) -> SystemInfo:
        """Return cached system information from /sys_info_rsp."""
        with self._lock:
            return self._sys_info

    def get_system_config(self) -> SystemConfig:
        """Return cached system configuration from /sys_config_rsp."""
        with self._lock:
            return self._sys_config

    def get_system_diag(self) -> SystemDiag:
        """Return cached system diagnostics from /sys_diag_rsp."""
        with self._lock:
            return self._sys_diag

    # =========================================================================
    # Pose / odometry
    # =========================================================================

    def get_pose(self) -> tuple[float, float, float]:
        """
        Return the current pose as (x, y, theta_deg) in user units and degrees.

        x and y are GPS-fused when a recent GPS fix is available, otherwise
        raw wheel-odometry position.
        theta is the sensor-fused heading (AHRS quaternion blended with wheel
        odometry via the active fusion strategy). This is the same value
        returned by get_fused_orientation().
        """
        x_mm, y_mm, theta_rad = self._get_pose_mm()
        s = self._unit.value
        return x_mm / s, y_mm / s, math.degrees(theta_rad)

    def get_velocity(self) -> tuple[float, float, float]:
        """Return (vx, vy, v_theta_deg_s) in user units/s and degrees/s."""
        with self._lock:
            vx, vy, vt = self._vel
        s = self._unit.value
        return vx / s, vy / s, math.degrees(vt)

    def wait_for_pose_update(self, timeout: float = None) -> bool:
        """Block until the next /sensor_kinematics message arrives (~25 Hz)."""
        return self._pose_event.wait(timeout=timeout)

    def wait_for_odometry_reset(self, timeout: float = 2.0) -> bool:
        """Block until the firmware confirms the odometry reset (theta ≈ initial_theta).

        Call this after reset_odometry() instead of multiple wait_for_pose_update()
        calls to guarantee that the initial-heading capture has happened before
        reading get_pose() or get_fused_orientation().

        Returns True if the reset was confirmed within *timeout* seconds, False
        if it timed out (the firmware may not have responded).
        """
        return self._odom_reset_event.wait(timeout=timeout)

    def set_obstacles(self, obstacles: list[tuple[float, float]]) -> None:
        """
        Cache APF obstacles in the robot frame using the current user length unit.

        Each obstacle is (x_forward, y_left) relative to the robot. Future lidar
        code can either call this repeatedly with fresh detections or install a
        live provider with set_obstacle_provider().
        """
        converted = np.float64([
            (
                self._require_finite_float("obstacle_x", obs_x) * self._unit.value,
                self._require_finite_float("obstacle_y", obs_y) * self._unit.value,
            )
            for obs_x, obs_y in obstacles
        ])
        with self._lock:
            self._obstacles_mm = converted

    def clear_obstacles(self) -> None:
        """Clear the cached APF obstacle list set by set_obstacles()."""
        with self._lock:
            self._obstacles_mm = np.float64([])

    def get_obstacles(self) -> list[tuple[float, float]]:
        """Return the current APF obstacles in the current user length unit."""
        scale = self._unit.value
        return [(x_mm / scale, y_mm / scale) for x_mm, y_mm in self._get_obstacles_mm()]

    def set_obstacle_provider(
        self,
        provider: Callable[[], list[tuple[float, float]]] | None,
    ) -> None:
        """
        Register a live APF obstacle callback that returns robot-frame positions in mm.

        TODO: a future lidar/object-detection node can install a provider here so
        APF follows the latest obstacle snapshot without changing the student API.
        """
        if provider is not None and not callable(provider):
            raise TypeError("provider must be callable or None")
        with self._lock:
            self._obstacle_provider = provider

    # =========================================================================
    # Differential drive — velocity commands
    # =========================================================================

    def set_velocity(self, linear: float, angular_deg_s: float) -> None:
        """
        Body-frame velocity command.
          linear        — forward speed in user units/s
          angular_deg_s — rotation rate in degrees/s (CCW positive)
        """
        linear_mm    = linear * self._unit.value
        angular_rad_s = math.radians(angular_deg_s)
        self._send_body_velocity_mm(linear_mm, angular_rad_s)

    def set_motor_velocity(self, motor_id: int, velocity: float) -> None:
        """Command a single DC motor by velocity in user units/s."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        self._send_motor_velocity_mm(motor_id, velocity * self._unit.value)

    def stop(self) -> None:
        """
        Zero velocity on both drive motors. The firmware stays in RUNNING state.
        Use estop() for an emergency stop.
        """
        self._send_motor_velocity_mm(self._left_wheel_motor, 0.0)
        self._send_motor_velocity_mm(self._right_wheel_motor, 0.0)

    def disable_drive_motors(self) -> None:
        """Disable the currently configured drive motors."""
        with self._lock:
            drive_motors = tuple(dict.fromkeys((self._left_wheel_motor, self._right_wheel_motor)))
        for motor_id in drive_motors:
            self.disable_motor(motor_id)

    def shutdown(self) -> None:
        """
        Stop navigation and leave the drive motors disabled.

        This is intended for node shutdown so the robot does not keep the
        configured drive motors armed after the ROS process exits.
        """
        self.cancel_motion()
        self.stop()
        time.sleep(self._SHUTDOWN_SETTLE_S)
        self.disable_drive_motors()
        time.sleep(self._SHUTDOWN_SETTLE_S)
        if self.get_state() == FirmwareState.RUNNING:
            self.set_state(FirmwareState.IDLE, timeout=1.0)
        self.save_trajectory_image()

    def set_left_wheel(self, motor_id: int) -> None:
        """Alias for set_odom_left_motor()."""
        self.set_odom_left_motor(motor_id)

    def set_right_wheel(self, motor_id: int) -> None:
        """Alias for set_odom_right_motor()."""
        self.set_odom_right_motor(motor_id)

    def set_drive_wheels(self, left_motor_id: int, right_motor_id: int) -> None:
        """Alias for set_odom_motors()."""
        self.set_odom_motors(left_motor_id, right_motor_id)

    def get_left_wheel(self) -> int:
        return self._left_wheel_motor

    def get_right_wheel(self) -> int:
        return self._right_wheel_motor

    # =========================================================================
    # Navigation  (higher-level, runs a background thread)
    # =========================================================================

    def move_to(
        self,
        x: float,
        y: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ):
        """
        Navigate to (x, y) at the given speed.
        Uses pure-pursuit steering. Requires firmware to be in RUNNING state.

        Always returns a MotionHandle.
        blocking=True waits before returning the handle.
        tolerance      — arrival radius in user units
        """
        x_mm   = x         * self._unit.value
        y_mm   = y         * self._unit.value
        vel_mm = velocity  * self._unit.value
        tol_mm = tolerance * self._unit.value
        lookahead_mm = max(tol_mm * 2.0, 100.0)

        def target():
            self._nav_follow_purepursuit_path(
                [(x_mm, y_mm)],
                vel_mm,
                lookahead_mm,
                tol_mm,
                2.0,
            )

        return self._start_nav(target, blocking, timeout)

    def move_by(
        self,
        dx: float,
        dy: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ):
        """Navigate by (dx, dy) relative to current pose."""
        cur_x, cur_y, _ = self.get_pose()
        return self.move_to(cur_x + dx, cur_y + dy, velocity,
                            blocking=blocking, tolerance=tolerance, timeout=timeout)

    def move_forward(
        self,
        distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ):
        """
        Navigate forward by distance along the robot's current heading.

        distance and tolerance are in the current user unit system.
        """
        distance = self._require_positive_float("distance", distance)
        return self._move_along_heading(
            distance,
            velocity,
            tolerance=tolerance,
            blocking=blocking,
            timeout=timeout,
        )

    def move_backward(
        self,
        distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ):
        """
        Navigate backward by distance along the robot's current heading.

        distance and tolerance are in the current user unit system.
        """
        distance = self._require_positive_float("distance", distance)
        return self._move_along_heading(
            -distance,
            velocity,
            tolerance=tolerance,
            blocking=blocking,
            timeout=timeout,
        )

    def _move_along_heading(
        self,
        signed_distance: float,
        velocity: float,
        tolerance: float,
        blocking: bool = True,
        timeout: float = None,
    ):
        """Internal helper: move along the current robot heading by signed_distance."""
        cur_x, cur_y, cur_theta_deg = self.get_pose()
        cur_theta_rad = math.radians(cur_theta_deg)
        dx = math.cos(cur_theta_rad) * signed_distance
        dy = math.sin(cur_theta_rad) * signed_distance
        return self.move_to(
            cur_x + dx,
            cur_y + dy,
            velocity,
            tolerance=tolerance,
            blocking=blocking,
            timeout=timeout,
        )

    def purepursuit_follow_path(
        self,
        waypoints: list[tuple[float, float]],
        velocity: float,
        lookahead: float,
        tolerance: float,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        timeout: float = None,
        *,
        advance_radius: float | None = None,
    ):
        """
        Follow an ordered waypoint path with pure pursuit.

        waypoints           — [(x, y), ...] in the current user unit system
        velocity            — maximum forward speed in user units/s
        lookahead           — lookahead distance in user units
        tolerance           — goal tolerance in user units
        advance_radius      — optional intermediate waypoint acceptance radius
                              in user units; defaults to tolerance
        max_angular_rad_s   — angular-rate clamp in rad/s

        Always returns a MotionHandle.
        blocking=True waits before returning the handle.
        """
        if not waypoints:
            raise ValueError("waypoints must not be empty")

        path_mm = [
            (float(x) * self._unit.value, float(y) * self._unit.value)
            for x, y in waypoints
        ]
        vel_mm = float(velocity) * self._unit.value
        lookahead_mm = float(lookahead) * self._unit.value
        tolerance_mm = float(tolerance) * self._unit.value
        advance_radius_mm = (
            tolerance_mm if advance_radius is None
            else float(advance_radius) * self._unit.value
        )
        max_angular = float(max_angular_rad_s)

        def target():
            self._nav_follow_purepursuit_path(
                path_mm,
                vel_mm,
                lookahead_mm,
                advance_radius_mm,
                tolerance_mm,
                max_angular,
            )

        return self._start_nav(target, blocking, timeout)

    def apf_follow_path(
        self,
        waypoints: list[tuple[float, float]],
        velocity: float,
        lookahead: float,
        tolerance: float,
        repulsion_range: float,
        blocking: bool = True,
        max_angular_rad_s: float = 1.0,
        repulsion_gain: float = 500.0,
        timeout: float = None,
        *,
        advance_radius: float | None = None,
    ):
        """
        Follow an ordered waypoint path with artificial potential fields.

        waypoints         — [(x, y), ...] in the current user unit system
        velocity          — maximum forward speed in user units/s
        lookahead         — attractive lookahead distance in user units
        tolerance         — goal tolerance in user units
        repulsion_range   — obstacle influence radius in user units
        advance_radius    — optional intermediate waypoint acceptance radius
                            in user units; defaults to tolerance

        Obstacles come from set_obstacles() and/or set_obstacle_provider().
        """
        if not waypoints:
            raise ValueError("waypoints must not be empty")

        path_mm = [
            (float(x) * self._unit.value, float(y) * self._unit.value)
            for x, y in waypoints
        ]
        vel_mm = float(velocity) * self._unit.value
        lookahead_mm = float(lookahead) * self._unit.value
        tolerance_mm = float(tolerance) * self._unit.value
        advance_radius_mm = (
            tolerance_mm if advance_radius is None
            else float(advance_radius) * self._unit.value
        )
        repulsion_range_mm = float(repulsion_range) * self._unit.value
        max_angular = float(max_angular_rad_s)
        repulsion_gain = float(repulsion_gain)

        def target():
            self._nav_follow_apf_path(
                path_mm,
                vel_mm,
                lookahead_mm,
                advance_radius_mm,
                tolerance_mm,
                repulsion_range_mm,
                max_angular,
                repulsion_gain,
            )

        return self._start_nav(target, blocking, timeout)

    def turn_to(
        self,
        angle_deg: float,
        blocking: bool = True,
        tolerance_deg: float = 2.0,
        timeout: float = None,
    ):
        """
        Rotate to an absolute heading. Firmware must be in RUNNING state.
        angle_deg is in degrees (CCW positive, same convention as the firmware).

        Always returns a MotionHandle.
        blocking=True waits before returning the handle.
        """
        target_rad = math.radians(angle_deg)
        tol_rad    = math.radians(tolerance_deg)

        def target():
            self._turn_to_heading(target_rad, tol_rad)

        return self._start_nav(target, blocking, timeout)

    def turn_by(
        self,
        delta_deg: float,
        blocking: bool = True,
        tolerance_deg: float = 2.0,
        timeout: float = None,
    ):
        """
        Rotate by delta_deg relative to current heading.

        Always returns a MotionHandle.
        blocking=True waits before returning the handle.
        """
        _, _, cur_deg = self.get_pose()
        return self.turn_to(cur_deg + delta_deg,
                            blocking=blocking, tolerance_deg=tolerance_deg, timeout=timeout)

    def is_moving(self) -> bool:
        """True if a navigation command is active."""
        return self._nav_thread is not None and self._nav_thread.is_alive()

    def cancel_motion(self) -> None:
        """Abort any active navigation command. The robot stops."""
        self._nav_cancel.set()
        if self._nav_thread is not None:
            self._nav_thread.join(timeout=1.0)
            if self._nav_thread.is_alive():
                return
        self._nav_thread = None
        self._nav_cancel.clear()

    # =========================================================================
    # DC motors — low-level
    # =========================================================================

    def set_motor_pwm(self, motor_id: int, pwm: int) -> None:
        """Raw PWM command, −255 … 255."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCSetPwm()
        msg.motor_number = motor_id
        msg.pwm = int(pwm)
        self._dc_pwm_pub.publish(msg)

    def set_motor_position(
        self,
        motor_id: int,
        ticks: int,
        max_vel_ticks: int = 200,
        blocking: bool = True,
        tolerance_ticks: int = 20,
        timeout: float = None,
    ) -> bool:
        """
        Move a DC motor to an absolute encoder position.
        Returns True when within tolerance, False on timeout.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCSetPosition()
        msg.motor_number  = motor_id
        msg.target_ticks  = int(ticks)
        msg.max_vel_ticks = int(max_vel_ticks)
        self._dc_pos_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_dc_position(motor_id, ticks, tolerance_ticks, timeout)

    def enable_motor(
        self,
        motor_id: int,
        mode: DCMotorMode | int = DCMotorMode.VELOCITY,
    ) -> None:
        """
        Enable a DC motor in the requested firmware mode.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        mode = self._require_enum("mode", mode, DCMotorMode)
        msg = DCEnable()
        msg.motor_number = motor_id
        msg.mode = mode
        self._dc_en_pub.publish(msg)

    def disable_motor(self, motor_id: int) -> None:
        """Disable a DC motor (mode 0)."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCEnable()
        msg.motor_number = motor_id
        msg.mode = int(DCMotorMode.DISABLED)
        self._dc_en_pub.publish(msg)

    def home_motor(
        self,
        motor_id: int,
        direction: int = 1,
        home_velocity: int = 100,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """
        Run a DC motor toward its limit switch for homing.
        direction: +1 or -1
        Returns True when homing completes, False on timeout.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCHome()
        msg.motor_number  = motor_id
        msg.direction     = direction
        msg.home_velocity = home_velocity
        self._dc_home_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_dc_not_homing(motor_id, timeout)

    def reset_motor_position(self, motor_id: int) -> None:
        """Zero the encoder position for a DC motor."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCResetPosition()
        msg.motor_number = motor_id
        self._dc_rst_pub.publish(msg)

    def set_pid_gains(
        self,
        motor_id: int,
        loop_type: DCPidLoop | int,
        kp: float,
        ki: float,
        kd: float,
        max_output: float = 255.0,
        max_integral: float = 1000.0,
    ) -> None:
        """
        Set PID gains for a DC motor.
        loop_type follows the firmware enum:
        0=position, 1=velocity.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        loop_type = self._require_enum("loop_type", loop_type, DCPidLoop)
        msg = DCPidSet()
        msg.motor_number  = motor_id
        msg.loop_type     = loop_type
        msg.kp            = float(kp)
        msg.ki            = float(ki)
        msg.kd            = float(kd)
        msg.max_output    = float(max_output)
        msg.max_integral  = float(max_integral)
        self._dc_pid_set.publish(msg)

    def request_pid(self, motor_id: int, loop_type: DCPidLoop | int) -> None:
        """
        Request PID parameters from firmware. Response arrives on /dc_pid_rsp topic.
        loop_type follows the firmware enum:
        0=position, 1=velocity.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        loop_type = self._require_enum("loop_type", loop_type, DCPidLoop)
        msg = DCPidReq()
        msg.motor_number = motor_id
        msg.loop_type    = loop_type
        self._dc_pid_req.publish(msg)

    def get_pid(self, motor_id: int, loop_type: DCPidLoop | int) -> DCPid | None:
        """Return cached PID parameters for one motor/loop pair, or None if unknown."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        loop_type = self._require_enum("loop_type", loop_type, DCPidLoop)
        with self._lock:
            return self._dc_pid_cache.get((motor_id, loop_type))

    def get_dc_state(self) -> DCStateAll:
        """Return cached DC motor state (position, velocity, mode, etc.)."""
        with self._lock:
            return self._dc_state

    # =========================================================================
    # Stepper motors
    # =========================================================================

    def step_enable(self, stepper_id: int) -> None:
        """Enable a stepper motor."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepEnable()
        msg.stepper_number = stepper_id
        msg.enable = True
        self._step_en_pub.publish(msg)

    def step_disable(self, stepper_id: int) -> None:
        """Disable a stepper motor."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepEnable()
        msg.stepper_number = stepper_id
        msg.enable = False
        self._step_en_pub.publish(msg)

    def step_move(
        self,
        stepper_id: int,
        steps: int,
        move_type: StepMoveType | int = StepMoveType.RELATIVE,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """
        Move a stepper motor.
        move_type follows the firmware enum:
        0=absolute, 1=relative.
        Returns True when motion completes, False on timeout.
        """
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        move_type = self._require_enum("move_type", move_type, StepMoveType)
        msg = StepMove()
        msg.stepper_number = stepper_id
        msg.move_type      = move_type
        msg.target         = int(steps)
        self._step_mv_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_stepper_idle(stepper_id, timeout)

    def step_home(
        self,
        stepper_id: int,
        direction: int = -1,
        home_velocity: int = 500,
        backoff_steps: int = 50,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """
        Home a stepper motor against its limit switch.
        Returns True when complete, False on timeout.
        """
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepHome()
        msg.stepper_number = stepper_id
        msg.direction      = direction
        msg.home_velocity  = home_velocity
        msg.backoff_steps  = backoff_steps
        self._step_hm_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_stepper_idle(stepper_id, timeout)

    def step_set_config(
        self,
        stepper_id: int,
        max_velocity: int,
        acceleration: int,
    ) -> None:
        """Set speed and acceleration for a stepper motor."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepConfigSet()
        msg.stepper_number = stepper_id
        msg.max_velocity   = int(max_velocity)
        msg.acceleration   = int(acceleration)
        self._step_cfg_pub.publish(msg)

    def request_step_config(self, stepper_id: int) -> None:
        """Request current speed/acceleration config for one stepper."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepConfigReq()
        msg.stepper_number = stepper_id
        self._step_cfg_req_pub.publish(msg)

    def get_step_config(self, stepper_id: int) -> StepConfig | None:
        """Return cached config for one stepper, or None if it has not been seen yet."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        with self._lock:
            return self._step_config_cache.get(stepper_id)

    def get_step_state(self) -> StepStateAll:
        """Return cached stepper motor state."""
        with self._lock:
            return self._step_state

    # =========================================================================
    # Servos
    # =========================================================================

    def set_servo(self, channel: int, angle_deg: float) -> None:
        """
        Set a servo to angle_deg (0–180°).
        Maps 0° → 1000 µs, 180° → 2000 µs.
        """
        channel = self._require_id("channel", channel, 1, 16)
        angle_clamped = max(0.0, min(180.0, angle_deg))
        pulse_us = int(self._SERVO_MIN_US + (angle_clamped / 180.0) * (self._SERVO_MAX_US - self._SERVO_MIN_US))
        msg = ServoSet()
        msg.channel  = channel
        msg.pulse_us = pulse_us
        self._srv_set_pub.publish(msg)

    def set_servo_270(self, channel: int, angle_deg: float) -> None:
        """
        Set a 270° servo to angle_deg (0–270°).
        Bypasses default limits to map 0° → 500 µs, 270° → 2500 µs.
        """
        channel = self._require_id("channel", channel, 1, 16)
        angle_clamped = max(0.0, min(270.0, angle_deg))
        pulse_us = int(500 + (angle_clamped / 270.0) * (2500 - 500))
        msg = ServoSet()
        msg.channel  = channel
        msg.pulse_us = pulse_us
        self._srv_set_pub.publish(msg)

    def set_servo_180_wide(self, channel: int, angle_deg: float) -> None:
        """
        Set a wide-band 180° servo (like the DS3240MG-180) to angle_deg (0–180°).
        Bypasses default limits to map 0° → 500 µs, 180° → 2500 µs.
        """
        channel = self._require_id("channel", channel, 1, 16)
        angle_clamped = max(0.0, min(180.0, angle_deg))
        
        # Maps 0-180 directly to 500-2500
        pulse_us = int(500 + (angle_clamped / 180.0) * (2000))
        
        msg = ServoSet()
        msg.channel  = channel
        msg.pulse_us = pulse_us
        self._srv_set_pub.publish(msg)

    def set_servo_pulse(self, channel: int, pulse_us: int) -> None:
        """Set a servo directly by pulse width in microseconds."""
        channel = self._require_id("channel", channel, 1, 16)
        msg = ServoSet()
        msg.channel  = channel
        msg.pulse_us = int(pulse_us)
        self._srv_set_pub.publish(msg)

    def enable_servo(self, channel: int) -> None:
        channel = self._require_id("channel", channel, 1, 16)
        msg = ServoEnable()
        msg.channel = channel
        msg.enable  = True
        self._srv_en_pub.publish(msg)

    def disable_servo(self, channel: int) -> None:
        channel = self._require_id("channel", channel, 1, 16)
        msg = ServoEnable()
        msg.channel = channel
        msg.enable  = False
        self._srv_en_pub.publish(msg)

    def get_servo_state(self) -> ServoStateAll:
        """Return cached servo state."""
        with self._lock:
            return self._servo_state

    # =========================================================================
    # IO — buttons, limits, LEDs, NeoPixel
    # =========================================================================

    def get_button(self, button_id: int) -> bool:
        """
        Non-blocking: current state of button button_id (1–10).
        Reads from the latest cached IOInputState message.
        Return True if the button is pressed.
        """
        button_id = self._require_id("button_id", button_id, 1, BUTTON_COUNT)
        with self._lock:
            return bool((self._buttons >> (button_id - 1)) & 1)

    def was_button_pressed(self, button_id: int, consume: bool = True) -> bool:
        """
        Return True once per rising edge seen on button_id.

        Edges are latched in the ROS subscription callback at firmware telemetry
        rate, so short presses are not lost when the FSM loop runs slower.
        """
        button_id = self._require_id("button_id", button_id, 1, BUTTON_COUNT)
        mask = 1 << (button_id - 1)
        with self._lock:
            pressed = bool(self._button_edges & mask)
            if pressed and consume:
                self._button_edges &= ~mask
            return pressed

    def wait_for_button(self, button_id: int, timeout: float = None) -> bool:
        """
        Blocking: wait until button_id transitions from not-pressed to pressed.
        Returns True if pressed within timeout, False on timeout.
        """
        button_id = self._require_id("button_id", button_id, 1, BUTTON_COUNT)
        with self._lock:
            if (self._buttons >> (button_id - 1)) & 1:
                return True
            ev = threading.Event()
            self._button_events[button_id] = ev

        pressed = ev.wait(timeout=timeout)
        with self._lock:
            self._button_events.pop(button_id, None)
        return pressed

    def get_limit(self, limit_id: int) -> bool:
        """Non-blocking: current state of limit switch limit_id (1–8)."""
        limit_id = self._require_id("limit_id", limit_id, 1, LIMIT_COUNT)
        with self._lock:
            return bool((self._limits >> (limit_id - 1)) & 1)

    def get_io_output_state(self) -> IOOutputState:
        """Return cached LED/NeoPixel output state from /io_output_state."""
        with self._lock:
            return self._io_output_state

    def was_limit_triggered(self, limit_id: int, consume: bool = True) -> bool:
        """Return True once per rising edge seen on limit_id."""
        limit_id = self._require_id("limit_id", limit_id, 1, LIMIT_COUNT)
        mask = 1 << (limit_id - 1)
        with self._lock:
            triggered = bool(self._limit_edges & mask)
            if triggered and consume:
                self._limit_edges &= ~mask
            return triggered

    def wait_for_limit(self, limit_id: int, timeout: float = None) -> bool:
        """Blocking: wait until limit switch limit_id is triggered."""
        limit_id = self._require_id("limit_id", limit_id, 1, LIMIT_COUNT)
        with self._lock:
            if (self._limits >> (limit_id - 1)) & 1:
                return True
            ev = threading.Event()
            self._limit_events[limit_id] = ev

        triggered = ev.wait(timeout=timeout)
        with self._lock:
            self._limit_events.pop(limit_id, None)
        return triggered

    def set_led(
        self,
        led_id: int,
        brightness: int,
        mode: LEDMode | int | None = None,
        period_ms: int | None = None,
        duty_cycle: int = 500,
    ) -> None:
        """
        Control an onboard LED.
        When mode is omitted, brightness 0 maps to OFF and non-zero brightness
        maps to steady ON. Explicit modes follow the firmware contract:
        OFF=0, ON=1, BLINK=2, BREATHE=3, PWM=4.
        BLINK and BREATHE default to a 1000 ms period when none is supplied.
        duty_cycle is in permille (0-1000). For BLINK it controls the ON-time
        share of the period. For BREATHE it controls the rise-time share of
        the period; 500 gives a symmetric inhale/exhale.
        """
        led_id = self._require_id("led_id", led_id, 0, 4)
        clamped_brightness = max(0, min(255, int(brightness)))
        resolved_mode = (
            int(LEDMode.OFF if clamped_brightness == 0 else LEDMode.ON)
            if mode is None
            else self._require_enum("mode", mode, LEDMode)
        )
        if period_ms is None:
            period_ms = 1000 if resolved_mode in (int(LEDMode.BLINK), int(LEDMode.BREATHE)) else 0
        msg = IOSetLed()
        msg.led_id     = led_id
        msg.brightness = clamped_brightness
        msg.mode       = resolved_mode
        msg.period_ms  = max(0, int(period_ms))
        msg.duty_cycle = max(0, min(1000, int(duty_cycle)))
        self._led_pub.publish(msg)

    def set_neopixel(self, index: int, red: int, green: int, blue: int) -> None:
        """Set a NeoPixel LED color. index is 0-based."""
        msg = IOSetNeopixel()
        msg.index = index
        msg.red   = red
        msg.green = green
        msg.blue  = blue
        self._neo_pub.publish(msg)

    # =========================================================================
    # IMU
    # =========================================================================

    def get_imu(self) -> SensorImu:
        """Return cached IMU state (quaternion, accel, gyro, magnetometer)."""
        with self._lock:
            return self._imu

    def get_fused_orientation(self) -> float:
        """
        Return complementary-filter orientation estimate in degrees.

        Blends the absolute AHRS heading (corrects long-term drift) with
        the odometry theta (smooth, short-term accurate). The filter weight
        ``_orientation_fusion_alpha`` (default 0.02) controls how quickly the AHRS
        heading pulls the estimate; it only activates once the firmware reports
        ``mag_calibrated = True``. Before calibration, returns odometry theta.
        """
        with self._lock:
            return math.degrees(self._fused_theta)

    def set_orientation_fusion_strategy(self, strategy: SensorFusion) -> None:
        """
        Replace the active heading-fusion strategy.

        Any SensorFusion subclass is accepted 
        The new strategy takes effect on the next kinematics update.
        """
        if strategy.measurement_type != "orientation":
            raise ValueError(f"Orientation fusion strategy must have measurement_type='orientation', but got '{strategy.measurement_type}'")
        with self._lock:
            self._orientation_fusion = strategy

    def set_orientation_fusion_alpha(self, alpha: float) -> None:
        """
        Set the AHRS heading weight for the complementary filter (0.0–1.0).

        Only valid when the active strategy is OrientationComplementaryFilter (the default).
        Raises TypeError if a different strategy is currently active — call
        set_orientation_fusion_strategy(OrientationComplementaryFilter(alpha)) instead.

        Lower values trust odometry more (less AHRS noise, more drift).
        Higher values correct drift faster but introduce more AHRS noise.
        Default is 0.02.
        """
        with self._lock:
            if not isinstance(self._orientation_fusion, OrientationComplementaryFilter):
                raise TypeError(
                    f"set_orientation_fusion_alpha() requires the active strategy to be "
                    f"OrientationComplementaryFilter, but it is "
                    f"{type(self._fusion).__name__}. "
                    "Use set_orientation_fusion_strategy(OrientationComplementaryFilter(alpha)) instead."
                )
            self._orientation_fusion.alpha = max(0.0, min(1.0, float(alpha)))

    def set_imu_z_down(self, z_down: bool) -> None:
        """
        Configure the IMU mounting orientation.

        Set to True when the sensor is mounted upside-down (Z-axis pointing
        toward the ground).  When enabled, the yaw extracted from the AHRS
        quaternion is negated to undo the sign flip caused by the inverted
        attitude the Fusion library converges to.

        The IMU must be re-calibrated after physically flipping the
        sensor; calibration data collected in one orientation is not valid in
        the other.

        Default is False (Z-axis points up, chip label visible from above).
        """
        with self._lock:
            self._imu_z_down = bool(z_down)

    def get_fused_pose(self) -> tuple[float, float, float]:
        """
        Return the fused (x, y, theta_deg) in user units and degrees.

        Position is a complementary-filter blend of GPS (ArUco tag) and odometry.
        Orientation is a complementary-filter blend of AHRS heading and odometry.
        When the GPS tag is not visible for more than ``_gps_timeout_s`` seconds,
        the position falls back to pure odometry automatically.
        """
        return self.get_pose()

    def set_tracked_tag_id(self, tag_id: int) -> None:
        """
        Set the ArUco tag ID used for GPS position fusion.

        Only detections matching this tag contribute to the fused position.
        Pass -1 (default) to accept any tag from /tag_detections.
        """
        with self._lock:
            self._tracked_tag_id = int(tag_id)

    def get_tracked_tag_id(self) -> int:
        """Return the currently tracked ArUco tag ID (-1 means any)."""
        with self._lock:
            return self._tracked_tag_id

    def set_gps_offset(self, offset_x_mm: float, offset_y_mm: float) -> None:
        """
        Set the fixed translation from GPS frame to arena frame (mm).

        The GPS (ArUco tag) coordinate origin does not coincide with the robot's
        starting corner. Measure this offset once — e.g. by driving the robot to
        a point whose arena coordinates are known, reading the GPS fix at that
        point, and computing offset = arena_pos - gps_pos — then pass it here.

        arena_x = gps_x + offset_x
        arena_y = gps_y + offset_y
        """
        with self._lock:
            self._gps_offset_x_mm = float(offset_x_mm)
            self._gps_offset_y_mm = float(offset_y_mm)

    def set_tag_body_offset(self, forward_mm: float, left_mm: float) -> None:
        """
        Set the ArUco tag mounting offset relative to the robot body origin (mm).

        If the tag is not centred on the robot's body origin, pass the tag's
        position in the robot's local frame here so the fusion corrects for it:

        - ``forward_mm`` — positive = tag is ahead of body centre.
        - ``left_mm``    — positive = tag is to the left of body centre.

        The correction is applied every time a tag detection arrives by rotating
        the body-frame offset into the arena frame using the current fused heading
        and subtracting it from the detected tag position.

        Default is (0, 0) — tag assumed to be at the body origin.
        """
        with self._lock:
            self._tag_body_offset_x_mm = float(forward_mm)
            self._tag_body_offset_y_mm = float(left_mm)

    def set_position_fusion_strategy(self, strategy: SensorFusion) -> None:
        """
        Replace the active position-fusion strategy.

        Any SensorFusion subclass is accepted 
        The new strategy takes effect on the next kinematics update.
        """
        if strategy.measurement_type != "position":
            raise ValueError(f"Position fusion strategy must have measurement_type='position', but got '{strategy.measurement_type}'")
        with self._lock:
            self._pos_fusion = strategy
    
    def set_position_fusion_alpha(self, alpha: float) -> None:
        """
        Set the GPS weight for position fusion (0.0–1.0).

        0.0 — trust odometry only (GPS ignored even when available).
        1.0 — snap position directly to GPS each kinematics tick.
        Default is 0.10.
        """
        with self._lock:
            self._pos_fusion.alpha = max(0.0, min(1.0, float(alpha)))

    def save_trajectory_image(self, path: str = "trajectory.png") -> None:
        """Save a PNG comparing raw odometry vs fused trajectory. Requires matplotlib."""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self._node.get_logger().error("matplotlib not installed; cannot save trajectory image")
            return

        odom = list(self._odom_traj)
        fused = list(self._fused_traj)
        if not odom:
            self._node.get_logger().warn("No trajectory data to save")
            return

        fig, ax = plt.subplots(figsize=(8, 8))
        ox, oy = zip(*odom)
        ax.plot(ox, oy, label="odometry (raw)", color="steelblue", linewidth=1.5)
        if fused:
            fx, fy = zip(*fused)
            ax.plot(fx, fy, label="fused (GPS+odom)", color="darkorange", linewidth=1.5, linestyle="--")
        ax.scatter([ox[0]], [oy[0]], color="green", zorder=5, label="start")
        ax.scatter([ox[-1]], [oy[-1]], color="red", zorder=5, label="end")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title("Trajectory: raw odometry vs fused")
        ax.legend()
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        self._node.get_logger().info(f"Trajectory image saved to {path}")

    def is_gps_active(self) -> bool:
        """Return True if a GPS fix for the tracked tag arrived within the staleness window."""
        with self._lock:
            return (
                self._gps_last_time > 0.0
                and (_time.monotonic() - self._gps_last_time) < self._gps_timeout_s
            )

    # =========================================================================
    # Units
    # =========================================================================

    def set_unit(self, unit: Unit) -> None:
        """Change the unit system. Does not affect already-sent commands."""
        self._unit = unit

    def get_unit(self) -> Unit:
        return self._unit

    # =========================================================================
    # Internal — navigation
    # =========================================================================

    def _start_nav(self, target_fn, blocking: bool, timeout: float):
        """Start a new navigation thread. Only one base-motion command may run at a time."""
        if self.is_moving():
            raise RuntimeError("Another motion is still running")

        self._nav_done.clear()
        self._nav_cancel.clear()

        def runner() -> None:
            try:
                target_fn()
            except Exception as exc:
                self._node.get_logger().error(f"motion thread failed: {exc}")
                try:
                    self.stop()
                except Exception:
                    pass
            finally:
                self._nav_done.set()

        self._nav_thread = threading.Thread(target=runner, daemon=True)
        self._nav_thread.start()
        handle = MotionHandle(self._nav_done, self._nav_cancel)
        if blocking:
            handle.wait(timeout=timeout)
        return handle

    def _nav_follow_purepursuit_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        max_vel_mm: float,
        lookahead_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        max_angular_rad_s: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: pure-pursuit to an ordered list of waypoints (mm)."""
        from robot.path_planner import PurePursuitPlanner

        planner = PurePursuitPlanner(
            lookahead_dist=lookahead_mm,
            max_angular=max_angular_rad_s,
            goal_tolerance=tolerance_mm,
        )
        self._nav_follow_path(
            waypoints_mm,
            planner,
            max_vel_mm,
            advance_radius_mm,
            tolerance_mm,
            update_hz=update_hz,
        )

    def _nav_follow_apf_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        max_vel_mm: float,
        lookahead_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        repulsion_range_mm: float,
        max_angular_rad_s: float,
        repulsion_gain: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: APF path following with robot-frame obstacles."""
        from robot.path_planner import APFPlanner

        planner = APFPlanner(
            lookahead_dist=lookahead_mm,
            max_linear=max_vel_mm,
            max_angular=max_angular_rad_s,
            repulsion_gain=repulsion_gain,
            repulsion_range=repulsion_range_mm,
            goal_tolerance=tolerance_mm,
            obstacle_provider=self._get_obstacles_mm,
        )
        self._nav_follow_path(
            waypoints_mm,
            planner,
            max_vel_mm,
            advance_radius_mm,
            tolerance_mm,
            update_hz=update_hz,
        )

    def _nav_follow_path(
        self,
        waypoints_mm: list[tuple[float, float]],
        planner,
        max_vel_mm: float,
        advance_radius_mm: float,
        tolerance_mm: float,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Shared path-following loop for pure pursuit and APF planners."""
        remaining_path = list(waypoints_mm)
        dt = 1.0 / update_hz

        while not self._nav_cancel.is_set():
            x_mm, y_mm, theta_rad = self._get_pose_mm()
            remaining_path = self._advance_remaining_path(
                remaining_path,
                x_mm,
                y_mm,
                advance_radius_mm,
            )

            goal_x_mm, goal_y_mm = remaining_path[0]
            if len(remaining_path) == 1 and _dist2d(x_mm, y_mm, goal_x_mm, goal_y_mm) <= tolerance_mm:
                self.stop()
                return

            linear_mm, angular_rad_s = planner.compute_velocity(
                (x_mm, y_mm, theta_rad),
                remaining_path,
                max_vel_mm,
            )
            self._send_body_velocity_mm(linear_mm, angular_rad_s)
            if not self._sleep_with_cancel(dt):
                break

        self.stop()

    @staticmethod
    def _advance_remaining_path(
        remaining_path: list[tuple[float, float]],
        x_mm: float,
        y_mm: float,
        advance_radius_mm: float,
    ) -> list[tuple[float, float]]:
        if not remaining_path:
            return remaining_path
            
        # 1. Find the closest waypoint in the near future
        min_dist = float('inf')
        closest_idx = 0
        
        # Look ahead up to 40 waypoints (2000mm at 20mm spacing) 
        # to prevent jumping to adjacent parallel lanes
        search_window = min(len(remaining_path), 40) 
        for i in range(search_window):
            dist = _dist2d(x_mm, y_mm, remaining_path[i][0], remaining_path[i][1])
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # 2. Fast-forward the path to the closest waypoint (drops everything behind the rover)
        for _ in range(closest_idx):
            remaining_path.pop(0)

        # 3. Apply the normal radius pop (if we are physically over the target)
        while len(remaining_path) > 1:
            next_x_mm, next_y_mm = remaining_path[0]
            if _dist2d(x_mm, y_mm, next_x_mm, next_y_mm) > advance_radius_mm:
                break
            remaining_path.pop(0)
            
        return remaining_path

    def _turn_to_heading(
        self,
        target_rad: float,
        tolerance_rad: float,
        max_angular_rad: float = 2.0,
        update_hz: float = float(DEFAULT_NAV_HZ),
    ) -> None:
        """Navigation thread body: rotate to target_rad in place."""
        dt = 1.0 / update_hz
        while not self._nav_cancel.is_set():
            _, _, theta_rad = self._get_pose_mm()
            error = _wrap_angle(target_rad - theta_rad)
            if abs(error) < tolerance_rad:
                self.stop()
                return
            angular = max(-max_angular_rad, min(max_angular_rad, error * 3.0))
            self._send_body_velocity_mm(0.0, angular)
            if not self._sleep_with_cancel(dt):
                break
        self.stop()

    def _sleep_with_cancel(self, seconds: float) -> bool:
        """
        Wait for up to seconds, but wake early if the active base motion is cancelled.

        Returns True when the full interval elapsed, False when cancel was requested.
        """
        return not self._nav_cancel.wait(timeout=seconds)


    # =========================================================================
    # Obstacle avoidance — DWA path following
    # =========================================================================

    def _init_dwa_planner(
        self,
        max_vel_mm: float,
        max_acc_mm: float,
        max_angular_rad: float,
        max_angular_acc_rad: float,
        lookahead_mm: float,
        robot_front_mm: float,       # <-- New parameter
        robot_rear_mm: float,        # <-- New parameter
        robot_half_width_mm: float,  # <-- New parameter
        tolerance_mm: float,
        gains_of_costs: list,
        period: float,
        predict_time: float,
        predict_velocity_samples_resolution: list,
        obstacles_range_mm: float,
        ttc_weight: float,
    ) -> None:
        """
        Initializes the Dynamic Window Approach (DWA) planner with a rectangular footprint.
        Call this once before starting the navigation loop.
        """
        from robot.path_planner import DWAPlanner
        
        self._planner = DWAPlanner(
            lookahead_dist=lookahead_mm,
            max_linear_speed=max_vel_mm,
            max_angular_speed=max_angular_rad,
            max_linear_acc=max_acc_mm,
            max_angular_acc=max_angular_acc_rad,
            goal_tolerance=tolerance_mm,
            gains_of_costs=gains_of_costs,
            dt=period,
            predict_time=predict_time,
            predict_velocity_samples_resolution=predict_velocity_samples_resolution,
            robot_front_mm=robot_front_mm,             # <-- Passed to planner
            robot_rear_mm=robot_rear_mm,               # <-- Passed to planner
            robot_half_width_mm=robot_half_width_mm,   # <-- Passed to planner
            obstacles_range=obstacles_range_mm,
            ttc_weight=ttc_weight,
        )

    def _nav_follow_path_loop(self, path: list[tuple[float, float]], period: float) -> str:
        """
        Executes a single step of the DWA navigation loop, dynamically popping 
        passed waypoints to prevent backtracking.
        """
        with self._lock:
            obstacles = self._obstacles_mm.copy()
            pose = self._pose
            vel = self._vel

        # 1. Pop old waypoints from the path just like Pure Pursuit does!
        # advance_radius = getattr(self._planner, 'lookahead_dist', 125.0)    # Using the planner's lookahead_dist or defined value as the pop radius.
        advance_radius = 125.0
        self._advance_remaining_path(path, pose[0], pose[1], advance_radius)

        # 2. Check if the final waypoint has been reached
        if self._planner.TargetReached(path, pose[0], pose[1]):
            self._node.get_logger().info("MOVING: Target reached! Stopping.")
            self.stop()
            return "IDLE"

        # 3. Compute optimal velocity commands for this tick
        v, w = self._planner.compute_velocity(path, pose, vel, obstacles, period)

        # --- COLLISION CATCHER ---
        if v is None and w is None:
            self.stop()
            return "COLLISION"
        # ------------------------------
        
        # --- Debug Logging ---
        self._node.get_logger().info(
            f"Computed velocity: linear={v:.1f} mm/s, angular={math.degrees(w):.1f} deg/s"
        )
        self._node.get_logger().info(
            f"LiDAR points detected: {len(obstacles)}"
        )
        # ---------------------
        
        # 4. Send commands to the motors
        self.set_velocity(v, math.degrees(w))

        return "MOVING"

    # For collision handling: not using anymore
    # def get_closest_lidar_point(self) -> tuple[float | None, float | None]:
    #     """
    #     Returns the (distance_mm, angle_deg) of the absolute closest obstacle 
    #     currently visible in the filtered LiDAR scan.
    #     """
    #     with self._nav_lock:
    #         # Safely grab the correct TA array
    #         obstacles = getattr(self, '_obstacles_mm', None)
    #         if obstacles is None or len(obstacles) == 0:
    #             return None, None
    #         min_dist = float('inf')
    #         best_angle = 0.0
    #         # _obstacles_mm contains [x, y] coordinates
    #         for pt in obstacles:
    #             x, y = pt[0], pt[1]
    #             # Cartesian to Polar conversion
    #             dist = math.hypot(x, y)               
    #             if dist < min_dist:
    #                 min_dist = dist
    #                 # atan2(y, x) gives the angle in radians, convert to degrees
    #                 best_angle = math.degrees(math.atan2(y, x))         
    #         if min_dist == float('inf'):
    #             return None, None              
    #         return min_dist, best_angle
    
    # =======================================================================================

    def _nav_follow_pp_path(
        self,
        lookahead_distance: float,
        max_linear_speed: float,
        max_angular_speed: float,
        goal_tolerance: float,
        obstacles_range: float,
        view_angle: float,
        safe_dist: float,
        avoidance_delay: int,
        offset: float,
        alpha_Ld: float,
        x_L: float = 0.0,
        lane_width: float = 600.0,
        obstacle_avoidance: bool = True,
    ) -> None:

        from robot.path_planner import PurePursuitPlannerWithAvoidance
        self._planner = PurePursuitPlannerWithAvoidance(
            lookahead_distance=lookahead_distance,
            max_linear_speed=max_linear_speed,
            max_angular_speed=max_angular_speed,
            goal_tolerance=goal_tolerance,
            obstacles_range=obstacles_range,
            view_angle=view_angle,
            safe_dist=safe_dist,
            avoidance_delay=avoidance_delay,
            offset=offset,
            x_L=x_L,
            lane_width=lane_width,
            alpha_Ld=alpha_Ld,
            obstacle_avoidance=obstacle_avoidance, 
        )

    def _nav_follow_pp_path2(
        self,
        lookahead_distance: float,
        max_linear_speed: float,
        max_angular_speed: float,
        goal_tolerance: float,
        obstacles_range: float,
        safe_dist: float,
        max_turning_angle: float,
        avoidance_delay: int,
        obstacle_buffer_len: int,
        obstacle_buffer_delay: int,
        alpha_Ld: float,
        alpha_Sd: float,
        alpha_angle: float,
        obstacle_avoidance: bool = True,
    ) -> None:

        from robot.path_planner import PurePursuitPlannerWithAvoidance2
        self._planner = PurePursuitPlannerWithAvoidance2(
            lookahead_distance=lookahead_distance,
            max_linear_speed=max_linear_speed,
            max_angular_speed=max_angular_speed,
            goal_tolerance=goal_tolerance,
            obstacles_range=obstacles_range,
            safe_dist=safe_dist,
            max_turning_angle=max_turning_angle,
            avoidance_delay=avoidance_delay,
            obstacle_buffer_len=obstacle_buffer_len,
            obstacle_buffer_delay=obstacle_buffer_delay,
            alpha_Ld=alpha_Ld,
            alpha_Sd=alpha_Sd,
            alpha_angle=alpha_angle,
            obstacle_avoidance=obstacle_avoidance,
        )
    
    def _nav_follow_pp_path_loop(self):
        with self._lock:
            obstacles = self._obstacles_mm.copy()

            pose = self._pose
            # vel = self._vel

        # virtual obstacles
        # obstacles = np.float64([(0.0,1000.0),])
        # obstacles = (obstacles-np.float64([[pose[0], pose[1]]]))
        # obstacles = (np.array([[np.cos(pose[2]), -np.sin(pose[2])], [np.sin(pose[2]), np.cos(pose[2])]]).T @ obstacles.T).T # obstacles in robot frame

        v, w = self._planner.compute_velocity(pose, obstacles)
        print(f"Computed velocity: linear={v:.1f} mm/s, angular={math.degrees(w):.1f} deg/s")
        self.set_velocity(v, math.degrees(w))
        # print(f"Current Pose: ({pose[0]:.1f}, {pose[1]:.1f}, {math.degrees(pose[2]):.1f} deg)")

        if self._planner.TargetReached(self._planner.remaining_path, pose[0], pose[1]):
            print("MOVING: Target reached! Stopping.")
            print(self._planner.remaining_path, pose[0], pose[1])
            self.stop()
            return "IDLE"

        return "MOVING"

    def _draw_lidar_obstacles(self):
        import matplotlib.pyplot as plt
        with self._lock:
             obstacles_mm = self._obstacles_mm.copy()
             pose = self._pose
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.clear()

        if obstacles_mm.size != 0:
            # lidar orientation due to installation is 180 deg rotated from robot forward, so rotate obstacles accordingly
            lidar_offset_mm = 100.0
            obstacles = (np.array([[np.cos(np.pi), -np.sin(np.pi)], [np.sin(np.pi), np.cos(np.pi)]]) @ obstacles_mm.T).T + np.array([[lidar_offset_mm, 0],])
            # since some robot parts (e.g., the arm) may cause obstacles to be detected, we can filter out those obstacles behind the lidar.
            obstacles = obstacles[obstacles[:,0] > 0,:]
            # transform obstacles from robot frame to world frame.
            obstacles = (np.array([[np.cos(pose[2]), -np.sin(pose[2])], [np.sin(pose[2]), np.cos(pose[2])]]) @ obstacles.T).T + np.array([[pose[0], pose[1]],])
            self.ax.scatter(obstacles[:, 0] / 1000.0, obstacles[:, 1] / 1000.0, s=5)
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("LiDAR Scan")
        self.ax.set_aspect("equal")
        plt.grid()
        plt.draw()
        plt.pause(0.001)
        plt.savefig("/data/plot.png")
        plt.close()

    # =========================================================================
    # Internal — blocking waits for actuator completion
    # =========================================================================

    def _wait_dc_position(
        self, motor_id: int, target: int, tolerance: int, timeout: float
    ) -> bool:
        motor_idx = motor_id - 1
        deadline = time.monotonic() + timeout if timeout else None
        while True:
            with self._lock:
                dc = self._dc_state
            if dc is not None and abs(dc.motors[motor_idx].position - target) <= tolerance:
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.02)

    def _wait_dc_not_homing(self, motor_id: int, timeout: float) -> bool:
        """Wait until a DC motor's mode is no longer 4 (homing)."""
        motor_idx = motor_id - 1
        deadline = time.monotonic() + timeout if timeout else None
        time.sleep(0.1)  # Give firmware time to start the homing move
        while True:
            with self._lock:
                dc = self._dc_state
            if dc is not None and dc.motors[motor_idx].mode != int(DCMotorMode.HOMING):
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.02)

    def _wait_stepper_idle(self, stepper_id: int, timeout: float) -> bool:
        """Wait until stepper motion_state == 0 (idle)."""
        stepper_idx = stepper_id - 1
        deadline = time.monotonic() + timeout if timeout else None
        time.sleep(0.1)  # Give firmware time to start moving
        while True:
            with self._lock:
                step = self._step_state
            if step is not None and step.steppers[stepper_idx].motion_state == int(StepperMotionState.IDLE):
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.02)

    # =========================================================================
    # Internal — low-level helpers
    # =========================================================================

    def _get_pose_mm(self) -> tuple[float, float, float]:
        """Return fused (x_mm, y_mm, theta_rad) without unit conversion."""
        with self._lock:
            return (self._fused_x_mm, self._fused_y_mm, self._fused_theta)

    def _get_obstacles_mm(self) -> list[tuple[float, float]]:
        """Return cached and provider-supplied APF obstacles in robot-frame millimeters."""
        with self._lock:
            cached = list(self._obstacles_mm)
            provider = self._obstacle_provider

        dynamic: list[tuple[float, float]] = []
        if provider is not None:
            try:
                provided = provider() or []
                dynamic = [
                    (
                        self._require_finite_float("obstacle_x_mm", obs_x),
                        self._require_finite_float("obstacle_y_mm", obs_y),
                    )
                    for obs_x, obs_y in provided
                ]
            except Exception as exc:
                self._node.get_logger().error(f"obstacle provider failed: {exc}")

        return cached + dynamic

    @staticmethod
    def _require_id(name: str, value: int, low: int, high: int) -> int:
        value = int(value)
        if not low <= value <= high:
            raise ValueError(f"{name} must be between {low} and {high}, got {value}")
        return value

    @staticmethod
    def _require_enum(name: str, value: int, enum_type: type[IntEnum]) -> int:
        try:
            return int(enum_type(int(value)))
        except (TypeError, ValueError) as exc:
            valid = ", ".join(f"{member.name}={int(member)}" for member in enum_type)
            raise ValueError(f"{name} must be one of: {valid}") from exc

    @staticmethod
    def _require_positive_float(name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive number, got {value}")
        return value

    @staticmethod
    def _require_finite_float(name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite, got {value}")
        return value

    def _apply_odom_param_snapshot(
        self,
        wheel_diameter_mm: float,
        wheel_base_mm: float,
        initial_theta_deg: float,
        left_motor_number: int,
        left_motor_dir_inverted: bool,
        right_motor_number: int,
        right_motor_dir_inverted: bool,
    ) -> None:
        with self._lock:
            self._wheel_diameter = self._require_positive_float("wheel_diameter_mm", wheel_diameter_mm)
            self._wheel_base = self._require_positive_float("wheel_base_mm", wheel_base_mm)
            self._initial_theta_deg = self._require_finite_float("initial_theta_deg", initial_theta_deg)
            self._left_wheel_motor = self._require_id("left_motor_number", left_motor_number, 1, 4)
            self._right_wheel_motor = self._require_id("right_motor_number", right_motor_number, 1, 4)
            if self._left_wheel_motor == self._right_wheel_motor:
                raise ValueError("left and right odometry motors must be different")
            self._left_wheel_dir_inverted = bool(left_motor_dir_inverted)
            self._right_wheel_dir_inverted = bool(right_motor_dir_inverted)
            self._ticks_per_mm = self._encoder_ppr / (math.pi * self._wheel_diameter)

    def _update_odometry_params(
        self,
        *,
        wheel_diameter_mm: float | None = None,
        wheel_base_mm: float | None = None,
        initial_theta_deg: float | None = None,
        left_wheel_motor: int | None = None,
        left_wheel_dir_inverted: bool | None = None,
        right_wheel_motor: int | None = None,
        right_wheel_dir_inverted: bool | None = None,
    ) -> None:
        with self._lock:
            next_wheel_diameter = self._wheel_diameter if wheel_diameter_mm is None else \
                self._require_positive_float("wheel_diameter_mm", wheel_diameter_mm)
            next_wheel_base = self._wheel_base if wheel_base_mm is None else \
                self._require_positive_float("wheel_base_mm", wheel_base_mm)
            next_initial_theta = self._initial_theta_deg if initial_theta_deg is None else \
                self._require_finite_float("initial_theta_deg", initial_theta_deg)
            next_left_motor = self._left_wheel_motor if left_wheel_motor is None else \
                self._require_id("left_motor_id", left_wheel_motor, 1, 4)
            next_right_motor = self._right_wheel_motor if right_wheel_motor is None else \
                self._require_id("right_motor_id", right_wheel_motor, 1, 4)
            if next_left_motor == next_right_motor:
                raise ValueError("left and right odometry motors must be different")

            next_left_inverted = self._left_wheel_dir_inverted if left_wheel_dir_inverted is None else \
                bool(left_wheel_dir_inverted)
            next_right_inverted = self._right_wheel_dir_inverted if right_wheel_dir_inverted is None else \
                bool(right_wheel_dir_inverted)

        self._apply_odom_param_snapshot(
            next_wheel_diameter,
            next_wheel_base,
            next_initial_theta,
            next_left_motor,
            next_left_inverted,
            next_right_motor,
            next_right_inverted,
        )

        snapshot = (
            next_wheel_diameter,
            next_wheel_base,
            next_initial_theta,
            next_left_motor,
            next_left_inverted,
            next_right_motor,
            next_right_inverted,
        )

        self._publish_odom_params(snapshot)

    def _publish_odom_params(
        self,
        snapshot: tuple[float, float, float, int, bool, int, bool],
    ) -> None:
        msg = SysOdomParamSet()
        (
            msg.wheel_diameter_mm,
            msg.wheel_base_mm,
            msg.initial_theta_deg,
            msg.left_motor_number,
            msg.left_motor_dir_inverted,
            msg.right_motor_number,
            msg.right_motor_dir_inverted,
        ) = snapshot
        self._odom_param_pub.publish(msg)

    def _send_body_velocity_mm(self, linear_mm_s: float, angular_rad_s: float) -> None:
        """Diff-drive mixing and publish. All values in mm/s and rad/s."""
        with self._lock:
            half_wb = self._wheel_base / 2.0
            left_motor = self._left_wheel_motor
            right_motor = self._right_wheel_motor
            left_dir_inverted = self._left_wheel_dir_inverted
            right_dir_inverted = self._right_wheel_dir_inverted
        if linear_mm_s or angular_rad_s:
            self._ensure_drive_motors_enabled()
        left_velocity = linear_mm_s - angular_rad_s * half_wb
        right_velocity = linear_mm_s + angular_rad_s * half_wb
        if left_dir_inverted:
            left_velocity = -left_velocity
        if right_dir_inverted:
            right_velocity = -right_velocity
        self._send_motor_velocity_mm(left_motor, left_velocity)
        self._send_motor_velocity_mm(right_motor, right_velocity)

    def _send_motor_velocity_mm(self, motor_id: int, velocity_mm_s: float) -> None:
        msg = DCSetVelocity()
        msg.motor_number = motor_id
        msg.target_ticks = int(velocity_mm_s * self._ticks_per_mm)
        self._dc_vel_pub.publish(msg)

    def _ensure_drive_motors_enabled(self) -> None:
        """Enable the configured drive motors in velocity mode when needed."""
        with self._lock:
            drive_motors = tuple(dict.fromkeys((self._left_wheel_motor, self._right_wheel_motor)))
            dc_state = self._dc_state

        if dc_state is None or not hasattr(dc_state, "motors"):
            for motor_id in drive_motors:
                self.enable_motor(motor_id, DCMotorMode.VELOCITY)
            return

        for motor_id in drive_motors:
            motor_index = motor_id - 1
            try:
                current_mode = dc_state.motors[motor_index].mode
            except (AttributeError, IndexError, TypeError):
                current_mode = None
            if current_mode != int(DCMotorMode.VELOCITY):
                self.enable_motor(motor_id, DCMotorMode.VELOCITY)


# =============================================================================
# Module-level helpers
# =============================================================================

def _dist2d(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def _wrap_angle(a: float) -> float:
    """Wrap angle to [−π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi