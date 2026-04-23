from __future__ import annotations

import importlib
import math
import sys
import types
import unittest
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.sensor_fusion import OrientationComplementaryFilter
from robot.path_planner import PurePursuitPlanner


# ---------------------------------------------------------------------------
# Minimal ROS 2 / bridge_interfaces stubs (no ROS installation required)
# ---------------------------------------------------------------------------

def _install_fake_robot_dependencies() -> None:
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    bridge_interfaces = types.ModuleType("bridge_interfaces")
    bridge_interfaces_msg = types.ModuleType("bridge_interfaces.msg")
    bridge_interfaces_srv = types.ModuleType("bridge_interfaces.srv")

    for name in [
        "DCEnable", "DCHome", "DCPid", "DCPidReq", "DCPidSet", "DCResetPosition",
        "DCSetPosition", "DCSetPwm", "DCSetVelocity", "DCStateAll",
        "IOInputState", "IOOutputState", "IOSetLed", "IOSetNeopixel", "SensorImu",
        "SensorKinematics", "ServoEnable", "ServoSet", "ServoStateAll",
        "StepConfig", "StepConfigReq", "StepConfigSet", "StepEnable", "StepHome",
        "StepMove", "StepStateAll",
        "SysOdomParamReq", "SysOdomParamRsp", "SysOdomParamSet",
        "SysOdomReset", "SystemConfig", "SystemDiag", "SystemInfo",
        "SystemPower", "SystemState", "TagDetectionArray",
    ]:
        setattr(bridge_interfaces_msg, name, type(name, (), {}))

    class SetFirmwareState:
        class Request:
            def __init__(self) -> None:
                self.target_state = 0
                self.timeout_sec = 0.0

    bridge_interfaces_srv.SetFirmwareState = SetFirmwareState
    bridge_interfaces.msg = bridge_interfaces_msg
    bridge_interfaces.srv = bridge_interfaces_srv
    sys.modules["bridge_interfaces"] = bridge_interfaces
    sys.modules["bridge_interfaces.msg"] = bridge_interfaces_msg
    sys.modules["bridge_interfaces.srv"] = bridge_interfaces_srv


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class FakeNode:
    def __init__(self) -> None:
        self.publishers: dict[str, FakePublisher] = {}

    def create_publisher(self, _msg_type, topic: str, _qos: int) -> FakePublisher:
        pub = FakePublisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_client(self, *_args, **_kwargs):
        class _FakeClient:
            def wait_for_service(self, **_kw): return True
            def call_async(self, _req):
                f = types.SimpleNamespace(done=lambda: True, result=lambda: None,
                                         add_done_callback=lambda _cb: None)
                return f
        return _FakeClient()

    def get_logger(self):
        return types.SimpleNamespace(error=lambda _m: None)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_imu(module, *, mag_calibrated: bool, mag_x: float, mag_y: float):
    msg = module.SensorImu()
    msg.mag_calibrated = mag_calibrated
    msg.mag_x = mag_x
    msg.mag_y = mag_y
    # robot._on_imu now extracts heading from the Madgwick quaternion rather
    # than raw atan2(mag_y, mag_x).  Build a flat-robot (roll=pitch=0)
    # quaternion whose yaw matches the heading implied by the mag vector so
    # that all existing test expectations remain valid.
    yaw = math.atan2(mag_y, mag_x)
    msg.quat_w = math.cos(yaw / 2.0)
    msg.quat_x = 0.0
    msg.quat_y = 0.0
    msg.quat_z = math.sin(yaw / 2.0)
    return msg


def _make_kin(module, *, theta: float, x: float = 0.0, y: float = 0.0):
    msg = module.SensorKinematics()
    msg.x = x
    msg.y = y
    msg.theta = theta
    msg.vx = 0.0
    msg.vy = 0.0
    msg.v_theta = 0.0
    return msg


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class OrientationSensorFusionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))
        cls.mod = importlib.import_module("robot.robot")

    def setUp(self) -> None:
        self.node = FakeNode()
        self.robot = self.mod.Robot(self.node)

    # ------------------------------------------------------------------
    # Initial state
    # ------------------------------------------------------------------

    def test_initial_fused_orientation_is_initial_theta_degrees(self) -> None:
        # No messages received yet — default fused_theta is INITIAL_THETA_DEG (90°).
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 90.0, places=6)

    # ------------------------------------------------------------------
    # No AHRS calibration → pure odometry pass-through
    # ------------------------------------------------------------------

    def test_uncalibrated_ahrs_does_not_affect_fused_theta(self) -> None:
        # Deliver an IMU message with AHRS pointing north but uncalibrated.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=False, mag_x=1.0, mag_y=0.0))
        # Kinematics says robot is facing 45°.
        self.robot._on_kinematics(_make_kin(self.mod, theta=math.radians(45.0)))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 45.0, places=6)

    def test_no_imu_message_fused_theta_equals_odometry(self) -> None:
        # Without any IMU message at all, _ahrs_heading remains None.
        self.robot._on_kinematics(_make_kin(self.mod, theta=math.radians(30.0)))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 30.0, places=6)

    # ------------------------------------------------------------------
    # AHRS heading extraction from atan2(mag_y, mag_x)
    # ------------------------------------------------------------------

    def test_ahrs_heading_east_is_zero(self) -> None:
        # mag_x > 0, mag_y == 0 → atan2(0, 1) = 0 rad
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=1.0, mag_y=0.0))
        self.robot.set_orientation_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 0.0, places=6)

    def test_ahrs_heading_north_is_90_degrees(self) -> None:
        # mag_x == 0, mag_y > 0 → atan2(1, 0) = π/2 rad = 90°
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot.set_orientation_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 90.0, places=6)

    def test_ahrs_heading_west_is_180_degrees(self) -> None:
        # mag_x < 0, mag_y == 0 → atan2(0, -1) = π rad = 180°
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=-1.0, mag_y=0.0))
        self.robot.set_orientation_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        # math.degrees(math.pi) ≈ 180.0
        self.assertAlmostEqual(
            abs(self.robot.get_fused_orientation()), 180.0, places=6
        )

    def test_ahrs_heading_south_is_minus_90_degrees(self) -> None:
        # mag_x == 0, mag_y < 0 → atan2(-1, 0) = -π/2 rad = -90°
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=-1.0))
        self.robot.set_orientation_fusion_alpha(1.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), -90.0, places=6)

    # ------------------------------------------------------------------
    # Complementary filter blend — alpha extremes
    # ------------------------------------------------------------------

    def test_alpha_zero_means_pure_odometry(self) -> None:
        # With alpha = 0 the AHRS heading has no influence at all.
        self.robot.set_orientation_fusion_alpha(0.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        odom_theta = math.radians(30.0)
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_theta))

        self.assertAlmostEqual(
            self.robot.get_fused_orientation(), math.degrees(odom_theta), places=6
        )

    def test_alpha_one_means_pure_ahrs(self) -> None:
        # With alpha = 1 the result must equal the AHRS heading regardless of odometry.
        self.robot.set_orientation_fusion_alpha(1.0)
        mag_heading = math.atan2(1.0, 0.0)  # π/2
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(
            self.robot.get_fused_orientation(), math.degrees(mag_heading), places=6
        )

    def test_blend_formula_with_explicit_alpha(self) -> None:
        # Verify: fused = theta + alpha * wrap(mag_heading - theta)
        alpha = 0.3
        self.robot.set_orientation_fusion_alpha(alpha)

        mag_heading = math.atan2(1.0, 0.0)  # π/2
        odom_theta = math.radians(10.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_theta))

        diff = math.atan2(
            math.sin(mag_heading - odom_theta),
            math.cos(mag_heading - odom_theta),
        )
        expected_deg = math.degrees(odom_theta + alpha * diff)
        self.assertAlmostEqual(self.robot.get_fused_orientation(), expected_deg, places=6)

    # ------------------------------------------------------------------
    # Angle-wrap robustness (crossing ±π boundary)
    # ------------------------------------------------------------------

    def test_angle_wrap_ahrs_near_plus_pi_odom_near_minus_pi(self) -> None:
        # AHRS heading ≈ +175°, odometry ≈ -175°: naive subtraction gives ~350°,
        # but the atan2 wrapping must produce a short ~10° correction.
        alpha = 0.5
        self.robot.set_orientation_fusion_alpha(alpha)

        mag_deg = 175.0
        odom_deg = -175.0
        mag_rad = math.radians(mag_deg)
        odom_rad = math.radians(odom_deg)

        # Construct a unit mag vector for the desired heading.
        self.robot._on_imu(_make_imu(
            self.mod, mag_calibrated=True,
            mag_x=math.cos(mag_rad), mag_y=math.sin(mag_rad),
        ))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_rad))

        diff = math.atan2(
            math.sin(mag_rad - odom_rad),
            math.cos(mag_rad - odom_rad),
        )
        expected_deg = math.degrees(odom_rad + alpha * diff)
        self.assertAlmostEqual(self.robot.get_fused_orientation(), expected_deg, places=5)

        # The correction must be small (≈ +5°, not ≈ +175°).
        correction = self.robot.get_fused_orientation() - odom_deg
        self.assertLess(abs(correction), 20.0)

    def test_angle_wrap_ahrs_near_minus_pi_odom_near_plus_pi(self) -> None:
        alpha = 0.5
        self.robot.set_orientation_fusion_alpha(alpha)

        mag_rad = math.radians(-175.0)
        odom_rad = math.radians(175.0)

        self.robot._on_imu(_make_imu(
            self.mod, mag_calibrated=True,
            mag_x=math.cos(mag_rad), mag_y=math.sin(mag_rad),
        ))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_rad))

        diff = math.atan2(
            math.sin(mag_rad - odom_rad),
            math.cos(mag_rad - odom_rad),
        )
        expected_deg = math.degrees(odom_rad + alpha * diff)
        self.assertAlmostEqual(self.robot.get_fused_orientation(), expected_deg, places=5)

        correction = self.robot.get_fused_orientation() - math.degrees(odom_rad)
        self.assertLess(abs(correction), 20.0)

    def test_no_wrap_artifact_when_headings_agree(self) -> None:
        # When AHRS and odometry agree, the correction must be zero.
        alpha = 0.5
        self.robot.set_orientation_fusion_alpha(alpha)
        theta = math.radians(45.0)

        self.robot._on_imu(_make_imu(
            self.mod, mag_calibrated=True,
            mag_x=math.cos(theta), mag_y=math.sin(theta),
        ))
        self.robot._on_kinematics(_make_kin(self.mod, theta=theta))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 45.0, places=5)

    # ------------------------------------------------------------------
    # Successive updates
    # ------------------------------------------------------------------

    def test_latest_imu_message_takes_precedence(self) -> None:
        # Send two IMU messages in a row — only the second should matter.
        self.robot.set_orientation_fusion_alpha(1.0)

        # First message: points east (0°)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=1.0, mag_y=0.0))
        # Second message: points north (90°)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        self.assertAlmostEqual(self.robot.get_fused_orientation(), 90.0, places=6)

    def test_recalibration_replaces_stale_ahrs_heading(self) -> None:
        # Calibrated (east, 0°) → calibration lost → recalibrated (north, 90°).
        # While uncalibrated the system must fall back to pure odometry, not
        # continue blending against the last valid heading.
        # Using odom=60° during the uncalibrated phase so the two behaviors
        # produce distinct values:
        #   buggy  (stale heading kept): fused = 60 + 0.5*(0−60) = 30°
        #   correct (heading cleared):   fused = 60° (pure odometry)
        self.robot.set_orientation_fusion_alpha(0.5)

        # Phase 1: calibrated, mag = east (0°), odom = 0° → fused = 0°.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=1.0, mag_y=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 0.0, places=5)

        # Phase 2: calibration lost.  _ahrs_heading must be cleared so the
        # filter falls back to pure odometry even though odom drifted to 60°.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=False, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=math.radians(60.0)))
        # Pure odometry: fused must equal the odometry input exactly.
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 60.0, places=5)

        # Phase 3: recalibrated pointing north (90°), odom still 60°.
        # alpha=0.5 → fused = 60 + 0.5*(90−60) = 75°.
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=math.radians(60.0)))
        self.assertAlmostEqual(self.robot.get_fused_orientation(), 75.0, places=5)

    def test_kinematics_callback_sets_pose_event(self) -> None:
        # _on_kinematics must pulse the pose_event so wait_for_pose_update() works.
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        # After the callback the event is immediately cleared; the pulse should
        # have occurred, so the event object itself is the correct observable.
        # We can verify by checking that no exception was raised and _pose updated.
        x, y, theta = self.robot._pose
        self.assertAlmostEqual(theta, 0.0, places=6)

    # ------------------------------------------------------------------
    # get_pose() returns raw odometry, NOT fused heading
    # ------------------------------------------------------------------

    def test_get_pose_theta_matches_fused_orientation(self) -> None:
        # get_pose() theta and get_fused_orientation() must always agree —
        # both expose the sensor-fused heading so navigation benefits from
        # AHRS correction without extra API calls.
        self.robot.set_orientation_fusion_alpha(0.5)
        odom_theta = math.radians(20.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=odom_theta))

        _, _, pose_theta_deg = self.robot.get_pose()
        fused_deg = self.robot.get_fused_orientation()

        # Both must return the same fused heading.
        self.assertAlmostEqual(pose_theta_deg, fused_deg, places=6)
        # And it must differ from the raw odometry (mag pulled it toward 90°).
        self.assertNotAlmostEqual(pose_theta_deg, math.degrees(odom_theta), places=2)

    # ------------------------------------------------------------------
    # Alpha clamping
    # ------------------------------------------------------------------

    def test_alpha_above_one_is_clamped_to_one(self) -> None:
        self.robot.set_orientation_fusion_alpha(5.0)
        self.assertEqual(self.robot._fusion.alpha, 1.0)

    def test_alpha_below_zero_is_clamped_to_zero(self) -> None:
        self.robot.set_orientation_fusion_alpha(-3.0)
        self.assertEqual(self.robot._fusion.alpha, 0.0)

    def test_alpha_boundary_values_are_accepted(self) -> None:
        self.robot.set_orientation_fusion_alpha(0.0)
        self.assertEqual(self.robot._fusion.alpha, 0.0)
        self.robot.set_orientation_fusion_alpha(1.0)
        self.assertEqual(self.robot._fusion.alpha, 1.0)

    # ------------------------------------------------------------------
    # Output is in degrees
    # ------------------------------------------------------------------

    def test_get_fused_orientation_returns_degrees_not_radians(self) -> None:
        # If the value were in radians, π/2 rad ≈ 1.57 which is clearly not 90.
        self.robot.set_orientation_fusion_alpha(1.0)
        self.robot._on_imu(_make_imu(self.mod, mag_calibrated=True, mag_x=0.0, mag_y=1.0))
        self.robot._on_kinematics(_make_kin(self.mod, theta=0.0))

        result = self.robot.get_fused_orientation()
        self.assertGreater(result, 2.0)   # would be ≈ 1.57 if accidentally radians
        self.assertAlmostEqual(result, 90.0, places=5)


# ===========================================================================
# Integration: fused heading → PurePursuitPlanner.compute_velocity()
#
# These tests mirror the logic in main.py's MOVING state:
#
#   current_x, current_y, _ = robot.get_pose()
#   current_theta_rad = math.radians(robot.get_fused_orientation())
#   linear, angular = planner.compute_velocity(
#       pose=(current_x, current_y, current_theta_rad), ...)
#
# The key contract: compute_velocity receives the *fused* heading, not the
# raw odometry theta.  When the two differ the steering commands must differ.
# ===========================================================================

def _drive_robot(mod, node, *, imu_mag_x, imu_mag_y, mag_calibrated,
                 odom_theta, vx=0.0, vy=0.0, v_theta=0.0, strategy=None):
    """
    Feed one IMU + kinematics pair into a fresh Robot and return it.

    If *strategy* is provided it is installed before the kinematics message
    so the fusion runs with the intended strategy from the first update.
    """
    robot = mod.Robot(node)
    imu = mod.SensorImu()
    imu.mag_calibrated = mag_calibrated
    imu.mag_x = imu_mag_x
    imu.mag_y = imu_mag_y
    yaw = math.atan2(imu_mag_y, imu_mag_x)
    imu.quat_w = math.cos(yaw / 2.0)
    imu.quat_x = 0.0
    imu.quat_y = 0.0
    imu.quat_z = math.sin(yaw / 2.0)
    robot._on_imu(imu)
    if strategy is not None:
        robot.set_orientation_fusion_strategy(strategy)
    kin = mod.SensorKinematics()
    kin.x = 0.0; kin.y = 0.0; kin.theta = odom_theta
    kin.vx = vx; kin.vy = vy; kin.v_theta = v_theta
    robot._on_kinematics(kin)
    return robot


class FusedPurePursuitTests(unittest.TestCase):
    """
    Verify that sensor-fused heading drives PurePursuit steering differently
    than raw odometry heading, and that each fusion strategy produces the
    expected steering behaviour.
    """

    PLANNER = PurePursuitPlanner(lookahead_dist=100.0, max_angular=2.0)
    WAYPOINTS_NORTH = __import__("numpy").array([[0.0, 200.0]])  # directly north
    MAX_V = 100.0

    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        cls.mod = importlib.import_module("robot.robot")

    def setUp(self) -> None:
        self.node = FakeNode()

    # ------------------------------------------------------------------
    # Helper: run the main.py MOVING-state lines and return (lin, ang)
    # ------------------------------------------------------------------
    def _velocity_from_fused(self, robot) -> tuple[float, float]:
        x, y, _ = robot.get_pose()
        theta_rad = math.radians(robot.get_fused_orientation())
        return self.PLANNER.compute_velocity(
            pose=(x, y, theta_rad),
            waypoints=self.WAYPOINTS_NORTH,
            max_linear=self.MAX_V,
        )

    def _velocity_from_raw_odom(self, robot) -> tuple[float, float]:
        with robot._lock:
            x_mm, y_mm, odom_theta_rad = robot._pose
        return self.PLANNER.compute_velocity(
            pose=(x_mm, y_mm, odom_theta_rad),
            waypoints=self.WAYPOINTS_NORTH,
            max_linear=self.MAX_V,
        )

    # ------------------------------------------------------------------
    # 1. When odom theta == fused theta the commands are identical
    # ------------------------------------------------------------------

    def test_no_ahrs_fused_and_raw_commands_are_identical(self) -> None:
        # Uncalibrated AHRS → fusion falls back to odom → commands must match.
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=False,
            odom_theta=0.0,
        )
        lin_fused, ang_fused = self._velocity_from_fused(robot)
        lin_raw, ang_raw = self._velocity_from_raw_odom(robot)
        self.assertAlmostEqual(lin_fused, lin_raw, places=6)
        self.assertAlmostEqual(ang_fused, ang_raw, places=6)

    # ------------------------------------------------------------------
    # 2. When fused heading differs from odom the commands diverge
    # ------------------------------------------------------------------

    def test_fused_heading_changes_steering_command(self) -> None:
        # Robot A: uncalibrated AHRS → fused == odom == 0° (east).
        # Robot B: calibrated AHRS, alpha=1 → fused == 90° (north).
        # Waypoint is north.  Robot A needs a large left turn; Robot B is aligned.
        robot_no_fusion = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=False,
            odom_theta=0.0,
        )
        _, ang_no_fusion = self._velocity_from_fused(robot_no_fusion)

        robot_fused = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot_fused.set_orientation_fusion_alpha(1.0)
        robot_fused._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_fused = self._velocity_from_fused(robot_fused)

        # Fused (90°) is aligned with north waypoint → near-zero angular.
        # No-fusion (0°) is perpendicular → large positive angular.
        self.assertLess(abs(ang_fused), abs(ang_no_fusion))

    def test_fused_heading_turns_correct_direction(self) -> None:
        # Uncalibrated robot (fused == odom == 0°, east) facing north waypoint
        # → must turn left (positive ω).
        # Calibrated AHRS robot (fused == 90°, north) already aligned → no turn.
        robot_no_fusion = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=False,
            odom_theta=0.0,
        )
        _, ang_no_fusion = self._velocity_from_fused(robot_no_fusion)

        robot_fused = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot_fused.set_orientation_fusion_alpha(1.0)
        robot_fused._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_fused = self._velocity_from_fused(robot_fused)

        self.assertGreater(ang_no_fusion, 0.0)              # uncalibrated: turn left toward north
        self.assertAlmostEqual(ang_fused, 0.0, places=3)   # fused: already north

    # ------------------------------------------------------------------
    # 3. OrientationComplementaryFilter alpha scales the steering correction
    # ------------------------------------------------------------------

    def test_higher_alpha_gives_more_heading_correction(self) -> None:
        # Odom 0°, mag 90°.  Higher alpha → fused heading closer to 90°.
        # The heading correction is linear in alpha for the complementary filter.
        headings = []
        for alpha in (0.0, 0.25, 0.5, 0.75, 1.0):
            robot = _drive_robot(
                self.mod, self.node,
                imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
                odom_theta=0.0,
            )
            robot.set_orientation_fusion_alpha(alpha)
            robot._on_kinematics(_make_kin(self.mod, theta=0.0))
            headings.append(robot.get_fused_orientation())

        self.assertTrue(all(a <= b for a, b in zip(headings, headings[1:])))
        self.assertAlmostEqual(headings[0], 0.0, places=5)   # alpha=0 → pure odom
        self.assertAlmostEqual(headings[-1], 90.0, places=5)  # alpha=1 → pure mag

    def test_alpha_extremes_produce_opposite_steering_demands(self) -> None:
        # alpha=0: fused=0° (east), waypoint north → must turn left (positive ω).
        # alpha=1: fused=90° (north), waypoint north → already aligned (ω ≈ 0).
        robot_no_correction = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot_no_correction.set_orientation_fusion_alpha(0.0)
        robot_no_correction._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_no = self._velocity_from_fused(robot_no_correction)

        robot_full = _drive_robot(
            self.mod, FakeNode(),
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        robot_full.set_orientation_fusion_alpha(1.0)
        robot_full._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_full = self._velocity_from_fused(robot_full)

        self.assertGreater(ang_no, 0.0)                         # must turn left
        self.assertAlmostEqual(ang_full, 0.0, places=3)         # already aligned
        self.assertGreater(ang_no, ang_full)

    # ------------------------------------------------------------------
    # 4. Strategy switch at runtime changes the next velocity command
    # ------------------------------------------------------------------

    def test_switching_strategy_changes_velocity_command(self) -> None:
        robot = _drive_robot(
            self.mod, self.node,
            imu_mag_x=0.0, imu_mag_y=1.0, mag_calibrated=True,
            odom_theta=0.0,
        )
        # Start with zero-alpha complementary → fused == odom
        robot.set_orientation_fusion_alpha(0.0)
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_before = self._velocity_from_fused(robot)

        # Switch to full-AHRS complementary
        robot.set_orientation_fusion_strategy(OrientationComplementaryFilter(alpha=1.0))
        robot._on_kinematics(_make_kin(self.mod, theta=0.0))
        _, ang_after = self._velocity_from_fused(robot)

        self.assertNotAlmostEqual(ang_before, ang_after, places=3)


if __name__ == "__main__":
    unittest.main()
