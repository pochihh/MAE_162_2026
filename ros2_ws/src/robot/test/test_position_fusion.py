"""
test_position_fusion.py — unit tests for GPS-anchored position fusion
======================================================================
Tests cover:
  - No GPS: fused falls back to raw odometry
  - GPS active: complementary filter blends GPS toward fused position
  - GPS offset: arena-frame correction applied to raw tag detections
  - GPS stale: dead reckoning from last anchor rather than raw odometry
  - Anchor refresh: anchor updates on every fresh GPS tick
  - Alpha extremes: alpha=0 (pure odom) and alpha=1 (pure GPS)
  - API: set_gps_offset(), set_position_fusion_alpha() clamping, is_gps_active()

No ROS 2 installation is required — all dependencies are stubbed out.
"""

from __future__ import annotations

import importlib
import math
import sys
import time
import types
import unittest
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))


# ---------------------------------------------------------------------------
# Minimal ROS 2 / bridge_interfaces stubs
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
        self.warnings: list[str] = []

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
                f = types.SimpleNamespace(
                    done=lambda: True, result=lambda: None,
                    add_done_callback=lambda _cb: None,
                )
                return f
        return _FakeClient()

    def get_logger(self):
        node = self
        return types.SimpleNamespace(
            error=lambda _m, **_kw: None,
            warn=lambda msg, **_kw: node.warnings.append(msg),
            info=lambda _m, **_kw: None,
        )


# ---------------------------------------------------------------------------
# Message helpers
# ---------------------------------------------------------------------------

def _make_kin(module, *, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
    msg = module.SensorKinematics()
    msg.x = x
    msg.y = y
    msg.theta = theta
    msg.vx = 0.0
    msg.vy = 0.0
    msg.v_theta = 0.0
    return msg


def _make_tag(module, *, x_m: float, y_m: float, tag_id: int = 0):
    """Build a fake TagDetectionArray with one detection (coordinates in metres)."""
    msg = module.TagDetectionArray()
    msg.detections = [types.SimpleNamespace(x=x_m, y=y_m, tag_id=tag_id)]
    return msg


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class PositionFusionTests(unittest.TestCase):

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
        self.robot.set_gps_offset(0.0, 0.0)

    # ------------------------------------------------------------------
    # Initial state
    # ------------------------------------------------------------------

    def test_initial_fused_position_is_zero(self) -> None:
        x, y, _ = self.robot.get_fused_pose()
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_initial_gps_not_active(self) -> None:
        self.assertFalse(self.robot.is_gps_active())

    # ------------------------------------------------------------------
    # No GPS ever → pure odometry fallback
    # ------------------------------------------------------------------

    def test_no_gps_fused_equals_raw_odometry(self) -> None:
        self.robot._on_kinematics(_make_kin(self.mod, x=300.0, y=150.0))
        x, y, _ = self.robot.get_fused_pose()
        self.assertAlmostEqual(x, 300.0, places=4)
        self.assertAlmostEqual(y, 150.0, places=4)

    def test_no_gps_successive_moves_track_odometry(self) -> None:
        for odom_x in [0.0, 100.0, 250.0, 400.0]:
            self.robot._on_kinematics(_make_kin(self.mod, x=odom_x, y=0.0))
        x, y, _ = self.robot.get_fused_pose()
        self.assertAlmostEqual(x, 400.0, places=4)
        self.assertAlmostEqual(y, 0.0, places=4)

    # ------------------------------------------------------------------
    # GPS offset
    # ------------------------------------------------------------------

    def test_set_gps_offset_stores_values(self) -> None:
        self.robot.set_gps_offset(500.0, 250.0)
        self.assertAlmostEqual(self.robot._gps_offset_x_mm, 500.0, places=6)
        self.assertAlmostEqual(self.robot._gps_offset_y_mm, 250.0, places=6)

    def test_gps_offset_applied_to_tag_detection(self) -> None:
        # GPS reads (0.1 m, 0.05 m) = (100 mm, 50 mm) in GPS frame.
        # Offset is (400 mm, 200 mm) → arena coords should be (500 mm, 250 mm).
        self.robot.set_gps_offset(400.0, 200.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.1, y_m=0.05))
        self.assertAlmostEqual(self.robot._gps_x_mm, 500.0, places=4)
        self.assertAlmostEqual(self.robot._gps_y_mm, 250.0, places=4)

    def test_zero_offset_emits_warning(self) -> None:
        # Default offset (0, 0) must trigger a warning on first tag detection.
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.1, y_m=0.1))
        self.assertTrue(
            any("offset" in w.lower() or "GPS" in w for w in self.node.warnings),
            "Expected a GPS offset warning but none was logged",
        )

    def test_nonzero_offset_suppresses_warning(self) -> None:
        self.robot.set_gps_offset(100.0, 50.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.1, y_m=0.1))
        self.assertEqual(len(self.node.warnings), 0)

    # ------------------------------------------------------------------
    # Tag body offset rotation (body frame: +x forward, +y left)
    # ------------------------------------------------------------------

    def test_tag_body_offset_forward_maps_to_world_x_at_zero_heading(self) -> None:
        self.robot.set_tag_body_offset(100.0, 0.0)
        self.robot._fused_theta = 0.0

        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.3))

        # At heading 0: a tag 100 mm forward of the body origin sits at +X.
        # Robot center = tag position - rotated body offset.
        self.assertAlmostEqual(self.robot._gps_x_mm, 400.0, places=4)
        self.assertAlmostEqual(self.robot._gps_y_mm, 300.0, places=4)

    def test_tag_body_offset_left_maps_to_negative_world_x_at_ninety_degrees(self) -> None:
        self.robot.set_tag_body_offset(0.0, 100.0)
        self.robot._fused_theta = math.pi / 2.0

        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.3))

        # At heading +90 deg: body +Y (left) rotates to world -X.
        # Robot center = tag position - (-100, 0) = tag position + (100, 0).
        self.assertAlmostEqual(self.robot._gps_x_mm, 600.0, places=4)
        self.assertAlmostEqual(self.robot._gps_y_mm, 300.0, places=4)

    def test_tag_body_offset_forward_maps_to_world_y_at_ninety_degrees(self) -> None:
        self.robot.set_tag_body_offset(100.0, 0.0)
        self.robot._fused_theta = math.pi / 2.0

        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.3))

        # At heading +90 deg: body +X (forward) rotates to world +Y.
        # Robot center = tag position - (0, 100).
        self.assertAlmostEqual(self.robot._gps_x_mm, 500.0, places=4)
        self.assertAlmostEqual(self.robot._gps_y_mm, 200.0, places=4)

    # ------------------------------------------------------------------
    # GPS active — complementary filter blending
    # ------------------------------------------------------------------

    def test_alpha_one_fused_snaps_to_gps(self) -> None:
        # With alpha=1 and a GPS fix at (500, 300) mm arena frame (no offset),
        # the fused position must equal the GPS position regardless of odometry.
        self.robot.set_position_fusion_alpha(1.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.3))
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))
        x, y, _ = self.robot.get_fused_pose()
        self.assertAlmostEqual(x, 500.0, places=3)
        self.assertAlmostEqual(y, 300.0, places=3)

    def test_alpha_zero_fused_equals_odometry_despite_gps(self) -> None:
        self.robot.set_position_fusion_alpha(0.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.3))
        self.robot._on_kinematics(_make_kin(self.mod, x=100.0, y=50.0))
        x, y, _ = self.robot.get_fused_pose()
        self.assertAlmostEqual(x, 100.0, places=3)
        self.assertAlmostEqual(y, 50.0, places=3)

    def test_blend_formula_with_explicit_alpha(self) -> None:
        # fused = odom + alpha * (gps - odom)
        alpha = 0.3
        odom_x, odom_y = 100.0, 50.0
        gps_x_mm, gps_y_mm = 200.0, 150.0  # GPS frame = arena frame (no offset)

        self.robot.set_position_fusion_alpha(alpha)
        self.robot._on_tag_detections(
            _make_tag(self.mod, x_m=gps_x_mm / 1000.0, y_m=gps_y_mm / 1000.0)
        )
        self.robot._on_kinematics(_make_kin(self.mod, x=odom_x, y=odom_y))

        x, y, _ = self.robot.get_fused_pose()
        expected_x = odom_x + alpha * (gps_x_mm - odom_x)
        expected_y = odom_y + alpha * (gps_y_mm - odom_y)
        self.assertAlmostEqual(x, expected_x, places=3)
        self.assertAlmostEqual(y, expected_y, places=3)

    def test_gps_active_returns_true_after_fresh_fix(self) -> None:
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.2, y_m=0.1))
        self.assertTrue(self.robot.is_gps_active())

    # ------------------------------------------------------------------
    # Anchor updates
    # ------------------------------------------------------------------

    def test_anchor_set_after_first_gps_fix(self) -> None:
        self.assertFalse(self.robot._pos_fusion._anchor_valid)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.3, y_m=0.2))
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))
        self.assertTrue(self.robot._pos_fusion._anchor_valid)

    def test_anchor_values_match_fused_position(self) -> None:
        self.robot.set_position_fusion_alpha(1.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.4, y_m=0.2))
        self.robot._on_kinematics(_make_kin(self.mod, x=50.0, y=30.0))

        # With alpha=1 fused snaps to GPS; anchor must equal fused.
        x, y, _ = self.robot.get_fused_pose()
        self.assertAlmostEqual(self.robot._pos_fusion._anchor_x_mm, x, places=4)
        self.assertAlmostEqual(self.robot._pos_fusion._anchor_y_mm, y, places=4)

    def test_anchor_updates_on_each_gps_tick(self) -> None:
        self.robot.set_position_fusion_alpha(1.0)

        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.1, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))
        anchor_x_first = self.robot._pos_fusion._anchor_x_mm

        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.3, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=10.0, y=0.0))
        anchor_x_second = self.robot._pos_fusion._anchor_x_mm

        self.assertNotAlmostEqual(anchor_x_first, anchor_x_second, places=2)
        self.assertAlmostEqual(anchor_x_second, 300.0, places=3)

    # ------------------------------------------------------------------
    # Dead reckoning after GPS goes stale
    # ------------------------------------------------------------------

    def test_dead_reckoning_no_jump_at_stale_transition(self) -> None:
        # Deliver a GPS fix and establish an anchor.
        self.robot.set_position_fusion_alpha(1.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))
        x_before, y_before, _ = self.robot.get_fused_pose()

        # Expire GPS by pushing last_time into the past.
        self.robot._gps_last_time = 0.0

        # Next kinematics tick — same odometry position, GPS now stale.
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))
        x_after, y_after, _ = self.robot.get_fused_pose()

        # No jump: dead reckoning from anchor with zero odometry delta = same position.
        self.assertAlmostEqual(x_after, x_before, places=3)
        self.assertAlmostEqual(y_after, y_before, places=3)

    def test_dead_reckoning_tracks_odometry_delta_from_anchor(self) -> None:
        # Anchor at (500, 0) mm with odom at (0, 0).
        self.robot.set_position_fusion_alpha(1.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))

        # GPS goes stale; robot moves +200 mm in odometry.
        self.robot._gps_last_time = 0.0
        self.robot._on_kinematics(_make_kin(self.mod, x=200.0, y=0.0))
        x, y, _ = self.robot.get_fused_pose()

        # Fused must be anchor (500) + delta (200) = 700 mm.
        self.assertAlmostEqual(x, 700.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_dead_reckoning_is_better_than_raw_odometry(self) -> None:
        # Establish anchor at arena position (500, 0), odom at (100, 0).
        self.robot.set_position_fusion_alpha(1.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=100.0, y=0.0))

        # GPS stale; robot moves another 150 mm in odometry → odom now at 250.
        self.robot._gps_last_time = 0.0
        self.robot._on_kinematics(_make_kin(self.mod, x=250.0, y=0.0))
        x_fused, _, _ = self.robot.get_fused_pose()

        # Dead reckoning: 500 + (250 - 100) = 650
        # Raw odometry would report 250 — 400 mm off from the true arena position.
        self.assertAlmostEqual(x_fused, 650.0, places=3)
        self.assertNotAlmostEqual(x_fused, 250.0, places=0)

    def test_gps_reacquisition_re_anchors_position(self) -> None:
        # Establish anchor, let GPS go stale, drift in odometry, then re-acquire.
        self.robot.set_position_fusion_alpha(1.0)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.5, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=0.0, y=0.0))

        self.robot._gps_last_time = 0.0
        self.robot._on_kinematics(_make_kin(self.mod, x=200.0, y=0.0))

        # GPS re-acquires with a new absolute fix at (750, 0) mm.
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.75, y_m=0.0))
        self.robot._on_kinematics(_make_kin(self.mod, x=200.0, y=0.0))
        x, _, _ = self.robot.get_fused_pose()

        # Alpha=1 → snaps to new GPS fix.
        self.assertAlmostEqual(x, 750.0, places=3)

    # ------------------------------------------------------------------
    # is_gps_active()
    # ------------------------------------------------------------------

    def test_is_gps_active_false_after_timeout(self) -> None:
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.1, y_m=0.0))
        self.robot._gps_last_time = 0.0   # simulate stale GPS
        self.assertFalse(self.robot.is_gps_active())

    # ------------------------------------------------------------------
    # set_position_fusion_alpha() clamping
    # ------------------------------------------------------------------

    def test_fusion_alpha_above_one_clamped(self) -> None:
        self.robot.set_position_fusion_alpha(5.0)
        self.assertAlmostEqual(self.robot._pos_fusion.alpha, 1.0, places=6)

    def test_fusion_alpha_below_zero_clamped(self) -> None:
        self.robot.set_position_fusion_alpha(-2.0)
        self.assertAlmostEqual(self.robot._pos_fusion.alpha, 0.0, places=6)

    def test_fusion_alpha_boundary_values_accepted(self) -> None:
        self.robot.set_position_fusion_alpha(0.0)
        self.assertAlmostEqual(self.robot._pos_fusion.alpha, 0.0, places=6)
        self.robot.set_position_fusion_alpha(1.0)
        self.assertAlmostEqual(self.robot._pos_fusion.alpha, 1.0, places=6)

    # ------------------------------------------------------------------
    # get_fused_pose() is consistent with get_pose()
    # ------------------------------------------------------------------

    def test_get_fused_pose_matches_get_pose(self) -> None:
        self.robot.set_position_fusion_alpha(0.5)
        self.robot._on_tag_detections(_make_tag(self.mod, x_m=0.4, y_m=0.2))
        self.robot._on_kinematics(_make_kin(self.mod, x=100.0, y=50.0))

        fused = self.robot.get_fused_pose()
        pose  = self.robot.get_pose()
        self.assertAlmostEqual(fused[0], pose[0], places=5)
        self.assertAlmostEqual(fused[1], pose[1], places=5)


if __name__ == "__main__":
    unittest.main()
