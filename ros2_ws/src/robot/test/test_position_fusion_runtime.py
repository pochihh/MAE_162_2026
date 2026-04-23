from __future__ import annotations

import importlib
import sys
import threading
import types
import unittest
from pathlib import Path


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))


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


class _FakeStateClient:
    def wait_for_service(self, timeout_sec: float | None = None) -> bool:
        return True


class _FakeRobot:
    def __init__(self, timeline: list[dict[str, float | bool]]) -> None:
        self._lock = threading.Lock()
        self._set_state_client = _FakeStateClient()
        self._timeline = timeline
        self._timeline_index = 0
        self._state = 1  # IDLE
        self._pose = (0.0, 0.0, 0.0)
        self._gps_x_mm = 0.0
        self._gps_y_mm = 0.0
        self._fused = (0.0, 0.0, 0.0)
        self.velocity_cmds: list[tuple[float, float]] = []
        self.offset_calls: list[tuple[float, float]] = []
        self.reset_count = 0
        self.stop_count = 0

    def get_state(self) -> int:
        return self._state

    def set_state(self, state) -> bool:
        self._state = int(state)
        return True

    def reset_odometry(self) -> None:
        self.reset_count += 1
        with self._lock:
            self._pose = (0.0, 0.0, 0.0)
            self._fused = (0.0, 0.0, 0.0)

    def wait_for_pose_update(self, timeout: float | None = None) -> bool:
        if self._timeline_index >= len(self._timeline):
            return True
        sample = self._timeline[self._timeline_index]
        self._timeline_index += 1
        with self._lock:
            self._pose = (
                float(sample["odom_x"]),
                float(sample["odom_y"]),
                0.0,
            )
            self._gps_x_mm = float(sample["gps_x"])
            self._gps_y_mm = float(sample["gps_y"])
            self._fused = (
                float(sample["fused_x"]),
                float(sample["fused_y"]),
                0.0,
            )
        self._gps_active = bool(sample["gps_active"])
        return True

    def is_gps_active(self) -> bool:
        return getattr(self, "_gps_active", False)

    def set_gps_offset(self, x_mm: float, y_mm: float) -> None:
        self.offset_calls.append((float(x_mm), float(y_mm)))

    def _get_pose_mm(self) -> tuple[float, float, float]:
        with self._lock:
            return self._fused

    def set_velocity(self, linear_mm_s: float, angular_deg_s: float) -> None:
        self.velocity_cmds.append((float(linear_mm_s), float(angular_deg_s)))

    def stop(self) -> None:
        self.stop_count += 1

    def set_position_fusion_alpha(self, alpha: float) -> None:
        self.alpha = float(alpha)


class PositionFusionRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        cls.mod = importlib.import_module("robot.test_position_fusion")

    def test_drive_straight_learns_gps_offset_on_first_acquisition(self) -> None:
        # Two initial reset-propagation waits, then:
        # 1. no GPS
        # 2. first GPS fix -> learn offset and skip recording
        # 3. next GPS-backed sample -> record and finish
        timeline = [
            {"odom_x": 0.0, "odom_y": 0.0, "gps_x": 0.0,  "gps_y": 0.0,  "fused_x": 0.0, "fused_y": 0.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 0.0, "gps_x": 0.0,  "gps_y": 0.0,  "fused_x": 0.0, "fused_y": 0.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 100.0, "gps_x": 10.0, "gps_y": 20.0, "fused_x": 0.0, "fused_y": 100.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 200.0, "gps_x": 25.0, "gps_y": 75.0, "fused_x": 0.0, "fused_y": 200.0, "gps_active": True},
            {"odom_x": 0.0, "odom_y": 1000.0, "gps_x": 25.0, "gps_y": 75.0, "fused_x": 0.0, "fused_y": 980.0, "gps_active": True},
        ]
        robot = _FakeRobot(timeline)
        rec = self.mod._Record()

        self.mod._drive_straight(robot, rec)

        self.assertEqual(robot.reset_count, 1)
        self.assertEqual(robot.offset_calls, [(-25.0, 125.0)])
        self.assertEqual(robot.stop_count, 1)
        self.assertEqual(len(rec.t), 2)
        self.assertEqual(list(rec.gps_active), [False, True])
        self.assertAlmostEqual(rec.odom_y[-1], 1000.0, places=6)
        self.assertAlmostEqual(rec.fused_y[-1], 980.0, places=6)

    def test_drive_straight_commands_forward_only_motion(self) -> None:
        timeline = [
            {"odom_x": 0.0, "odom_y": 0.0, "gps_x": 0.0, "gps_y": 0.0, "fused_x": 0.0, "fused_y": 0.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 0.0, "gps_x": 0.0, "gps_y": 0.0, "fused_x": 0.0, "fused_y": 0.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 100.0, "gps_x": 0.0, "gps_y": 0.0, "fused_x": 0.0, "fused_y": 100.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 250.0, "gps_x": 0.0, "gps_y": 0.0, "fused_x": 0.0, "fused_y": 250.0, "gps_active": False},
            {"odom_x": 0.0, "odom_y": 1000.0, "gps_x": 0.0, "gps_y": 0.0, "fused_x": 0.0, "fused_y": 1000.0, "gps_active": False},
        ]
        robot = _FakeRobot(timeline)
        rec = self.mod._Record()

        self.mod._drive_straight(robot, rec)

        self.assertGreaterEqual(len(robot.velocity_cmds), 1)
        self.assertTrue(
            all(linear == self.mod.DRIVE_SPEED_MM_S and angular == 0.0
                for linear, angular in robot.velocity_cmds)
        )

    def test_run_starts_from_zero_gps_offset_and_sets_alpha(self) -> None:
        robot = _FakeRobot([])
        calls: list[str] = []

        def fake_drive_straight(inner_robot, rec) -> None:
            self.assertIs(inner_robot, robot)
            calls.append("drive")

        def fake_plot_results(rec) -> None:
            calls.append("plot")

        original_drive = self.mod._drive_straight
        original_plot = self.mod._plot_results
        try:
            self.mod._drive_straight = fake_drive_straight
            self.mod._plot_results = fake_plot_results
            self.mod.run(robot)
        finally:
            self.mod._drive_straight = original_drive
            self.mod._plot_results = original_plot

        self.assertEqual(robot.offset_calls, [(0.0, 0.0)])
        self.assertAlmostEqual(robot.alpha, self.mod.POS_FUSION_ALPHA, places=6)
        self.assertEqual(calls, ["drive", "plot"])


if __name__ == "__main__":
    unittest.main()
