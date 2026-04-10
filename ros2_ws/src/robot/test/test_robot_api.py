from __future__ import annotations

import importlib
import sys
import threading
import types
import unittest
from unittest import mock
from pathlib import Path


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: list[object] = []

    def publish(self, msg: object) -> None:
        self.published.append(msg)


class FakeLogger:
    def info(self, _msg: str) -> None:
        pass

    def error(self, _msg: str) -> None:
        pass


class FakeFuture:
    def __init__(self) -> None:
        self._done = False
        self._callbacks: list = []

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def done(self) -> bool:
        return self._done

    def result(self):
        return None


class FakeClient:
    def wait_for_service(self, timeout_sec=None) -> bool:
        return True

    def call_async(self, _request) -> FakeFuture:
        return FakeFuture()


class FakeNode:
    def __init__(self) -> None:
        self.publishers: dict[str, FakePublisher] = {}
        self.logger = FakeLogger()

    def create_publisher(self, _msg_type, topic: str, _qos: int) -> FakePublisher:
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_client(self, *_args, **_kwargs) -> FakeClient:
        return FakeClient()

    def get_logger(self) -> FakeLogger:
        return self.logger


def _install_fake_robot_dependencies() -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    bridge_interfaces = types.ModuleType("bridge_interfaces")
    bridge_interfaces_msg = types.ModuleType("bridge_interfaces.msg")
    bridge_interfaces_srv = types.ModuleType("bridge_interfaces.srv")

    def make_message(name: str):
        return type(name, (), {})

    for name in [
        "DCEnable",
        "DCHome",
        "DCPid",
        "DCPidReq",
        "DCPidSet",
        "DCResetPosition",
        "DCSetPosition",
        "DCSetPwm",
        "DCSetVelocity",
        "DCStateAll",
        "IOInputState",
        "IOOutputState",
        "IOSetLed",
        "IOSetNeopixel",
        "SensorImu",
        "SensorKinematics",
        "ServoEnable",
        "ServoSet",
        "ServoStateAll",
        "StepConfig",
        "StepConfigReq",
        "StepConfigSet",
        "StepEnable",
        "StepHome",
        "StepMove",
        "StepStateAll",
        "SysOdomParamReq",
        "SysOdomParamRsp",
        "SysOdomParamSet",
        "SysOdomReset",
        "SystemConfig",
        "SystemDiag",
        "SystemInfo",
        "SystemPower",
        "SystemState",
    ]:
        setattr(bridge_interfaces_msg, name, make_message(name))

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


class RobotApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))
        cls.robot_module = importlib.import_module("robot.robot")
        cls.hardware_map = importlib.import_module("robot.hardware_map")

    def setUp(self) -> None:
        self.node = FakeNode()
        self.robot = self.robot_module.Robot(self.node)

    def test_diff_drive_uses_1_based_motor_numbers(self) -> None:
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_set_velocity"].published
        self.assertEqual(len(published), 2)
        self.assertEqual([msg.motor_number for msg in published], [1, 2])

    def test_body_velocity_auto_enables_drive_motors(self) -> None:
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_enable"].published
        self.assertEqual(len(published), 2)
        self.assertEqual([msg.motor_number for msg in published], [1, 2])
        self.assertTrue(all(msg.mode == int(self.hardware_map.DCMotorMode.VELOCITY) for msg in published))

    def test_body_velocity_skips_reenable_when_drive_motors_are_already_enabled(self) -> None:
        enabled = int(self.hardware_map.DCMotorMode.VELOCITY)
        self.robot._dc_state = types.SimpleNamespace(
            motors=[
                types.SimpleNamespace(mode=enabled),
                types.SimpleNamespace(mode=enabled),
                types.SimpleNamespace(mode=0),
                types.SimpleNamespace(mode=0),
            ]
        )

        self.robot.set_velocity(100.0, 0.0)

        self.assertEqual(self.node.publishers["/dc_enable"].published, [])

    def test_diff_drive_mapping_is_configurable(self) -> None:
        self.robot.set_drive_wheels(3, 4)
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_set_velocity"].published
        self.assertEqual([msg.motor_number for msg in published], [3, 4])

    def test_high_level_velocity_respects_runtime_wheel_direction_flags(self) -> None:
        self.robot.set_odometry_parameters(
            left_motor_dir_inverted=False,
            right_motor_dir_inverted=False,
        )
        self.node.publishers["/dc_set_velocity"].published.clear()
        self.robot.set_velocity(100.0, 0.0)
        both_forward = self.node.publishers["/dc_set_velocity"].published
        self.assertGreater(both_forward[0].target_ticks, 0)
        self.assertGreater(both_forward[1].target_ticks, 0)

        self.robot.set_odometry_parameters(
            left_motor_dir_inverted=True,
            right_motor_dir_inverted=False,
        )
        self.node.publishers["/dc_set_velocity"].published.clear()
        self.robot.set_velocity(100.0, 0.0)
        mixed = self.node.publishers["/dc_set_velocity"].published
        self.assertLess(mixed[0].target_ticks, 0)
        self.assertGreater(mixed[1].target_ticks, 0)

    def test_set_wheel_diameter_publishes_full_odom_param_snapshot(self) -> None:
        self.robot.set_wheel_diameter_mm(80.0)

        msg = self.node.publishers["/sys_odom_param_set"].published[-1]
        self.assertEqual(msg.wheel_diameter_mm, 80.0)
        self.assertEqual(msg.wheel_base_mm, 333.0)
        self.assertEqual(msg.initial_theta_deg, 90.0)
        self.assertEqual(msg.left_motor_number, 1)
        self.assertFalse(msg.left_motor_dir_inverted)
        self.assertEqual(msg.right_motor_number, 2)
        self.assertTrue(msg.right_motor_dir_inverted)

    def test_set_odometry_parameters_uses_current_unit_for_geometry(self) -> None:
        self.robot.set_unit(self.robot_module.Unit.INCH)

        self.robot.set_odometry_parameters(
            wheel_diameter=3.0,
            wheel_base=12.0,
        )

        msg = self.node.publishers["/sys_odom_param_set"].published[-1]
        self.assertAlmostEqual(msg.wheel_diameter_mm, 76.2, places=6)
        self.assertAlmostEqual(msg.wheel_base_mm, 304.8, places=6)
        self.assertAlmostEqual(self.robot.get_odometry_parameters()["wheel_diameter_mm"], 76.2, places=6)
        self.assertAlmostEqual(self.robot.get_odometry_parameters()["wheel_base_mm"], 304.8, places=6)

    def test_request_odometry_parameters_publishes_query(self) -> None:
        self.robot.request_odometry_parameters()

        msg = self.node.publishers["/sys_odom_param_req"].published[-1]
        self.assertEqual(msg.target, 0xFF)

    def test_system_info_diag_and_config_are_cached(self) -> None:
        info = types.SimpleNamespace(firmware_major=1, dc_motor_count=4)
        config = types.SimpleNamespace(motor_dir_mask=3, neopixel_count=4)
        diag = types.SimpleNamespace(free_sram=1234, tx_dropped_frames=5)

        self.robot._on_sys_info(info)
        self.robot._on_sys_config(config)
        self.robot._on_sys_diag(diag)

        self.assertIs(self.robot.get_system_info(), info)
        self.assertIs(self.robot.get_system_config(), config)
        self.assertIs(self.robot.get_system_diag(), diag)

    def test_odom_param_response_updates_local_snapshot(self) -> None:
        msg = types.SimpleNamespace(
            wheel_diameter_mm=82.5,
            wheel_base_mm=345.0,
            initial_theta_deg=45.0,
            left_motor_number=3,
            left_motor_dir_inverted=True,
            right_motor_number=4,
            right_motor_dir_inverted=False,
        )

        self.robot._on_odom_param_rsp(msg)

        self.assertEqual(
            self.robot.get_odometry_parameters(),
            {
                "wheel_diameter_mm": 82.5,
                "wheel_base_mm": 345.0,
                "initial_theta_deg": 45.0,
                "left_motor_number": 3,
                "left_motor_dir_inverted": True,
                "right_motor_number": 4,
                "right_motor_dir_inverted": False,
            },
        )

    def test_odom_param_stale_firmware_echo_reasserts_user_config(self) -> None:
        self.robot.set_odometry_parameters(left_motor_id=3, right_motor_id=4)
        self.node.publishers["/sys_odom_param_set"].published.clear()

        # Firmware echoes its stale defaults (M1/M2) — Python state must be
        # preserved AND user params must be re-published to resync firmware.
        firmware_echo = types.SimpleNamespace(
            wheel_diameter_mm=70.0,
            wheel_base_mm=300.0,
            initial_theta_deg=0.0,
            left_motor_number=1,
            left_motor_dir_inverted=False,
            right_motor_number=2,
            right_motor_dir_inverted=True,
        )
        self.robot._on_odom_param_rsp(firmware_echo)

        # Python state unchanged
        params = self.robot.get_odometry_parameters()
        self.assertEqual(params["left_motor_number"], 3)
        self.assertEqual(params["right_motor_number"], 4)

        # Firmware was re-sent our config
        reassert = self.node.publishers["/sys_odom_param_set"].published
        self.assertEqual(len(reassert), 1)
        self.assertEqual(reassert[0].left_motor_number, 3)
        self.assertEqual(reassert[0].right_motor_number, 4)

    def test_odom_param_matching_firmware_echo_is_silent(self) -> None:
        self.robot.set_odometry_parameters(left_motor_id=3, right_motor_id=4)
        self.node.publishers["/sys_odom_param_set"].published.clear()

        params = self.robot.get_odometry_parameters()
        matching_echo = types.SimpleNamespace(
            wheel_diameter_mm=params["wheel_diameter_mm"],
            wheel_base_mm=params["wheel_base_mm"],
            initial_theta_deg=params["initial_theta_deg"],
            left_motor_number=3,
            left_motor_dir_inverted=params["left_motor_dir_inverted"],
            right_motor_number=4,
            right_motor_dir_inverted=params["right_motor_dir_inverted"],
        )
        self.robot._on_odom_param_rsp(matching_echo)

        # No re-publish needed when echo already matches
        self.assertEqual(self.node.publishers["/sys_odom_param_set"].published, [])

    def test_duplicate_odom_motor_pair_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be different"):
            self.robot.set_odometry_parameters(left_motor_id=2, right_motor_id=2)

    def test_request_pid_and_get_pid_cache(self) -> None:
        self.robot.request_pid(2, self.hardware_map.DCPidLoop.VELOCITY)
        req = self.node.publishers["/dc_pid_req"].published[-1]
        self.assertEqual(req.motor_number, 2)
        self.assertEqual(req.loop_type, int(self.hardware_map.DCPidLoop.VELOCITY))

        rsp = types.SimpleNamespace(
            motor_number=2,
            loop_type=int(self.hardware_map.DCPidLoop.VELOCITY),
            kp=1.0,
            ki=2.0,
            kd=3.0,
            max_output=255.0,
            max_integral=1000.0,
        )
        self.robot._on_dc_pid(rsp)
        self.assertIs(self.robot.get_pid(2, self.hardware_map.DCPidLoop.VELOCITY), rsp)

    def test_request_step_config_and_get_step_config_cache(self) -> None:
        self.robot.request_step_config(3)
        req = self.node.publishers["/step_config_req"].published[-1]
        self.assertEqual(req.stepper_number, 3)

        rsp = types.SimpleNamespace(stepper_number=3, max_velocity=800, acceleration=1200)
        self.robot._on_step_config(rsp)
        self.assertIs(self.robot.get_step_config(3), rsp)

    def test_get_io_output_state_returns_cached_message(self) -> None:
        msg = types.SimpleNamespace(led_brightness=[0, 1, 2, 3, 4], neo_pixel_count=2)
        self.robot._on_io_output(msg)
        self.assertIs(self.robot.get_io_output_state(), msg)

    def test_zero_brightness_defaults_to_led_off_mode(self) -> None:
        self.robot.set_led(1, 0)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.led_id, 1)
        self.assertEqual(msg.brightness, 0)
        self.assertEqual(msg.mode, 0)

    def test_nonzero_brightness_defaults_to_steady_mode(self) -> None:
        self.robot.set_led(1, 255)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 1)
        self.assertEqual(msg.brightness, 255)

    def test_motion_handle_reports_finished(self) -> None:
        finished = threading.Event()
        cancel = threading.Event()
        handle = self.robot_module.MotionHandle(finished, cancel)

        self.assertFalse(handle.is_finished())
        self.assertFalse(handle.is_done())

        finished.set()

        self.assertTrue(handle.is_finished())
        self.assertTrue(handle.is_done())

    def test_blocking_navigation_returns_finished_handle(self) -> None:
        handle = self.robot._start_nav(lambda: None, blocking=True, timeout=0.1)

        self.assertIsInstance(handle, self.robot_module.MotionHandle)
        self.assertTrue(handle.is_finished())

    def test_purepursuit_follow_path_requires_waypoints(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            self.robot.purepursuit_follow_path(
                [],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                blocking=False,
            )

    def test_purepursuit_follow_path_defaults_advance_radius_to_tolerance(self) -> None:
        with mock.patch.object(self.robot, "_nav_follow_purepursuit_path") as nav_follow, \
             mock.patch.object(
                 self.robot,
                 "_start_nav",
                 side_effect=lambda target, blocking, timeout: (target(), "handle")[1],
             ):
            result = self.robot.purepursuit_follow_path(
                [(0.0, 0.0), (100.0, 0.0)],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                blocking=False,
            )

        self.assertEqual(result, "handle")
        nav_follow.assert_called_once_with(
            [(0.0, 0.0), (100.0, 0.0)],
            100.0,
            100.0,
            20.0,
            20.0,
            1.0,
        )

    def test_purepursuit_follow_path_accepts_explicit_advance_radius(self) -> None:
        with mock.patch.object(self.robot, "_nav_follow_purepursuit_path") as nav_follow, \
             mock.patch.object(
                 self.robot,
                 "_start_nav",
                 side_effect=lambda target, blocking, timeout: (target(), "handle")[1],
             ):
            self.robot.purepursuit_follow_path(
                [(0.0, 0.0), (100.0, 0.0)],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                advance_radius=35.0,
                blocking=False,
            )

        nav_follow.assert_called_once_with(
            [(0.0, 0.0), (100.0, 0.0)],
            100.0,
            100.0,
            35.0,
            20.0,
            1.0,
        )

    def test_apf_follow_path_requires_waypoints(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            self.robot.apf_follow_path(
                [],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                repulsion_range=150.0,
                blocking=False,
            )

    def test_apf_follow_path_defaults_advance_radius_to_tolerance(self) -> None:
        with mock.patch.object(self.robot, "_nav_follow_apf_path") as nav_follow, \
             mock.patch.object(
                 self.robot,
                 "_start_nav",
                 side_effect=lambda target, blocking, timeout: (target(), "handle")[1],
             ):
            result = self.robot.apf_follow_path(
                [(0.0, 0.0), (100.0, 0.0)],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                repulsion_range=150.0,
                blocking=False,
            )

        self.assertEqual(result, "handle")
        nav_follow.assert_called_once_with(
            [(0.0, 0.0), (100.0, 0.0)],
            100.0,
            100.0,
            20.0,
            20.0,
            150.0,
            1.0,
            500.0,
        )

    def test_unit_dependent_navigation_parameters_must_be_explicit(self) -> None:
        with self.assertRaises(TypeError):
            self.robot.move_to(100.0, 0.0, 100.0)

        with self.assertRaises(TypeError):
            self.robot.move_by(100.0, 0.0, 100.0)

        with self.assertRaises(TypeError):
            self.robot.purepursuit_follow_path([(0.0, 0.0), (100.0, 0.0)], velocity=100.0)

    def test_purepursuit_follow_path_rejects_overlapping_motion(self) -> None:
        self.robot._nav_thread = types.SimpleNamespace(is_alive=lambda: True)

        with self.assertRaisesRegex(RuntimeError, "Another motion is still running"):
            self.robot.purepursuit_follow_path(
                [(0.0, 0.0), (100.0, 0.0)],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                blocking=False,
            )

    def test_apf_follow_path_rejects_overlapping_motion(self) -> None:
        self.robot._nav_thread = types.SimpleNamespace(is_alive=lambda: True)

        with self.assertRaisesRegex(RuntimeError, "Another motion is still running"):
            self.robot.apf_follow_path(
                [(0.0, 0.0), (100.0, 0.0)],
                velocity=100.0,
                lookahead=100.0,
                tolerance=20.0,
                repulsion_range=150.0,
                blocking=False,
            )

    def test_move_forward_uses_current_heading(self) -> None:
        self.robot._pose = (10.0, 20.0, 0.0)

        with mock.patch.object(self.robot, "move_to", return_value="ok") as move_to:
            result = self.robot.move_forward(100.0, 50.0, tolerance=10.0, blocking=False)

        self.assertEqual(result, "ok")
        move_to.assert_called_once_with(
            110.0,
            20.0,
            50.0,
            tolerance=10.0,
            blocking=False,
            timeout=None,
        )

    def test_move_backward_uses_current_heading(self) -> None:
        self.robot._pose = (10.0, 20.0, self.robot_module.math.pi / 2.0)

        with mock.patch.object(self.robot, "move_to", return_value="ok") as move_to:
            result = self.robot.move_backward(100.0, 50.0, tolerance=10.0, blocking=True)

        self.assertEqual(result, "ok")
        move_to.assert_called_once()
        x_arg, y_arg, velocity_arg = move_to.call_args.args[:3]
        self.assertAlmostEqual(x_arg, 10.0, places=6)
        self.assertAlmostEqual(y_arg, -80.0, places=6)
        self.assertEqual(velocity_arg, 50.0)
        self.assertEqual(move_to.call_args.kwargs["tolerance"], 10.0)
        self.assertTrue(move_to.call_args.kwargs["blocking"])

    def test_set_obstacles_converts_current_unit_to_mm(self) -> None:
        self.robot.set_unit(self.robot_module.Unit.INCH)

        self.robot.set_obstacles([(1.0, 2.0)])

        self.assertEqual(self.robot._get_obstacles_mm(), [(25.4, 50.8)])
        self.assertEqual(self.robot.get_obstacles(), [(1.0, 2.0)])

    def test_obstacle_provider_is_combined_with_cached_obstacles(self) -> None:
        self.robot.set_obstacles([(10.0, 20.0)])
        self.robot.set_obstacle_provider(lambda: [(30.0, -40.0)])

        self.assertEqual(self.robot._get_obstacles_mm(), [(10.0, 20.0), (30.0, -40.0)])
        self.assertEqual(self.robot.get_obstacles(), [(10.0, 20.0), (30.0, -40.0)])

    def test_path_progress_does_not_finish_early_on_closed_loop(self) -> None:
        remaining = [
            (0.0, 0.0),
            (0.0, 100.0),
            (100.0, 100.0),
            (100.0, 0.0),
            (0.0, 0.0),
        ]

        advanced = self.robot._advance_remaining_path(
            remaining,
            x_mm=0.0,
            y_mm=0.0,
            advance_radius_mm=20.0,
        )

        self.assertEqual(
            advanced,
            [
                (0.0, 100.0),
                (100.0, 100.0),
                (100.0, 0.0),
                (0.0, 0.0),
            ],
        )

    def test_led_mode_enum_matches_firmware_contract(self) -> None:
        self.robot.set_led(1, 128, mode=self.hardware_map.LEDMode.BREATHE)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 3)
        self.assertEqual(msg.period_ms, 1000)
        self.assertEqual(msg.duty_cycle, 500)

    def test_blink_defaults_to_symmetric_duty_cycle(self) -> None:
        self.robot.set_led(1, 255, mode=self.hardware_map.LEDMode.BLINK)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 2)
        self.assertEqual(msg.period_ms, 1000)
        self.assertEqual(msg.duty_cycle, 500)

    def test_invalid_motor_number_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "motor_id must be between 1 and 4"):
            self.robot.set_motor_velocity(0, 100.0)

    def test_invalid_pid_loop_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "loop_type must be one of"):
            self.robot.request_pid(1, 2)

    def test_shutdown_stops_and_disables_drive_motors(self) -> None:
        self.robot.shutdown()

        velocity_msgs = self.node.publishers["/dc_set_velocity"].published
        disable_msgs = self.node.publishers["/dc_enable"].published
        self.assertEqual([msg.motor_number for msg in velocity_msgs], [1, 2])
        self.assertTrue(all(msg.target_ticks == 0 for msg in velocity_msgs))
        self.assertEqual([msg.motor_number for msg in disable_msgs], [1, 2])
        self.assertTrue(all(msg.mode == int(self.hardware_map.DCMotorMode.DISABLED) for msg in disable_msgs))

    def test_shutdown_requests_idle_when_firmware_was_running(self) -> None:
        self.robot._sys_state = int(self.robot_module.FirmwareState.RUNNING)

        with mock.patch.object(self.robot, "set_state", return_value=True) as set_state:
            self.robot.shutdown()

        set_state.assert_called_once_with(self.robot_module.FirmwareState.IDLE, timeout=1.0)

    def test_button_edges_are_latched_in_subscription_callback(self) -> None:
        first = self.robot_module.IOInputState()
        first.button_mask = 0
        first.limit_mask = 0
        self.robot._on_io_input(first)

        pressed = self.robot_module.IOInputState()
        pressed.button_mask = 0b1
        pressed.limit_mask = 0
        self.robot._on_io_input(pressed)

        self.assertTrue(self.robot.get_button(1))
        self.assertTrue(self.robot.was_button_pressed(1))
        self.assertFalse(self.robot.was_button_pressed(1))

    def test_wait_helpers_translate_public_ids_to_state_array_indices(self) -> None:
        dc_state = types.SimpleNamespace(
            motors=[
                types.SimpleNamespace(position=0, mode=0),
                types.SimpleNamespace(position=0, mode=0),
                types.SimpleNamespace(position=0, mode=0),
                types.SimpleNamespace(position=123, mode=0),
            ]
        )
        step_state = types.SimpleNamespace(
            steppers=[
                types.SimpleNamespace(motion_state=1),
                types.SimpleNamespace(motion_state=1),
                types.SimpleNamespace(motion_state=1),
                types.SimpleNamespace(motion_state=0),
            ]
        )
        self.robot._dc_state = dc_state
        self.robot._step_state = step_state

        self.assertTrue(self.robot._wait_dc_position(4, 123, 0, 0.01))
        self.assertTrue(self.robot._wait_dc_not_homing(4, 0.01))
        with mock.patch.object(self.robot_module.time, "sleep", lambda *_args, **_kwargs: None):
            self.assertTrue(self.robot._wait_stepper_idle(4, 0.01))


if __name__ == "__main__":
    unittest.main()
