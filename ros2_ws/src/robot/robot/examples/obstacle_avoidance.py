"""
main.py — student entry point
==============================
This is the only file students are expected to edit.

The structure is intentionally simple:
- keep one plain `state` variable
- write helper functions for robot actions
- use `if state == "..."` inside the main loop

To run:
    ros2 run robot robot
"""

from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


def configure_robot(robot: Robot) -> None:
    """Apply the user unit plus robot-specific wheel mapping and odometry settings."""
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    """Start the firmware and reset odometry before the main mission begins."""
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)
    

    state = "INIT"
    drive_handle = None
    # FSM refresh rate control
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            path_control_points = [ #Define your path control points here (x, y) in mm
                (0.0, 0.0), # 1st point
                (0.0, 500.0), # 2nd point
                (500.0, 500.0), # 3rd point
                (500.0, 0.0), # 4th point
                (0.0, 0.0), # 5th point
            ]    
            path = np.float64(densify_polyline(path_control_points, spacing=20.0))
            # initialize the DWA path follower with parameters
            robot._nav_follow_dwa_path(max_vel_mm = 200.0,
            max_acc_mm = 300.0,
            max_angular_radv = 1.0,
            max_angular_acc_rad = 2.0,
            lookahead_mm = 200.0,
            advance_radius_mm = 150.0,
            tolerance_mm = 100.0,
            gains_of_costs = [2.0, 0.02, 0.2, 0.8, 0.1],
            period = period,
            predict_time = 2.0,
            predict_velocity_samples_resolution = [20.0, 0.1],
            obstacles_range_mm = 1000.0,
            ttc_weight = 0.1,
            )
            print("Path is ready, Entering IDLE state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            if robot.get_button(Button.BTN_1):
                LOOKAHEAD_DIST = 100.0 # Lookahead distance in mm (adjust as needed)
                robot._nav_follow_path_loop(path, period)
                print("Start Moving!")
                print("[FSM] MOVING")
                state = "MOVING"

        elif state == "MOVING":
            show_moving_leds(robot)
            
            
        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
