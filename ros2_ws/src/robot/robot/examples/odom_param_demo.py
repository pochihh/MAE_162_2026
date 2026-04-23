"""
Example: Runtime Odometry Parameter Setup
=========================================
Configures odometry and diff-drive kinematics from Python, then exercises the
high-level motion helpers with button presses.

HOW TO APPLY THIS EXAMPLE
--------------------------
Copy the constants and the run() function into your main.py, replacing the
existing run() function. Then run as normal:

    ros2 run robot robot

WHAT IT DOES
------------
- Applies wheel diameter, wheel base, initial theta, odom wheel mapping, and
  per-side direction inversion through `/sys_odom_param_set`
- Resets odometry so the new initial heading takes effect
- Button 1: drive forward a fixed distance
- Button 2: rotate in place
- Button 3: reset odometry
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, Motor
from robot.robot import FirmwareState, Robot


WHEEL_DIAMETER_MM = 74.0
WHEEL_BASE_MM = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

FORWARD_DISTANCE_MM = 250.0
TURN_ANGLE_DEG = 90.0
DRIVE_SPEED_MM_S = 150.0


def run(robot: Robot) -> None:
    robot.set_odometry_parameters(
        wheel_diameter_mm=WHEEL_DIAMETER_MM,
        wheel_base_mm=WHEEL_BASE_MM,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.reset_odometry()

    if not robot.set_state(FirmwareState.RUNNING):
        print("[OdomDemo] Failed to enter RUNNING state")
        return

    print("[OdomDemo] BTN1=forward, BTN2=turn left 90 deg, BTN3=reset odometry")
    prev_b1 = prev_b2 = prev_b3 = False

    while True:
        b1 = robot.get_button(Button.BTN_1)
        b2 = robot.get_button(Button.BTN_2)
        b3 = robot.get_button(Button.BTN_3)

        if b1 and not prev_b1:
            print("[OdomDemo] move_by forward")
            robot.move_by(FORWARD_DISTANCE_MM, 0.0, DRIVE_SPEED_MM_S, blocking=True)
            print(f"[OdomDemo] pose={robot.get_pose()}")

        if b2 and not prev_b2:
            print("[OdomDemo] turn_by 90 deg")
            robot.turn_by(TURN_ANGLE_DEG, blocking=True)
            print(f"[OdomDemo] pose={robot.get_pose()}")

        if b3 and not prev_b3:
            robot.reset_odometry()
            print("[OdomDemo] odometry reset")

        prev_b1 = b1
        prev_b2 = b2
        prev_b3 = b3
        time.sleep(0.02)
