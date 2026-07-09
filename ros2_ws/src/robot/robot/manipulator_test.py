"""
manipulator_test.py — standalone arm test entry point.

Boots the robot, waits for BTN_1, counts down 20 seconds (time to fit the
casing and realign the arm), then runs grab_cylinder() once.
Press BTN_1 again at the IDLE prompt to run another grab.
Press BTN_2 at any time to cancel and return to IDLE.
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED
from robot.robot import FirmwareState, Robot
from robot.manipulator import grab_cylinder

COUNTDOWN_S = 20


def _start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def _enable_lidar(robot: Robot) -> None:
    robot.enable_lidar()
    robot.start_lidar_world_publisher()
    print("[arm-test] LiDAR enabled")


def run(robot: Robot) -> None:
    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    state     = "INIT"

    # Tracks when the countdown started so we can print elapsed seconds.
    countdown_start: float = 0.0

    while True:
        now = time.monotonic()

        # ----------------------------------------------------------------
        if state == "INIT":
            _start_robot(robot)
            _enable_lidar(robot)

            robot.enable_servo(1)
            robot.enable_servo(2)
            robot.enable_servo(3)

            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            print("[arm-test] IDLE — press BTN_1 to start grab sequence")
            state = "IDLE"

        # ----------------------------------------------------------------
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                countdown_start = now
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                print(f"[arm-test] COUNTDOWN — {COUNTDOWN_S}s to fit casing and realign arm...")
                state = "COUNTDOWN"

        # ----------------------------------------------------------------
        elif state == "COUNTDOWN":
            if robot.was_button_pressed(Button.BTN_2):
                print("[arm-test] Countdown cancelled — returning to IDLE")
                robot.set_led(LED.ORANGE, 200)
                robot.set_led(LED.GREEN, 0)
                print("[arm-test] IDLE — press BTN_1 to start grab sequence")
                state = "IDLE"
            else:
                elapsed  = now - countdown_start
                remaining = COUNTDOWN_S - elapsed

                if remaining > 0:
                    # Print a tick every whole second.
                    prev_remaining = COUNTDOWN_S - (now - period - countdown_start)
                    if int(prev_remaining) != int(remaining):
                        print(f"[arm-test] {int(remaining) + 1}...")
                else:
                    print("[arm-test] GO — running grab_cylinder()")
                    robot.set_led(LED.ORANGE, 0)
                    robot.set_led(LED.GREEN, 200)
                    state = "GRABBING"

        # ----------------------------------------------------------------
        elif state == "GRABBING":
            success = grab_cylinder(robot)

            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)

            if success:
                print("[arm-test] Grab succeeded.")
            else:
                print("[arm-test] Grab failed — no cylinder detected.")

            print("[arm-test] IDLE — press BTN_1 to run again")
            state = "IDLE"

        # ----------------------------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
