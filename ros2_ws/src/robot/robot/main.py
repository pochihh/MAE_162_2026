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


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID = 11 # set aruco tag ID 11 
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
    robot.set_tracked_tag_id(TAG_ID) # set aruco tag ID as the tracked tag for localization


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
                (0.0, 3.6576e3), # 2nd point
                #(500.0, 500.0), # 3rd point
                #(500.0, 0.0), # 4th point
                #(0.0, 0.0), # 5th point
            ]    
            path1 = path_control_points
            #path1 = densify_polyline(path_control_points, spacing=20.0)
            remaining_path = path1.copy() 
            print("Path is ready, Entering IDLE state.")
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.get_button(Button.BTN_1):
                LOOKAHEAD_DIST = 100.0 # Lookahead distance in mm (adjust as needed)
                planner1 = PurePursuitPlanner(
                    lookahead_dist=LOOKAHEAD_DIST, 
                    max_angular=1.5, # Max angular velocity in rad/s (adjust as needed)
                    goal_tolerance=20.0, # Distance in mm to consider the target reached (adjust as needed)
             )
                print("Pure Pursuit Planner is initialized. Start Moving!")
                print("[FSM] MOVING")
                state = "MOVING"
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()

        elif state == "MOVING":
            show_moving_leds(robot)
            # Step 1: Get current pose, including current coordinates and heading angle in degrees
            # using robot.get_pose() function. Store the values in current_x, current_y, and current_theta_deg variables.
            current_x, current_y, current_theta_deg = robot.get_pose()

            # Step 2: Convert current_theta_deg to radians and store it in current_theta_rad variable.
            current_theta_rad = math.radians(current_theta_deg)

            # Step 3: Use the _advance_remaining_path() function to update the remaining_path variable
            # by advancing it based on the current position (current_x, current_y) and an advance radius(20.0) mm.
            # This will take out the waypoints that are already passed (within 20mm of the current position),
            # effectively "advancing" the path as the robot moves.
            remaining_path = robot._advance_remaining_path(remaining_path, current_x, current_y, advance_radius_mm=LOOKAHEAD_DIST)

            # Step 4: Use the _lookahead_point() function to calculate the current pursuit point
            # in your path, defined as (current_pursuit_x, current_pursuit_y)
            current_pursuit_x, current_pursuit_y = planner1._lookahead_point(
                current_x,
                current_y,
                waypoints=remaining_path,
            )

            # Step 5: Use the compute_velocity() function of the PurePursuitPlanner
            # to calculate the linear and angular velocity commands
            linear_velocity_cmd, angular_velocity_cmd_rad_s = planner1.compute_velocity(
                pose=(current_x, current_y, current_theta_rad),
                waypoints=remaining_path,
                max_linear=80.0, # Max linear velocity in mm/s (adjust as needed
            )

            # Step 6: Use the robot.set_velocity() function to send the velocity commands to the robot.
            robot.set_velocity(
                linear_velocity_cmd,
                math.degrees(angular_velocity_cmd_rad_s),
            )

            # Step 7: Check if the current target point is reached using the
            # CurrentTargetReached() function of the PurePursuitPlanner.
            # Just uncomment the following lines to enable the print statements.
            if planner1.CurrentTargetReached(current_pursuit_x, current_pursuit_y, current_x, current_y):
                print("MOVING: Target reached! Stopping.")
                robot.stop()
                print("[FSM] IDLE")
                state = "IDLE"
            
        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()