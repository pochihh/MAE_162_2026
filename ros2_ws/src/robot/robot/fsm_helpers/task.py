from __future__ import annotations

import math
import time
import os
import numpy as np

from robot.fsm_helpers.path_helpers import generate_open_rounded_path
from robot.util import densify_polyline

from robot.robot import Robot
import robot.hardware_map as hm
import robot.fsm_helpers.burgerbot_parameters as bp
import robot.fsm_helpers.course_parameters as cp
import robot.fsm_helpers.vision_helpers as vh
import robot.fsm_helpers.firmware_helpers as fh
import robot.fsm_helpers.ingredient_helpers as ih

ENABLE_CAM = True

class Task:
    def __init__(self, robot: Robot, mission_data: dict):
        self._robot = robot
        self._mission_data = mission_data

    def update(self) -> None:
        pass

    def is_done(self) -> bool:
        return True
    
    def on_enter(self) -> None:
        pass

    def on_exit(self) -> None:
        pass

    def cancel(self) -> None:
        """Abort the task and stop relevant hardware."""
        self._robot.stop()


class HomeTask(Task):
    def __init__(self, robot: Robot, mission_data: dict):
        super().__init__(robot, mission_data)
        self._is_done = False
        self._stepper_handle = None
        self._gripper_handle = None
        self._stepper_done = False
        self._gripper_done = False

    def on_enter(self) -> None:
        print("[HomeTask] ENTER: Homing lift and gripper.")
        
        # 1. Start lift homing only if not already there
        if self._robot.get_limit(hm.Limit.LIM_1):
            print("[HomeTask] Lift already at home limit.")
            self._stepper_done = True
        else:
            self._stepper_handle = fh.home_lift(self._robot)
        
        # 2. Start gripper homing only if not already there
        if self._robot.get_limit(hm.GRIPPER_LIMIT):
            print("[HomeTask] Gripper already at home limit.")
            self._gripper_done = True
        else:
            self._gripper_handle = fh.home_gripper(self._robot)

    def update(self) -> None:
        if self._is_done:
            return

        # Check stepper progress
        if not self._stepper_done:
            # Fallback: check limit switch directly in case handle hangs
            if self._robot.get_limit(hm.Limit.LIM_1) or (self._stepper_handle and self._stepper_handle.is_done()):
                print(f"[HomeTask] Lift homing complete.")
                if self._stepper_handle:
                    self._stepper_handle.cancel()
                    self._stepper_handle = None
                self._stepper_done = True
        
        # Check gripper homing progress
        if not self._gripper_done:
            if self._robot.get_limit(hm.GRIPPER_LIMIT) or (self._gripper_handle and self._gripper_handle.is_done()):
                print(f"[HomeTask] Gripper homing complete.")
                if self._gripper_handle:
                    self._gripper_handle.cancel()
                    self._gripper_handle = None
                self._gripper_done = True

        # Task is done when both subsystems have reached home
        if self._stepper_done and self._gripper_done:
            self._is_done = True

    def cancel(self) -> None:
        if self._stepper_handle:
            self._stepper_handle.cancel()
            self._stepper_handle = None
        if self._gripper_handle:
            self._gripper_handle.cancel()
            self._gripper_handle = None
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done

    def on_exit(self) -> None:
        print("[HomeTask] EXIT: Homing complete.")


class NavTask(Task):
    def __init__(self, robot, mission_data, waypoints, goal_heading=None, path_planner="pp", delivery_segment=False):
        super().__init__(robot, mission_data)
        # Ensure waypoints is a list of tuples
        if waypoints is not None:
            self._vertices = [tuple(wp) for wp in np.atleast_2d(waypoints)]
        else:
            self._vertices = []
        
        self._goal_heading = goal_heading
        self._path_planner = path_planner
        self._delivery_segment = delivery_segment

        self._is_done = False
        self._wp_lapf_idx = 0
        self._handle = None
        self._start_heading = None
        self._start_traj_idx = len(self._robot._fused_traj)

        self._stage_idx = 0
        self._stage_sequence = []
        self._waypoints = list(self._vertices) # Fallback to original vertices

        if self._vertices:
            self._stage_sequence.append(bp.NavStage.POSITION)
            # Remove vertices if within TOLERANCE_MM of initial pose
            x, y, _ = self._robot.get_pose()
            while self._vertices:
                v = self._vertices[0]
                dx = v[0] - x
                dy = v[1] - y
                if math.hypot(dx, dy) < bp.TOLERANCE_MM:
                    self._vertices.pop(0)
                else:
                    break
            path_vertices = [(x, y)] + self._vertices

            if self._path_planner == "pp":
                # Make a dense rounded path if in pure pursuit
                self._waypoints = generate_open_rounded_path(
                    vertices=path_vertices,
                    R=bp.TURN_RADIUS,
                    ds=bp.WP_SPACING
                )

            elif self._path_planner == "lapf":
                # Make denser polyline path if in lapf
                self._waypoints = densify_polyline(
                    control_points=path_vertices,
                    spacing=bp.WP_SPACING_LAPF
                )
                
            # Add an initial heading pivot
            for wp in self._waypoints:
                dx = wp[0] - x
                dy = wp[1] - y
                if math.hypot(dx, dy) > bp.TOLERANCE_MM:
                    self._start_heading = math.degrees(math.atan2(dy, dx))
                    self._stage_sequence.insert(0, bp.NavStage.START_HEADING)
                    break
        
        # Add goal heading if provided
        if self._goal_heading is not None:
            self._stage_sequence.append(bp.NavStage.HEADING)

        if self._stage_sequence:
            # Set stage sequence
            self._stage = self._stage_sequence[self._stage_idx]
        else:
            self._stage = None
            self._is_done = True

    def update(self):

        if self._handle and self._handle.is_done():
            self._handle = None
            self._stage_idx += 1
            if self._stage_idx < len(self._stage_sequence):
                self._stage = self._stage_sequence[self._stage_idx]
            else:
                if self._delivery_segment:
                    self._mission_data["delivery_status"] = True
                self._is_done = True
                return

        if self._stage == bp.NavStage.START_HEADING:
            if self._handle is None:
                print(f"Turning to face first waypoint: {self._start_heading:.2f} deg")
                self._handle = self._robot.turn_to(
                    angle_deg=self._start_heading,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None
                )

        elif self._stage == bp.NavStage.POSITION:
            if self._handle is None:
                if self._path_planner == "pp":
                    print(f"Driving seg with {len(self._waypoints)} waypoints")
                    self._handle = self._robot.purepursuit_follow_path(
                        waypoints=self._waypoints,
                        velocity=bp.VELOCITY_MM_S,
                        lookahead=bp.LOOKAHEAD_MM,
                        tolerance=bp.TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=1.0,
                        advance_radius=bp.ADVANCE_RADIUS_MM
                    )
                elif self._path_planner == "lapf":
                    print(f"Driving (LAPF) seg with {len(self._waypoints)} waypoints")
                    self._handle = self._robot.lapf_follow_path(
                        waypoints=self._waypoints,
                        velocity=bp.VELOCITY_LAPF,
                        max_angular_rad_s=bp.ANGULAR_RAD_S_LAPF,
                        tolerance=bp.TOLERANCE_LAPF,
                        repulsion_range=bp.REPULSION_RANGE_LAPF,
                        repulsion_gain=bp.REPULSION_GAIN_LAPF,
                        attraction_gain=bp.ATTRACTION_GAIN_LAPF,
                        force_ema_alpha=bp.FORCE_EMA_ALPHA_LAPF,
                        inflation_margin_mm=bp.INFLATION_MARGIN_LAPF,
                        leash_length_mm=bp.LEASH_LENGTH_LAPF,
                        leash_half_angle_deg=bp.LEASH_HALF_ANGLE_DEG_LAPF,

                        lookahead=bp.LOOKAHEAD_LAPF,
                        advance_radius=bp.ADVANCE_RADIUS_LAPF,
                        blocking=False,
                    )

        elif self._stage == bp.NavStage.HEADING:
            if self._handle is None:
                print(f"Turning to: {self._goal_heading:.2f} deg")
                self._handle = self._robot.turn_to(
                    angle_deg=self._goal_heading,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )

    def is_done(self) -> bool:
        return self._is_done

    def cancel(self) -> None:
        if self._handle:
            self._handle.cancel()
            self._handle = None
        super().cancel()

    def on_exit(self) -> None:
        """Write waypoints and vertices to mission data."""
        for wp in self._waypoints: self._mission_data["waypoints"].append(wp)
        for v in self._vertices: self._mission_data["vertices"].append(v)


class WaitTask(Task):
    def __init__(self, robot, mission_data, params: dict):
        super().__init__(robot, mission_data)
        self._params = params
        self._is_done = False
        self._handle = None
        self._time_pause_period = 2.0
        self._time_pause_start = None
        self._time_detection_period = 2.0
        self._time_last_detection = 0.0
        self._traffic_light_detected = False

    def update(self):
        now = time.monotonic()

        if self._params.get("trigger") == "green_light":
            # 1. Check for traffic light first (Priority)
            if vh.sees_traffic_light(self._robot):
                self._traffic_light_detected = True
                if now - self._time_last_detection >= self._time_detection_period:
                    self._time_last_detection = now
                    print("[WaitTask] Traffic light detected")
                self._robot.stop()
                if self._handle:
                    self._handle.cancel()
                    self._handle = None
                
                detected_color = vh.find_traffic_light_color(self._robot)
                if detected_color == "green":
                    self._is_done = True
                    print("[WaitTask] Completion triggered by green light")
                return # Don't turn if we see a light (even if it's red)

            # 2. Handle the turn/pause search cycle
            if self._handle is not None:
                # We are currently in the middle of a turn
                if self._handle.is_done():
                    self._handle = None
                    self._time_pause_start = now
                    print("[WaitTask] Turn complete, pausing for vision...")
            elif not self._traffic_light_detected:
                # We are currently in a pause period
                if now - self._time_pause_start >= self._time_pause_period:
                    print("[WaitTask] Pause complete, rotating search...")
                    self._handle = self._robot.turn_by(
                        delta_deg=20.0,
                        blocking=False,
                        tolerance_deg=2.0,
                        timeout=None
                    )

    def on_enter(self) -> None:
        self._time_pause_start = time.monotonic()
        print(f"[WaitTask] Searching for traffic light")

    def on_exit(self) -> None:
        self._robot.stop()
        print(f"[WaitTask] EXIT")

    def cancel(self) -> None:
        if self._handle:
            self._handle.cancel()
            self._handle = None
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done


class ManipTask(Task):
    def __init__(self, robot, mission_data, params: dict):
        super().__init__(robot, mission_data)
        self._is_done = False
        
        command = params.get("command")
        if command == "pick":
            self._sequence = bp.PICK_SEQUENCE
            ingredient = params.get("ingredient")
            self._raise_steps = hm.LIFT_LIFTOFF_STEPS
            if ingredient == "bun":
                self._raise_steps -= ih.Bun.HEIGHT_STEPS
            elif ingredient == "patty":
                # self._raise_steps -= ih.Patty.HEIGHT_STEPS
                self._raise_steps -= -1000

        elif command == "place":
            self._sequence = bp.PLACE_SEQUENCE
            self._raise_steps = hm.LIFT_LIFTOFF_STEPS
            for ingredient in self._mission_data["burger_stack"]:
                if ingredient == "bun":
                    self._raise_steps -= ih.Bun.HEIGHT_STEPS
                elif ingredient == "patty":
                    self._raise_steps -= ih.Patty.HEIGHT_STEPS
        else:
            self._sequence = []
            self._is_done = True

        self._ingredient = params.get("ingredient")
        if self._ingredient == "bun":
            self._level_height = hm.LIFT_LIFTOFF_STEPS - ih.Bun.HEIGHT_STEPS
            self._close_deg = ih.Bun.GRIP_ANGLE
        elif self._ingredient == "patty":
            self._level_height = hm.LIFT_LIFTOFF_STEPS - ih.Patty.HEIGHT_STEPS
            self._close_deg = ih.Patty.GRIP_ANGLE
        elif command == "place":
            self._level_height = -600 # TODO: Add parameter (LOW)
        
        self._idx = 0
        self._handle = None
        self._stage_init = True
        self._pre_approach_pose = None

    def on_enter(self) -> None:
        """Enable manipulator hardware once at task start."""
        self._robot.step_enable(hm.LIFT_STEPPER_ID)
        self._robot.enable_servo(hm.GRIPPER_SERVO_ID)

    def on_exit(self) -> None:
        """Disable power-hungry manipulator hardware upon exit."""
        self._robot.step_disable(hm.LIFT_STEPPER_ID)
        self._mission_data["burger_stack"].append(self._ingredient)

    def update(self):
        if self._is_done:
            return

        stage = self._sequence[self._idx]

        if self._stage_init:
            print(f"[Maniptask] {stage.name}")
            if stage == bp.ManipStage.LEVEL:
                self._handle = fh.move_lift(
                    robot=self._robot,
                    position=self._level_height,
                    move_type=hm.StepMoveType.ABSOLUTE,
                    disable_on_done=False
                )
            elif stage == bp.ManipStage.OPEN:
                self._handle = fh.set_gripper(
                    robot=self._robot,
                    angle=hm.GRIPPER_OPEN_DEG
                )
            elif stage == bp.ManipStage.CLOSE:
                self._handle = fh.set_gripper(
                    robot=self._robot,
                    angle=self._close_deg
                )
            elif stage == bp.ManipStage.FORWARD:
                self._pre_approach_pose = self._robot.get_pose()
                self._handle = fh.approach_ingredient_table(self._robot)
            elif stage == bp.ManipStage.RETREAT:
                x, y, _ = self._robot.get_pose()

                if self._pre_approach_pose is not None:
                    px, py, _ = self._pre_approach_pose
                    backoff_distance = math.dist([x, y], [px, py])
                else:
                    backoff_distance = 100.0
                
                print(f"[ManipTask] Retreating {backoff_distance:.1f} mm")
                self._handle = self._robot.move_backward(
                    distance=backoff_distance,
                    velocity=bp.VELOCITY_MM_S,
                    tolerance=bp.TOLERANCE_MM,
                    blocking=False
                )
            elif stage == bp.ManipStage.LOWER:
                self._handle = fh.move_lift(
                    robot=self._robot,
                    position=hm.LIFT_LIFTOFF_STEPS,
                    move_type=hm.StepMoveType.ABSOLUTE,
                    disable_on_done=False
                )
            elif stage == bp.ManipStage.RAISE:
                self._handle = fh.move_lift(
                    robot=self._robot,
                    position=self._raise_steps,
                    move_type=hm.StepMoveType.ABSOLUTE,
                    disable_on_done=False
                )
            
            self._stage_init = False

        # Check for completion
        done = False
        if self._handle:
            if self._handle.is_done():
                self._handle = None
                done = True
        
        if done:
            self._idx += 1
            if self._idx < len(self._sequence):
                self._stage_init = True
            else:
                self._is_done = True

    def cancel(self) -> None:
        if self._handle:
            self._handle.cancel()
            self._handle = None
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done


class IdentTask(Task):
    def __init__(self, robot, mission_data):
        super().__init__(robot, mission_data)
        self._is_done = False
        self._last_attempt_time = 0
        self._attempt_period = 2.0 # Wait 2s between identification attempts

    def update(self):
        now = time.monotonic()
        if now - self._last_attempt_time < self._attempt_period:
            return
        
        self._last_attempt_time = now
        
        if vh.capture_and_crop_identify_person(robot=self._robot, save_path=vh.IDENTIFY_PERSON_PATH):
            score_1 = vh.image_match_score(vh.IDENTIFY_PERSON_PATH, vh.SUSPECT_1_PATH)
            score_2 = vh.image_match_score(vh.IDENTIFY_PERSON_PATH, vh.SUSPECT_2_PATH)
            print(f"[IdentTask] Match results: Suspect1={score_1}, Suspect2={score_2}")

            if score_1 < vh.MIN_IMAGE_MATCH_SCORE and score_2 < vh.MIN_IMAGE_MATCH_SCORE:
                print("[IdentTask] UPDATE: No confident image match found. Will retry...")
            elif score_1 > score_2:
                self._mission_data["matched_customer"] = 1
                self._mission_data["delivery_status"] = False
                self._is_done = True
                print("[IdentTask] UPDATE: Matched to customer 1.")
            elif score_2 > score_1:
                self._mission_data["matched_customer"] = 2
                self._mission_data["delivery_status"] = False
                self._is_done = True
                print("[IdentTask] UPDATE: Matched to customer 2.")
        else:
            print(f"[IdentTask] UPDATE: Capture and identification failure. Will retry...")

    def cancel(self) -> None:
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done
    
    def on_enter(self):
        print("[IdentTask] ENTER: Analyzing target customer.")

    def on_exit(self):
        print("[IdentTask] EXIT: Customer match complete.")


def build_task(robot: Robot, task_dict: dict, mission_data: dict) -> Task:
    state = task_dict.get("state")

    if state == "NAV":
        waypoints = list(task_dict.get("waypoints", []))
        goal_heading = task_dict.get("goal_heading")
        delivery_segment = False

        # Robustness: Inject customer data BEFORE path generation
        if not mission_data.get("delivery_status"):
            matched_customer = mission_data.get("matched_customer")
            if matched_customer == 1:
                waypoints.append(cp.WP_CUSTOMER_1)
                goal_heading = 0
                delivery_segment = True
            elif matched_customer == 2:
                waypoints.append(cp.WP_CUSTOMER_2)
                goal_heading = 0
                delivery_segment = True
            
        return NavTask(
            robot=robot,
            mission_data=mission_data,
            waypoints=waypoints,
            goal_heading=goal_heading,
            path_planner=task_dict.get("path_planner", "pp"),
            delivery_segment=delivery_segment
        )
    elif state == "WAIT":
        return WaitTask(robot, mission_data, task_dict)
    elif state == "MANIP":
        return ManipTask(robot, mission_data, task_dict)
    elif state == "IDENT":
        return IdentTask(robot, mission_data)
    elif state == "HOME":
        return HomeTask(robot, mission_data)
    else:
        return Task(robot, mission_data)
