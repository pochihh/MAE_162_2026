"""
Example: Square Drive
=====================
Demonstrates move_to() and turn_by() by driving the robot in a 1-meter square.

HOW TO APPLY THIS EXAMPLE
--------------------------
Copy the SquareFSM class and the run() function into your main.py, replacing
the existing MyFSM class and run() function. Then run as normal:

    ros2 run robot robot

PREREQUISITES
-------------
- The bridge node must be running (ros2 run bridge bridge)
- The robot must be on a flat surface with ~1.2 m of clear space in front
  and to the left.

WHAT IT DOES
------------
State machine:
    IDLE → SIDE_1 → TURN_1 → SIDE_2 → TURN_2 → SIDE_3 → TURN_3 → SIDE_4 → DONE

Press button 1 to start. The robot drives four 1000 mm sides, turning 90°
left between each. After the last side it stops and returns to IDLE.

TUNING
------
- Change SIDE_LENGTH_MM to adjust the square size.
- Change DRIVE_SPEED to adjust the speed (mm/s).
- Adjust move_to() tolerance parameter if the robot overshoots corners.
- The turn_by() tolerance_deg can be tightened if headings drift.
"""

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED

SIDE_LENGTH_MM = 1000   # length of each side in mm
DRIVE_SPEED    = 200    # mm/s


class SquareFSM(RobotFSM):

    SIDES = ["SIDE_1", "SIDE_2", "SIDE_3", "SIDE_4"]
    TURNS = ["TURN_1", "TURN_2", "TURN_3"]

    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="IDLE")

        # IDLE → first side
        self.add_transition("IDLE",   "start",    "SIDE_1", action=self._begin_side)

        # Each side → turn (or done for the last side)
        self.add_transition("SIDE_1", "arrived",  "TURN_1", action=self._begin_turn)
        self.add_transition("SIDE_2", "arrived",  "TURN_2", action=self._begin_turn)
        self.add_transition("SIDE_3", "arrived",  "TURN_3", action=self._begin_turn)
        self.add_transition("SIDE_4", "arrived",  "DONE",   action=self._finish)

        # Each turn → next side
        self.add_transition("TURN_1", "turned",   "SIDE_2", action=self._begin_side)
        self.add_transition("TURN_2", "turned",   "SIDE_3", action=self._begin_side)
        self.add_transition("TURN_3", "turned",   "SIDE_4", action=self._begin_side)

        # Emergency stop from any state
        for s in self.SIDES + self.TURNS + ["IDLE", "DONE"]:
            self.add_transition(s, "estop", "IDLE", action=self.robot.stop)

        self._side_origin = (0.0, 0.0)   # start of the current side (user units)
        self._side_dir    = (1.0, 0.0)   # unit vector for current side direction

    def update(self) -> None:
        state = self.get_state()
        if state == "IDLE":
            if self.robot.was_button_pressed(Button.BTN_1):
                self.trigger("start")
        elif state in self.SIDES:
            if not self.robot.is_moving():
                self.trigger("arrived")
        elif state in self.TURNS:
            if not self.robot.is_moving():
                self.trigger("turned")
        # Emergency stop button
        if self.robot.was_button_pressed(Button.BTN_2):
            self.trigger("estop")

    def _begin_side(self) -> None:
        x, y, _ = self.robot.get_pose()
        # Compute destination: current position + SIDE_LENGTH along current direction
        scale = SIDE_LENGTH_MM / self.robot.get_unit().value
        dx = self._side_dir[0] * scale
        dy = self._side_dir[1] * scale
        self.robot.set_state(FirmwareState.RUNNING)
        self.robot.move_to(x + dx, y + dy, DRIVE_SPEED, blocking=False)
        self.robot.set_led(LED.GREEN, 255)
        self.robot.set_led(LED.ORANGE, 0)
        self.robot.set_led(LED.RED, 0)

    def _begin_turn(self) -> None:
        # Rotate direction vector 90° CCW for next side
        dx, dy = self._side_dir
        self._side_dir = (-dy, dx)
        self.robot.turn_by(90, blocking=False, tolerance_deg=3)
        self.robot.set_led(LED.GREEN, 0)
        self.robot.set_led(LED.ORANGE, 255)

    def _finish(self) -> None:
        self.robot.stop()
        self.robot.set_led(LED.ORANGE, 0)
        self.robot.set_led(LED.GREEN, 255)
        self.robot.set_led(LED.RED, 0)


def run(robot: Robot) -> None:
    fsm = SquareFSM(robot)
    fsm.spin(hz=DEFAULT_FSM_HZ)
