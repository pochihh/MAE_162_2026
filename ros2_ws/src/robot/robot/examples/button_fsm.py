"""
Example: Button-Triggered FSM
==============================
Demonstrates a multi-state FSM driven entirely by button presses.
No path planning — useful for testing hardware and understanding the FSM
structure before adding motion.

HOW TO APPLY THIS EXAMPLE
--------------------------
Copy ButtonFSM and run() into your main.py. Then run:

    ros2 run robot robot

You can also keep your own FSM and borrow individual state actions (e.g.,
the LED feedback pattern or the blocking button wait trick).

WHAT IT DOES
------------
States and transitions:

    IDLE ──(button 1)──► FORWARD   robot drives forward at 150 mm/s
    FORWARD ──(button 2)──► TURNING  robot turns 90° CCW in place
    TURNING ──(turn done)──► IDLE    robot stops; discrete LEDs indicate state

Button 1 always restarts from IDLE.
Button 3 triggers ESTOP from any state.

TWO STYLES OF BUTTON HANDLING SHOWN
--------------------------------------
1. Latched edge polling in update()  — used for buttons 1, 2, and 3.
   Fast response without missing short button presses.

2. Blocking wait in an action — used to confirm the turn is done.
   Blocks the FSM loop; only use for short waits.
"""

import time

from robot.robot import Robot, FirmwareState
from robot.robot_fsm import RobotFSM
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED


class ButtonFSM(RobotFSM):

    ALL_STATES = ["IDLE", "FORWARD", "TURNING"]

    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="IDLE")

        self.add_transition("IDLE",    "go",    "FORWARD", action=self._start_forward)
        self.add_transition("FORWARD", "turn",  "TURNING", action=self._start_turn)
        self.add_transition("TURNING", "done",  "IDLE",    action=self._on_idle)

        # ESTOP from any state
        for s in self.ALL_STATES:
            self.add_transition(s, "estop", "IDLE", action=self._emergency_stop)

    # ------------------------------------------------------------------
    # Polling  (called every spin cycle)
    # ------------------------------------------------------------------

    def update(self) -> None:
        # Button 3 — emergency stop from any state
        if self.robot.was_button_pressed(Button.BTN_3):
            self.trigger("estop")
            return

        state = self.get_state()

        if state == "IDLE":
            if self.robot.was_button_pressed(Button.BTN_1):
                self.trigger("go")

        elif state == "FORWARD":
            if self.robot.was_button_pressed(Button.BTN_2):
                self.trigger("turn")

        elif state == "TURNING":
            # Transition when motion completes
            if not self.robot.is_moving():
                self.trigger("done")

    # ------------------------------------------------------------------
    # State actions
    # ------------------------------------------------------------------

    def _start_forward(self) -> None:
        self.robot.set_state(FirmwareState.RUNNING)
        self.robot.set_velocity(linear=150, angular_deg_s=0)
        self._show_led(LED.BLUE)

    def _start_turn(self) -> None:
        self.robot.stop()
        # Non-blocking turn; update() watches is_moving() for completion
        self.robot.turn_by(90, blocking=False, tolerance_deg=3)
        self._show_led(LED.ORANGE)

    def _on_idle(self) -> None:
        self.robot.stop()
        # Flash red LED three times to signal return to IDLE
        for _ in range(3):
            self.robot.set_led(LED.RED, 255)
            time.sleep(0.15)
            self.robot.set_led(LED.RED, 0)
            time.sleep(0.15)
        self._show_led(LED.GREEN)

    def _emergency_stop(self) -> None:
        self.robot.cancel_motion()
        self.robot.stop()
        self._show_led(LED.RED)
        # Blocking wait: hold here until button 1 is pressed to re-arm
        # This intentionally blocks the FSM loop — the robot is stopped.
        self.robot.wait_for_button(Button.BTN_1)
        self._clear_leds()

    def _clear_leds(self) -> None:
        for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
            self.robot.set_led(led, 0)

    def _show_led(self, led: LED) -> None:
        self._clear_leds()
        self.robot.set_led(led, 255)


def run(robot: Robot) -> None:
    fsm = ButtonFSM(robot)
    fsm.spin(hz=DEFAULT_FSM_HZ)
