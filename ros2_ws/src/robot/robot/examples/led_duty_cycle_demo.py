"""
Example: LED Duty-Cycle Demo
============================
Exercises the discrete LED modes without moving the robot.

HOW TO APPLY THIS EXAMPLE
-------------------------
Copy LedDutyCycleFSM and run() into your main.py. Then run:

    ros2 run robot robot

CONTROLS
--------
- Button 1: next pattern
- Button 2: previous pattern
- Button 3: turn all discrete LEDs off

NOTES
-----
- RED, BLUE, and ORANGE are the best LEDs for PWM/BREATHE tests.
- GREEN is effectively binary.
- PURPLE is digital only and cannot show smooth dimming.
"""

from robot.robot import FirmwareState, Robot
from robot.robot_fsm import RobotFSM
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, LEDMode


PATTERNS = [
    {
        "name": "RED blink 50%",
        "led": LED.RED,
        "mode": LEDMode.BLINK,
        "brightness": 255,
        "period_ms": 1000,
        "duty_cycle": 500,
    },
    {
        "name": "RED blink 20%",
        "led": LED.RED,
        "mode": LEDMode.BLINK,
        "brightness": 255,
        "period_ms": 1000,
        "duty_cycle": 200,
    },
    {
        "name": "BLUE blink 80%",
        "led": LED.BLUE,
        "mode": LEDMode.BLINK,
        "brightness": 255,
        "period_ms": 1000,
        "duty_cycle": 800,
    },
    {
        "name": "RED breathe symmetric",
        "led": LED.RED,
        "mode": LEDMode.BREATHE,
        "brightness": 255,
        "period_ms": 1600,
        "duty_cycle": 500,
    },
    {
        "name": "BLUE breathe fast rise",
        "led": LED.BLUE,
        "mode": LEDMode.BREATHE,
        "brightness": 255,
        "period_ms": 1600,
        "duty_cycle": 250,
    },
    {
        "name": "ORANGE breathe slow rise",
        "led": LED.ORANGE,
        "mode": LEDMode.BREATHE,
        "brightness": 255,
        "period_ms": 1600,
        "duty_cycle": 750,
    },
    {
        "name": "BLUE PWM 25%",
        "led": LED.BLUE,
        "mode": LEDMode.PWM,
        "brightness": 64,
        "period_ms": None,
        "duty_cycle": 500,
    },
]


class LedDutyCycleFSM(RobotFSM):
    def __init__(self, robot: Robot) -> None:
        super().__init__(robot, initial_state="INIT")
        self.add_transition("INIT", "ready", "READY", action=self._on_ready)

        self._pattern_index = 0
        self._btn1_down = False
        self._btn2_down = False
        self._btn3_down = False

    def update(self) -> None:
        if self.get_state() == "INIT":
            self.trigger("ready")
            return

        btn1 = self.robot.get_button(Button.BTN_1)
        btn2 = self.robot.get_button(Button.BTN_2)
        btn3 = self.robot.get_button(Button.BTN_3)

        if btn1 and not self._btn1_down:
            self._pattern_index = (self._pattern_index + 1) % len(PATTERNS)
            self._apply_pattern()

        if btn2 and not self._btn2_down:
            self._pattern_index = (self._pattern_index - 1) % len(PATTERNS)
            self._apply_pattern()

        if btn3 and not self._btn3_down:
            self._clear_leds()
            print("[LED DEMO] all discrete LEDs off")

        self._btn1_down = btn1
        self._btn2_down = btn2
        self._btn3_down = btn3

    def _on_ready(self) -> None:
        self.robot.set_state(FirmwareState.RUNNING)
        print("[LED DEMO] Button 1 = next, Button 2 = previous, Button 3 = off")
        self._apply_pattern()

    def _clear_leds(self) -> None:
        for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
            self.robot.set_led(led, 0)

    def _apply_pattern(self) -> None:
        pattern = PATTERNS[self._pattern_index]
        self._clear_leds()
        self.robot.set_led(
            pattern["led"],
            pattern["brightness"],
            mode=pattern["mode"],
            period_ms=pattern["period_ms"],
            duty_cycle=pattern["duty_cycle"],
        )
        print(f"[LED DEMO] {self._pattern_index + 1}/{len(PATTERNS)}: {pattern['name']}")


def run(robot: Robot) -> None:
    fsm = LedDutyCycleFSM(robot)
    fsm.spin(hz=DEFAULT_FSM_HZ)
