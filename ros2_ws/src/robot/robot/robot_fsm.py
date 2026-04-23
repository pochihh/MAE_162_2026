from __future__ import annotations

import time
import threading
from typing import Callable

from robot.hardware_map import Button, DEFAULT_FSM_HZ
from robot.robot import Robot


class RobotFSM:
    """
    Base class for student-authored finite state machines.

    Usage:
        class MyFSM(RobotFSM):
            def __init__(self, robot):
                super().__init__(robot)
                self.add_transition("IDLE", "start", "MOVING", action=self._begin)
                self.add_transition("MOVING", "done", "IDLE",  action=self._finish)

            def update(self):
                if self.get_state() == "IDLE" and self.robot.get_button(Button.BTN_1):
                    self.trigger("start")

            def _begin(self):
                self.robot.set_velocity(100, 0)

            def _finish(self):
                self.robot.stop()

        fsm = MyFSM(robot)
        fsm.spin()              # blocks; runs update() at firmware-matched rate
    """

    def __init__(self, robot: Robot, initial_state: str = "IDLE") -> None:
        self.robot = robot
        self._state: str = initial_state
        self._lock = threading.Lock()
        # {from_state: {event: (to_state, action, guard)}}
        self._transitions: dict[str, dict[str, tuple]] = {}

    # =========================================================================
    # Builder
    # =========================================================================

    def add_transition(
        self,
        from_state: str,
        event: str,
        to_state: str,
        action: Callable[[], None] = None,
        guard: Callable[[], bool] = None,
    ) -> None:
        """
        Register a state transition.

        from_state  — state this transition starts from
        event       — label used when calling trigger()
        to_state    — state to move to when triggered
        action      — optional callable executed after the transition
        guard       — optional callable; transition is blocked if guard() returns False
        """
        self._transitions.setdefault(from_state, {})[event] = (to_state, action, guard)

    # =========================================================================
    # Runtime
    # =========================================================================

    def trigger(self, event: str) -> bool:
        """
        Fire an event. Thread-safe.
        Returns True if the transition occurred, False if there is no matching
        transition from the current state or if the guard rejected it.
        """
        with self._lock:
            current = self._state
            trans = self._transitions.get(current, {}).get(event)
            if trans is None:
                return False
            to_state, action, guard = trans
            if guard is not None and not guard():
                return False
            self.on_exit(current)
            self._state = to_state

        if action is not None:
            action()
        self.on_enter(to_state)
        return True

    def get_state(self) -> str:
        with self._lock:
            return self._state

    def spin(self, hz: float = float(DEFAULT_FSM_HZ)) -> None:
        """
        Blocking main loop. Calls update() at the requested frequency.
        Run this in the main thread after setting up transitions.
        """
        period = 1.0 / hz
        next_tick = time.monotonic()
        while True:
            self.update()
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    # =========================================================================
    # Hooks  (override in subclass)
    # =========================================================================

    def on_enter(self, state: str) -> None:
        """Called immediately after entering state. Override per-state logic here."""

    def on_exit(self, state: str) -> None:
        """Called immediately before leaving state."""

    def update(self) -> None:
        """
        Called every spin cycle. Override to add polling-based transition logic,
        e.g., checking button states or sensor thresholds.
        """
