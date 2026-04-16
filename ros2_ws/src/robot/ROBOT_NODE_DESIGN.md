# Robot Node Design

## Overview

The `robot` package is structured in three layers on top of the existing `bridge` node:

```
┌──────────────────────────────────────────────┐
│  Layer 3 — Path Planner (PathPlanner)         │  pure pursuit; APF added when lidar ready
├──────────────────────────────────────────────┤
│  Layer 2 — main.py state loop                 │  student-authored logic + helper functions
├──────────────────────────────────────────────┤
│  Layer 1 — Robot API (Robot)                  │  wraps all bridge topics/services
├──────────────────────────────────────────────┤
│  bridge node  (existing)                      │  raw ROS ↔ firmware TLV
└──────────────────────────────────────────────┘
```

`Robot` is a plain Python class, not a ROS node. It is constructed with a reference to
the live `RobotNode` and uses it to create subscriptions and publishers. The `RobotNode`
(`robot_node.py`) is the only ROS node in this package.

**Student entry point is `main.py`.** Students write their FSM there and do not touch
`robot_node.py`. `robot_node.py` handles all ROS setup and then calls `main.run(robot)`.

---

## Layer 1 — Robot (`robot.py`)

### Construction

```python
from enum import Enum

class Unit(Enum):
    MM   = 1.0    # native firmware units; no conversion
    INCH = 25.4   # 1 inch = 25.4 mm

class Robot:
    def __init__(self, node: rclpy.node.Node, unit: Unit = Unit.MM,
                 wheel_diameter_mm: float = 74.0,
                 wheel_base_mm: float = 333.0,
                 encoder_ppr: int = 1440):
```

`unit` controls every length/velocity input and output used by the public
high-level API. Runtime odometry geometry set through `set_odometry_parameters()`
follows the current user unit system; the explicit `*_mm` helpers remain raw mm.
Internally everything is stored and sent in mm / ticks. The conversion is:

```
mm = user_value × unit.value
user_value = mm / unit.value
ticks_per_s = mm_per_s × encoder_ppr / (π × wheel_diameter_mm)
```

### Internal state

`Robot` keeps a local mirror of firmware state, updated by ROS subscriptions. A
`threading.Lock` protects the shared state because the ROS spin thread writes it and the
FSM/path-planner threads read it.

---

### Full API

#### System

| Method | Blocking | Notes |
|---|---|---|
| `set_state(state: FirmwareState)` | yes (service) | transitions IDLE/RUNNING/ESTOP |
| `get_state() → FirmwareState` | no | reads cached `/sys_state` |
| `estop()` | yes | shorthand for `set_state(ESTOP)` |
| `reset_estop()` | yes | shorthand for `set_state(IDLE)` |
| `reset_odometry()` | no | publishes `/sys_odom_reset`; resets pose to `(0, 0, current initial theta)` |
| `set_odometry_parameters(wheel_diameter=None, wheel_base=None, initial_theta_deg=None, left_motor_id=None, left_motor_dir_inverted=None, right_motor_id=None, right_motor_dir_inverted=None)` | no | publishes `/sys_odom_param_set`; `wheel_diameter` and `wheel_base` are in the current user unit system |
| `set_wheel_diameter_mm(mm)` | no | publishes `/sys_odom_param_set`; updates wheel diameter in explicit millimeters |
| `set_wheel_base_mm(mm)` | no | publishes `/sys_odom_param_set`; updates wheel base in explicit millimeters |
| `set_initial_theta(theta_deg)` | no | publishes `/sys_odom_param_set`; used on future odom resets |
| `set_odom_left_motor(motor_id)` / `set_odom_right_motor(motor_id)` | no | select which DC motors represent the diff-drive wheels |
| `set_odom_left_motor_dir_inverted(flag)` / `set_odom_right_motor_dir_inverted(flag)` | no | configure wheel-sign convention for odometry and high-level body motion |
| `request_odometry_parameters()` | no | publishes `/sys_odom_param_req`; asks firmware for the active runtime odom snapshot |
| `get_odometry_parameters() → dict` | no | returns the latest local / `/sys_odom_param_rsp` snapshot in firmware-native millimeters |
| `get_power() → SystemPower` | no | cached `/sys_power` |

#### Pose / odometry

| Method | Notes |
|---|---|
| `get_pose() → (x, y, theta_deg)` | x, y in user units; theta in degrees |
| `get_velocity() → (vx, vy, v_theta_deg_s)` | vx, vy in user units/s; v_theta in degrees/s |
| `wait_for_pose_update(timeout=None) → bool` | blocks until next `/sensor_kinematics` tick (~25 Hz) |

#### Differential drive — velocity

| Method | Blocking | Notes |
|---|---|---|
| `set_velocity(linear, angular_deg_s)` | no | body-frame: linear in user-units/s, angular in degrees/s; robot API uses the current odom wheel mapping and inversion flags |
| `set_left_wheel(motor_id)` / `set_right_wheel(motor_id)` | no | compatibility aliases for `set_odom_left_motor()` / `set_odom_right_motor()` |
| `set_motor_velocity(motor_id, velocity)` | no | per-motor; velocity in user-units/s → ticks/s |
| `stop()` | no | zero velocity on both drive motors; does NOT ESTOP |

Diff-drive mixing done by the API:
```
left_mm_s  = linear_mm_s − angular_rad_s × wheel_base_mm / 2
right_mm_s = linear_mm_s + angular_rad_s × wheel_base_mm / 2
```

If an odom wheel direction is marked inverted, the corresponding high-level
wheel command is negated before it is published. Raw `set_motor_velocity()`
remains untouched.

#### Differential drive — navigation (motion commands)

All navigation commands check that firmware is in RUNNING state before sending.

These are the high-level base-motion APIs. They always return a `MotionHandle`.

`blocking=True` — method waits internally until the goal is reached or `timeout`
expires, then returns the already-finished `MotionHandle`.

`blocking=False` — method returns immediately with a live `MotionHandle` that can be
polled, waited on, or cancelled later.

| Method | Default | Notes |
|---|---|---|
| `move_to(x, y, velocity, tolerance, blocking=True, timeout=None) → MotionHandle` | blocking | navigate to (x, y) in user units at velocity user-units/s |
| `move_by(dx, dy, velocity, tolerance, blocking=True, timeout=None) → MotionHandle` | blocking | relative move in the current user unit system |
| `move_forward(distance, velocity, tolerance, blocking=True, timeout=None) → MotionHandle` | blocking | move forward along the robot's current heading |
| `move_backward(distance, velocity, tolerance, blocking=True, timeout=None) → MotionHandle` | blocking | move backward along the robot's current heading |
| `turn_to(angle_deg, blocking=True, tolerance_deg=2, timeout=None) → MotionHandle` | blocking | rotate to absolute heading |
| `turn_by(delta_deg, blocking=True, tolerance_deg=2, timeout=None) → MotionHandle` | blocking | rotate by delta |
| `purepursuit_follow_path(path, velocity, lookahead, tolerance, blocking=True, max_angular_rad_s=1.0, timeout=None, *, advance_radius=None) → MotionHandle` | blocking | follow an ordered waypoint path with pure pursuit; `advance_radius` defaults to `tolerance` |
| `apf_follow_path(path, velocity, lookahead, tolerance, repulsion_range, blocking=True, max_angular_rad_s=1.0, repulsion_gain=500.0, timeout=None, *, advance_radius=None) → MotionHandle` | blocking | follow an ordered waypoint path with APF obstacle avoidance; `advance_radius` defaults to `tolerance` |
| `is_moving() → bool` | — | True if a motion command is active |
| `cancel_motion()` | — | abort current navigation command; calls `stop()` |

`MotionHandle`:
```python
handle.wait(timeout=None) → bool   # block until done
handle.is_finished() → bool        # non-blocking poll
handle.is_done() → bool            # backward-compatible alias
handle.cancel()                    # abort
```

This handle pattern is intentionally limited to high-level base motion. It is
useful when student code wants to:
- start a long-running path-follow or turn command
- keep a simple state loop responsive
- poll `is_finished()` from `main.py`

Low-level actuator APIs below keep a simpler command/wait contract instead of
introducing handles everywhere.

#### DC motors — low-level

| Method | Blocking | Notes |
|---|---|---|
| `set_motor_pwm(motor_id, pwm)` | no | raw PWM −255…255 |
| `set_motor_position(motor_id, ticks, blocking=True, timeout=None) → bool` | optional | position control mode; returns `True` on completion, `False` on timeout |
| `enable_motor(motor_id, mode=DCMotorMode.VELOCITY)` | no | publishes `DCEnable`; mode follows firmware enum |
| `disable_motor(motor_id)` | no | publishes `DCEnable` |
| `home_motor(motor_id, blocking=True, timeout=None) → bool` | optional | publishes `DCHome`; returns `True` on completion, `False` on timeout |
| `reset_motor_position(motor_id)` | no | publishes `DCResetPosition` |
| `set_pid_gains(motor_id, loop_type, kp, ki, kd, max_output, max_integral)` | no | publishes `DCPidSet`; `loop_type` uses `DCPidLoop` (`POSITION=0`, `VELOCITY=1`) |
| `request_pid(motor_id, loop_type)` | no | publishes `DCPidReq`; result arrives on `/dc_pid_rsp` |
| `get_dc_state() → DCStateAll` | no | cached `/dc_state_all` |

#### Stepper motors

| Method | Blocking | Notes |
|---|---|---|
| `step_enable(stepper_id)` / `step_disable(stepper_id)` | no | |
| `step_move(stepper_id, steps, move_type=StepMoveType.RELATIVE, blocking=True, timeout=None) → bool` | optional | publishes `StepMove`; `move_type` uses the firmware enum; returns `True` on completion, `False` on timeout |
| `step_home(stepper_id, blocking=True, timeout=None) → bool` | optional | publishes `StepHome`; returns `True` on completion, `False` on timeout |
| `step_set_config(stepper_id, ...)` | no | publishes `StepConfigSet` |
| `get_step_state() → StepStateAll` | no | cached `/step_state_all` |

#### Servos

| Method | Notes |
|---|---|
| `set_servo(channel, angle_deg)` | maps angle to pulse width; publishes `ServoSet` |
| `enable_servo(channel)` / `disable_servo(channel)` | publishes `ServoEnable` |
| `get_servo_state() → ServoStateAll` | cached `/servo_state_all` |

#### IO — buttons, limits, LEDs

| Method | Blocking | Notes |
|---|---|---|
| `get_button(button_id) → bool` | no | reads bitmask from cached `IOInputState`; ids 1–10 on current firmware |
| `was_button_pressed(button_id, consume=True) → bool` | no | rising-edge latch captured in ROS callback |
| `wait_for_button(button_id, timeout=None) → bool` | yes | blocks until button pressed or timeout |
| `get_limit(limit_id) → bool` | no | same approach as buttons; ids 1–8 on current firmware |
| `was_limit_triggered(limit_id, consume=True) → bool` | no | rising-edge latch captured in ROS callback |
| `wait_for_limit(limit_id, timeout=None) → bool` | yes | blocks until limit triggered |
| `set_led(led_id, brightness, mode=None, period_ms=None, duty_cycle=500)` | no | publishes `IOSetLed`; `mode` uses `LEDMode` (`OFF`, `ON`, `BLINK`, `BREATHE`, `PWM`); blink/breathe default to 1000 ms and `duty_cycle=500` |
| `set_neopixel(index, r, g, b)` | no | publishes `IOSetNeopixel`; index is 0-based |

`wait_for_button` and `was_button_pressed` use work done inside the IO
subscription callback when the requested bit transitions from 0→1, so short
button presses are not missed even if the FSM loop runs slower. For simple
FSMs that watch one button per state, `get_button()` is often the cleaner
choice because it avoids carrying a latched edge across a later state change.

For LED timing, `duty_cycle` is in permille (`0..1000`). In `BLINK` mode it is
the ON-time share of the full period. In `BREATHE` mode it is the rise-time
share of the full period, with the remaining time used for the fade back down.

#### IMU

| Method | Notes |
|---|---|
| `get_imu() → SensorImu` | cached `/sensor_imu` |

#### Units

| Method | Notes |
|---|---|
| `set_unit(unit: Unit)` | changes scale factor; does not affect already-sent commands |
| `get_unit() → Unit` | |

---

## Layer 2 — `main.py` state loop and custom helpers

Students now work directly in `main.py`. The intended structure is:
- one explicit `state` variable
- one monotonic-timed loop in `run(robot)`
- helper functions for custom actions or sequences

This keeps the mission logic readable without requiring students to learn
inheritance or a transition framework before they can use the robot.

### Minimal student example

```python
def run(robot):
    state = "IDLE"
    motion = None
    period = 1.0 / 50.0
    next_tick = time.monotonic()

    while True:
        if state == "IDLE":
            if robot.get_button(1):
                motion = robot.move_forward(
                    300.0,
                    100.0,
                    tolerance=20.0,
                    blocking=False,
                )
                state = "MOVING"

        elif state == "MOVING":
            if motion is not None and motion.is_finished():
                robot.stop()
                state = "IDLE"

        next_tick += period
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_tick = time.monotonic()
```

### Custom background sequences

For student-defined arm/manipulator logic, `main.py` can use the local task
helpers from `robot.util`:

```python
from robot.util import run_task

def my_sequence_1(robot, blocking=True):
    def worker(task):
        robot.set_servo(1, 30)
        task.sleep(0.5)
        robot.set_servo(2, 120)
        task.sleep(0.5)

    return run_task(worker, blocking=blocking)
```

This is intentionally separate from `MotionHandle`:
- base motion uses `MotionHandle`
- custom `main.py` sequences use `TaskHandle`
- low-level actuator move/home methods keep a simple `bool` completion result

---

## Layer 3 — PathPlanner (`path_planner.py`)

Path planners are **pure algorithm classes** — no threads, no ROS subscriptions.
`Robot` calls `compute_velocity()` from its own internal navigation thread.
Students who want a custom algorithm subclass
`PathPlanner` and implement `compute_velocity()`.

### Base class

```python
class PathPlanner:
    def compute_velocity(
        self,
        pose: tuple[float, float, float],   # (x, y, theta_rad) in consistent units
        waypoints: list[tuple[float, float]],
        max_linear: float,                   # max forward speed
    ) -> tuple[float, float]:               # (linear, angular_rad_s)
        raise NotImplementedError

    def get_obstacles(self) -> list:
        return []   # override when 2D lidar is available
```

### PurePursuitPlanner (ships with the package)

```python
class PurePursuitPlanner(PathPlanner):
    def __init__(self, lookahead_dist=150, max_angular=2.0): ...
    def compute_velocity(self, pose, waypoints, max_linear): ...
```

### APFPlanner (stub — activated when lidar is ready)

```python
class APFPlanner(PathPlanner):
    """Artificial potential fields. Override get_obstacles() when /scan is live."""
```

### Thread model

```
Main thread  ──► main.run(robot)
                   └─ explicit state loop in main.py
                        ├─ may call robot.move_to(..., blocking=False)  → MotionHandle
                        │    └─ Robot starts _nav_thread
                        │         └─ _nav_thread calls planner.compute_velocity()
                        │            and robot._send_body_velocity_mm() in a loop
                        └─ may call my_sequence_1(..., blocking=False)  → TaskHandle
                             └─ custom worker thread defined in main.py

ROS spin thread ──► updates Robot internal state (pose, buttons, etc.)
```

Locks:
- `Robot._lock` (threading.Lock) — protects all cached topic state
- `Robot._nav_cancel/done` (threading.Event) — coordinate nav thread lifecycle

---

## Key constants

These are constructor defaults in `Robot` and should match `firmware/arduino/src/config.h`:

| Constant | Value | Source |
|---|---|---|
| `wheel_diameter_mm` | 74.0 | `WHEEL_DIAMETER_MM` |
| `wheel_base_mm` | 333.0 | `WHEEL_BASE_MM` |
| `initial_theta_deg` | 90.0 | `INITIAL_THETA` |
| `encoder_ppr` | 1440 | `ENCODER_PPR × ENCODER_4X` |
| Default left drive motor | 1 | `ODOM_LEFT_MOTOR + 1` |
| Default right drive motor | 2 | `ODOM_RIGHT_MOTOR + 1` |
| Default left wheel direction inverted | `False` | `ODOM_LEFT_MOTOR_DIR_INVERTED` |
| Default right wheel direction inverted | `True` | `ODOM_RIGHT_MOTOR_DIR_INVERTED` |

Student code can override wheel mapping and odometry geometry at startup via
the `set_*` odometry-parameter helpers if the robot is wired differently.

---

## File layout

```
robot/
├── robot/
│   ├── __init__.py
│   ├── hardware_map.py        # user-facing IDs + firmware-derived timing constants
│   ├── robot_node.py          # ROS node; constructs Robot, spins ROS, calls main.run()
│   ├── robot.py               # Robot class (Layer 1)
│   ├── path_planner.py        # PathPlanner base + PurePursuitPlanner (Layer 3)
│   ├── util.py                # TaskHandle, run_task(), densify_polyline()
│   ├── main.py                # ← students edit this; explicit state loop entry point
│   └── examples/
│       ├── square_drive.py    # drive a square using move_to
│       ├── button_fsm.py      # older FSM-style example
│       └── move_servos.py     # custom TaskHandle sequence example
```

`main.py` is the primary file students edit. It defines helper functions and a
`run(robot)` loop that `robot_node.py` calls after ROS is set up.
