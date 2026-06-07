from __future__ import annotations

from enum import Enum, IntEnum


class Unit(Enum):
    MM = 1.0
    INCH = 25.4
    AMERICAN = INCH
    REST_OF_THE_WORLD = MM


# Servo pulse ranges
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# PCA9685 Hardware Mapping
class Motor(IntEnum):
    DC_M1 = 1
    DC_M2 = 2
    DC_M3 = 3
    DC_M4 = 4


class Stepper(IntEnum):
    STEPPER_1 = 1
    STEPPER_2 = 2
    STEPPER_3 = 3
    STEPPER_4 = 4


class DCMotorMode(IntEnum):
    DISABLED = 0
    POSITION = 1
    VELOCITY = 2
    PWM = 3
    HOMING = 4


class DCPidLoop(IntEnum):
    POSITION = 0
    VELOCITY = 1


class StepMoveType(IntEnum):
    ABSOLUTE = 0
    RELATIVE = 1


class StepperMotionState(IntEnum):
    IDLE = 0
    ACCEL = 1
    CRUISE = 2
    DECEL = 3
    HOMING = 4


class LEDMode(IntEnum):
    OFF = 0
    ON = 1
    BLINK = 2
    BREATHE = 3
    PWM = 4


class ServoChannel(IntEnum):
    CH_1 = 1
    CH_2 = 2
    CH_3 = 3
    CH_4 = 4
    CH_5 = 5
    CH_6 = 6
    CH_7 = 7
    CH_8 = 8
    CH_9 = 9
    CH_10 = 10
    CH_11 = 11
    CH_12 = 12
    CH_13 = 13
    CH_14 = 14
    CH_15 = 15
    CH_16 = 16


class Button(IntEnum):
    BTN_1 = 1
    BTN_2 = 2
    BTN_3 = 3
    BTN_4 = 4
    BTN_5 = 5
    BTN_6 = 6
    BTN_7 = 7
    BTN_8 = 8
    BTN_9 = 9
    BTN_10 = 10


class Limit(IntEnum):
    LIM_1 = 1
    LIM_2 = 2
    LIM_3 = 3
    LIM_4 = 4
    LIM_5 = 5
    LIM_6 = 6
    LIM_7 = 7
    LIM_8 = 8


class LED(IntEnum):
    RED = 0
    GREEN = 1
    BLUE = 2
    ORANGE = 3
    PURPLE = 4


BUTTON_COUNT = 10
LIMIT_COUNT = 8


# These rates mirror firmware/arduino/src/config.h.
UART_TASK_HZ = 100
SYS_STATE_RUN_HZ = 10
SYS_POWER_RUN_HZ = 10
DC_STATE_HZ = 50
STEP_STATE_HZ = 50
SERVO_STATE_HZ = 10
IMU_HZ = 25
KINEMATICS_HZ = 25
IO_INPUT_HZ = 50
IO_OUTPUT_HZ = 10

# Robot-side defaults should match the fastest upstream signal they consume.
DEFAULT_FSM_HZ = IO_INPUT_HZ
DEFAULT_NAV_HZ = KINEMATICS_HZ

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 78.3   # TODO: Tune more (LOW)
WHEEL_BASE = 259        # TODO: Tune (HIGH)
INITIAL_THETA_DEG = 90.0

# Differential drive configuration
LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

# Lidar
LIDAR_MOUNT_X_MM = 187.8 # 235.8 - 48 mm
LIDAR_MOUNT_Y_MM = 0.0
LIDAR_MOUNT_THETA_DEG = 0.0
LIDAR_RANGE_MIN_MM = 70.0 # Note: C1 Specs: Min range = 50 mm, resolution = 15 mm
LIDAR_RANGE_MAX_MM = 3500.0
LIDAR_FOV_DEG = (-70, 70.0)

# Aruco tag
TAG_BODY_OFFSET_X_MM = 150.7 # TODO: Remeasure (HIGH)
TAG_BODY_OFFSET_Y_MM = 0.0
TAG_ID = 20

# Servo 1 — gripper jaw
GRIPPER_SERVO_ID = ServoChannel.CH_1
GRIPPER_LIMIT = Limit.LIM_2
GRIPPER_OPEN_DEG = 5.0
GRIPPER_HOME_DEG = 30.0
GRIPPER_SETTLE_S = 1.0
GRIPPER_SPEED = 90.0        # deg/s

# Stepper 1 — lift
LIFT_STEPPER_ID = Stepper.STEPPER_1
LIFT_LIFTOFF_STEPS = -1000   # abs
LIFT_VELOCITY = 6000
LIFT_ACCELERATION = 2000
LIFT_MOVE_TIMEOUT_S = 10.0

# Other limit switches
TABLE_LIMIT = Limit.LIM_3   # TODO: Verify (HIGH)

# PID Gains
WHEEL_VEL_PID = (0.15, 0.06, 0.0) # (P, I, D) TODO: Tune to new controller (MED)
DB_STATIC = 100 # PWM
DB_KINETIC = 90 # PWM
STOP_THRESHOLD = 50 # ticks/sec ~ 8mm/s TODO: Tune (LOW)