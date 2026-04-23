# Stall Detection for Position Fusion Test

## Problem

When the robot drives into a wall the encoders continue spinning, so odometry
keeps accumulating while the robot's actual position stays fixed.  If GPS is
active during this period the GPS reading remains stationary while odometry
diverges, producing a misleading fused−odometry error plot.  If GPS is not
active the test runs all the way to the safety cap (`DRIVE_DISTANCE_MM +
GPS_SEARCH_EXTRA_MM`) needlessly.

## Proposed implementation

Add two tunable parameters at the top of `test_position_fusion.py`:

```python
STALL_THRESHOLD_MM = 5.0   # min odometry advance per FSM tick before stall declared
STALL_TICKS        = 5     # consecutive slow ticks required to confirm stall
```

Add state variables before the drive loop:

```python
prev_odom_x = odom_x0
prev_odom_y = odom_y0
stall_count  = 0
```

Inside the loop, after reading `odom_x, odom_y` and before the velocity
command, insert:

```python
# ── Stall detection ───────────────────────────────────────────────────
tick_delta = math.hypot(odom_x - prev_odom_x, odom_y - prev_odom_y)
if tick_delta < STALL_THRESHOLD_MM:
    stall_count += 1
    if stall_count >= STALL_TICKS:
        print(
            f"[pos_fusion_test] STALL detected at {dist_traveled:.0f} mm "
            f"(delta {tick_delta:.1f} mm/tick for {stall_count} consecutive "
            f"ticks — robot likely hit a wall). Stopping."
        )
        break
else:
    stall_count = 0
prev_odom_x = odom_x
prev_odom_y = odom_y
```

## Threshold calibration

`STALL_THRESHOLD_MM` should be less than one full tick of expected odometry
progress.  At 100 mm/s and the default `DEFAULT_FSM_HZ` (10 Hz) one tick
yields ~10 mm; a threshold of 5 mm gives a comfortable margin above noise
while still catching a genuine wall stop within half a second (`STALL_TICKS`
× period = 5 × 0.1 s = 0.5 s).

Lower `STALL_THRESHOLD_MM` if false positives occur on slow surfaces; raise
`STALL_TICKS` if a single slow tick (e.g. carpet grip) causes false trips.

## Notes

- The stall check should be placed **after** the stopping-condition checks so a
  legitimate distance-based stop always wins.
- Reset `stall_count = 0` whenever `gps_aligned` flips True, since the one
  skipped tick (`continue`) would otherwise count as a slow tick.
- Consider also stopping the velocity command (`robot.stop()`) immediately on
  stall rather than letting the caller's `robot.stop()` handle it.
