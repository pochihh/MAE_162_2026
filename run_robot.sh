#!/usr/bin/env bash
# Ensure all required ROS2 nodes are running cleanly, then start the robot FSM.
set -e

CONTAINER="docker-ros2_runtime-1"
ROS_SETUP="source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash"

_exec() { docker exec "$CONTAINER" bash -c "$*"; }

_cleanup() {
    echo ""
    echo "[run_robot] Ctrl+C — stopping robot..."

    # SIGINT first — lets the Python finally block call robot.stop()
    docker exec "$CONTAINER" bash -c \
        "kill -2 \$(ps aux | grep '[r]os2_ws/install/robot/lib/robot/robot' | awk '{print \$2}') 2>/dev/null; true"

    sleep 2

    # Force-kill anything still alive, then zero the motors directly
    docker exec "$CONTAINER" bash -c \
        "kill -9 \$(ps aux | grep '[r]os2_ws/install/robot/lib/robot/robot' | awk '{print \$2}') 2>/dev/null; true"
    docker exec "$CONTAINER" bash -c \
        "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && \
         ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
         '{motor_number: 1, target_ticks: 0}' > /dev/null 2>&1 ; \
         ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
         '{motor_number: 2, target_ticks: 0}' > /dev/null 2>&1; true"

    echo "[run_robot] robot stopped."
    exit 0
}
trap _cleanup INT TERM

echo "=== run_robot.sh ==="

# ── 1. Kill any existing robot FSM nodes ─────────────────────────────────────
echo "[1/6] killing stale robot nodes..."
_exec "kill -9 \$(ps aux | grep '[r]os2_ws/install/robot/lib/robot/robot' | awk '{print \$2}') 2>/dev/null; true"

# ── 2. Ensure rplidar_c1_node is running ─────────────────────────────────────
echo "[2/6] checking rplidar..."
LIDAR_RUNNING=$(_exec "ps aux | grep '[r]plidar_c1_node' | grep -v grep | wc -l")
if [ "$LIDAR_RUNNING" -eq 0 ]; then
    echo "[2/6] starting rplidar_c1_node..."
    _exec "$ROS_SETUP && nohup ros2 run rplidar_ros rplidar_c1_node > /tmp/rplidar.log 2>&1 &"
    sleep 1
else
    echo "[2/6] rplidar already running — OK"
fi

# ── 3. Ensure vision_node is running ─────────────────────────────────────────
echo "[3/6] checking vision_node..."
VISION_RUNNING=$(_exec "ps aux | grep '[v]ision_node' | grep -v grep | wc -l")
if [ "$VISION_RUNNING" -eq 0 ]; then
    echo "[3/6] starting vision_node..."
    _exec "$ROS_SETUP && nohup ros2 run vision vision_node \
        --ros-args -p confidence_threshold:=0.10 > /tmp/vision.log 2>&1 &"
    sleep 1
else
    echo "[3/6] vision_node already running — OK"
fi

# ── 4. Ensure robot_gps (ArUco GPS → /tag_detections) is running ─────────────
# Connects to Jetson at 192.168.8.120:7777 over TCP.
# If Jetson is unreachable the node retries automatically — safe to start always.
echo "[4/6] checking robot_gps..."
GPS_RUNNING=$(_exec "ps aux | grep '[r]obot_gps' | grep -v grep | wc -l")
if [ "$GPS_RUNNING" -eq 0 ]; then
    echo "[4/6] starting robot_gps..."
    _exec "$ROS_SETUP && nohup ros2 run sensors robot_gps > /tmp/robot_gps.log 2>&1 &"
    sleep 1
else
    echo "[4/6] robot_gps already running — OK"
fi

# ── 5. Reset firmware → IDLE ──────────────────────────────────────────────────
# RESET (cmd=3) transitions any state → IDLE.
# The robot node will set odometry params while IDLE, then start RUNNING itself.
# (SYS_ODOM_PARAM_SET is only accepted in IDLE; sending START here would prevent
#  the robot node from ever setting its motor assignments.)
echo "[5/6] resetting firmware to IDLE..."
_exec "$ROS_SETUP && ros2 topic pub --once /sys_cmd bridge_interfaces/msg/SysCommand \
    '{command: 3}' > /dev/null 2>&1; true"
sleep 1

# ── 6. Start robot FSM ───────────────────────────────────────────────────────
echo "[6/6] starting robot node..."
FSM_ENV=""
[ -n "$ROBOT_FSM_MODULE" ] && FSM_ENV="ROBOT_FSM_MODULE=$ROBOT_FSM_MODULE"
_exec "$ROS_SETUP && $FSM_ENV ros2 run robot robot"
