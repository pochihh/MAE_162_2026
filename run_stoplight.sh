#!/usr/bin/env bash
# Start the stoplight_detect script cleanly inside the ROS2 container.
set -e

CONTAINER="docker-ros2_runtime-1"
ROS_SETUP="source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash"

_exec() { docker exec "$CONTAINER" bash -c "$*"; }

_cleanup() {
    echo ""
    echo "[stoplight] Ctrl+C — stopping..."
    _exec "kill -2 \$(ps aux | grep '[s]toplight_detect' | awk '{print \$2}') 2>/dev/null; true"
    sleep 1
    _exec "kill -9 \$(ps aux | grep '[s]toplight_detect' | awk '{print \$2}') 2>/dev/null; true"
    echo "[stoplight] stopped."
    exit 0
}
trap _cleanup INT TERM

echo "=== run_stoplight.sh ==="

# ── 1. Kill stale processes ───────────────────────────────────────────────────
echo "[1/5] killing stale processes..."
_exec "kill -9 \$(ps aux | grep '[s]toplight_detect' | awk '{print \$2}') 2>/dev/null; true"
_exec "kill -9 \$(ps aux | grep '[v]ision_node'      | awk '{print \$2}') 2>/dev/null; true"
sleep 1

# ── 2. Verify camera device is present ───────────────────────────────────────
echo "[2/5] checking camera (/dev/video10)..."
if ! _exec "test -e /dev/video10"; then
    echo "[2/5] ERROR: /dev/video10 not found — is the camera plugged in?"
    exit 1
fi
echo "[2/5] /dev/video10 found — OK"

# ── 3. Build robot + vision (build first so the next start picks up new code) ─
echo "[3/5] building robot + vision packages..."
_exec "cd /ros2_ws && source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select robot vision --symlink-install 2>&1 | tail -5"

# ── 4. Start vision_node (now runs the freshly built version) ─────────────────
echo "[4/5] starting vision_node..."
_exec "$ROS_SETUP && nohup ros2 run vision vision_node \
    --ros-args -p confidence_threshold:=0.10 > /tmp/vision.log 2>&1 &"
sleep 2
echo "[4/5] vision_node started"

# ── 5. Launch stoplight_detect ────────────────────────────────────────────────
echo "[5/5] starting stoplight_detect  (stream → http://<pi-ip>:8083)"
_exec "$ROS_SETUP && ros2 run robot stoplight_detect"
