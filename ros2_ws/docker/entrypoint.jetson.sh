#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.jetson.sh — Container entrypoint for the Jetson GPS stack
#
# Builds only the packages needed on the Jetson (global_gps + its interfaces)
# and leaves the container idle so the launch file can be started manually.
# Build artifacts are cached in named Docker volumes.
# ─────────────────────────────────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash

echo "[entrypoint] Building ROS2 packages (cached after first run)..."
colcon build \
    --symlink-install \
    --packages-select bridge_interfaces global_gps \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
