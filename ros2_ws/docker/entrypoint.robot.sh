#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.robot.sh — Container entrypoint for the ROS2 robot workspace
#
# Builds the ROS2 workspace on startup, then leaves the container idle so users
# can start nodes manually with `docker compose exec`.
# Build artifacts are cached in named Docker volumes (build/ and install/),
# so only the first startup is slow.
# ─────────────────────────────────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash

echo "[entrypoint] Building ROS2 packages (cached after first run)..."
colcon build \
    --symlink-install \
    --packages-select robot sensors bridge bridge_interfaces \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
