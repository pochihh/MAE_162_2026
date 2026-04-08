# ─────────────────────────────────────────────────────────────────────────────
# Dockerfile.robot — ROS2 robot stack runtime image
#
# Target: Raspberry Pi 5 running Ubuntu 25.10 server (arm64)
# ROS2:   Jazzy Jalisco (multi-arch image includes linux/arm64)
#
# Build context: Project-NUEVO/ repository root
#   docker compose -f ros2_ws/docker/docker-compose.rpi.yml build
#
# The ROS workspace source and the shared nuevo_bridge backend are
# volume-mounted at runtime. The image only installs the core ROS2
# and bridge dependencies needed for robot control. No source code is
# baked in.
# colcon build runs in the entrypoint and its artifacts are cached in a
# named Docker volume, so only the first startup is slow (~60s).
# ─────────────────────────────────────────────────────────────────────────────

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# ── Core system packages ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── Python runtime deps for the shared bridge runtime ────────────────────────
RUN pip3 install --no-cache-dir --break-system-packages \
    "fastapi>=0.109.0,<1.0" \
    "uvicorn[standard]>=0.27.0,<1.0" \
    "pyserial>=3.5,<4.0" \
    "websockets>=12.0,<13.0" \
    "PyJWT>=2.8.0,<3.0" \
    "passlib[bcrypt]>=1.7.4"

# ── Initialize rosdep ─────────────────────────────────────────────────────────
RUN rosdep init || true && rosdep update

WORKDIR /ros2_ws
RUN mkdir -p src

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY ros2_ws/docker/entrypoint.robot.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-lc", "echo '[entrypoint] Container ready. Start ROS nodes manually with ros2 run ...'; exec sleep infinity"]
