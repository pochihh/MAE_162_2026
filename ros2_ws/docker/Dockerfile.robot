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
# libcamera Python 0.5.0 bindings are in /usr/local/lib (built from source).
# picamera2 and any libcamera Python code should find them here.
ENV PYTHONPATH=/usr/local/lib/aarch64-linux-gnu/python3.12/site-packages:/usr/lib/python3/dist-packages:${PYTHONPATH}
# IPA modules for the Pi5 PISP pipeline (built from source alongside libcamera).
ENV LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera

# ── Core system packages ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3-matplotlib

# ── Python runtime deps for the shared bridge runtime ────────────────────────
RUN pip3 install --no-cache-dir --break-system-packages \
    "fastapi>=0.109.0,<1.0" \
    "uvicorn[standard]>=0.27.0,<1.0" \
    "pyserial>=3.5,<4.0" \
    "websockets>=12.0,<13.0" \
    "PyJWT>=2.8.0,<3.0" \
    "passlib[bcrypt]>=1.7.4"

# ── Pi Camera — build libcamera 0.5.0 from source ────────────────────────────
#
# WHY BUILD FROM SOURCE:
#   Ubuntu Noble (24.04) ros:jazzy-ros-base only ships ros-jazzy-libcamera 0.7.0.
#   That binary was compiled for an older kernel where CFE entity names used
#   hyphens (rp1-cfe-fe-image0). Pi 5 kernel 6.12+ renames them with
#   underscores (rp1-cfe-fe_image0). The binary cannot be patched; we must
#   build a corrected libcamera from source.
#
#   Host Ubuntu 25.10 ships a Canonical-patched libcamera 0.5.0 that handles
#   both naming conventions. We replicate those patches below.
#
# BUILD ORDER: libpisp → libcamera → rpicam-apps

# Build deps (shared across all three)
RUN apt-get update && apt-get install -y --no-install-recommends \
        git cmake meson ninja-build pkg-config \
        python3-yaml python3-ply python3-jinja2 \
        libgnutls28-dev openssl \
        libudev-dev libyaml-dev \
        libboost-dev libboost-program-options-dev libboost-filesystem-dev \
        libevent-dev \
        libjpeg-dev libtiff-dev libpng-dev libexif-dev \
        python3-pybind11 \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# 1. Build libpisp v1.3.0 (Pi5 ISP library — required by libcamera pisp pipeline)
RUN git clone --depth 1 --branch v1.3.0 \
        https://github.com/raspberrypi/libpisp.git /tmp/libpisp \
    && meson setup /tmp/libpisp/build /tmp/libpisp \
        --buildtype=release \
    && ninja -C /tmp/libpisp/build -j$(nproc) \
    && ninja -C /tmp/libpisp/build install \
    && rm -rf /tmp/libpisp \
    && ldconfig

# 2. Build libcamera v0.5.0 from source with Pi5 kernel 6.12+ entity name patches
#
# Patches applied to src/libcamera/pipeline/rpi/pisp/pisp.cpp:
#   a) In match(): DeviceMatch entity names changed from hyphen to underscore
#      to match kernel 6.12+ rp1-cfe driver (rp1-cfe-fe_image0 etc.)
#   b) In platformRegister(): getEntityByName() calls patched the same way;
#      also "rp1-cfe-csi2-ch1" renamed to "rp1-cfe-embedded" (kernel 6.12+).
RUN git clone --depth 1 --branch v0.5.0 \
        https://github.com/raspberrypi/libcamera.git /tmp/libcamera \
    && sed -i \
        -e 's/cfe.add("rp1-cfe-fe-image0")/cfe.add("rp1-cfe-fe_image0")/g' \
        -e 's/cfe.add("rp1-cfe-fe-stats")/cfe.add("rp1-cfe-fe_stats")/g' \
        -e 's/cfe.add("rp1-cfe-fe-config")/cfe.add("rp1-cfe-fe_config")/g' \
        -e 's/cfe->getEntityByName("rp1-cfe-fe-image0")/cfe->getEntityByName("rp1-cfe-fe_image0")/g' \
        -e 's/cfe->getEntityByName("rp1-cfe-csi2-ch1")/cfe->getEntityByName("rp1-cfe-embedded")/g' \
        -e 's/cfe->getEntityByName("rp1-cfe-fe-stats")/cfe->getEntityByName("rp1-cfe-fe_stats")/g' \
        -e 's/cfe->getEntityByName("rp1-cfe-fe-config")/cfe->getEntityByName("rp1-cfe-fe_config")/g' \
        /tmp/libcamera/src/libcamera/pipeline/rpi/pisp/pisp.cpp \
    && meson setup /tmp/libcamera/build /tmp/libcamera \
        --buildtype=release \
        -Dpipelines=rpi/pisp,rpi/vc4 \
        -Dipas=rpi/pisp,rpi/vc4 \
        -Dtest=false \
        -Dlc-compliance=disabled \
        -Dcam=disabled \
        -Dqcam=disabled \
    && ninja -C /tmp/libcamera/build -j$(nproc) \
    && ninja -C /tmp/libcamera/build install \
    && rm -rf /tmp/libcamera \
    && ldconfig

# 3. Build rpicam-apps v1.9.1 against our patched libcamera 0.5.0
#
# Compatibility patches for libcamera 0.5.0 vs rpicam-apps v1.9.1 API delta:
#   - controls::FrameWallClock not yet in 0.5.0 → use buffer timestamp directly
#   - controls::rpi::Sync* not yet in 0.5.0 → disabled via DISABLE_RPI_FEATURES
#   - enable_libav=disabled: Ubuntu Noble libavcodec too old for rpicam-apps 1.9.x
RUN git clone --depth 1 --branch v1.9.1 \
        https://github.com/raspberrypi/rpicam-apps.git /tmp/rpicam-apps \
    && sed -i \
        -e 's/auto ts = completed_request->metadata.get(controls::FrameWallClock);//' \
        -e 's/int64_t timestamp_ns = ts ? \*ts : buffer->metadata().timestamp;/int64_t timestamp_ns = buffer->metadata().timestamp;/' \
        /tmp/rpicam-apps/core/rpicam_encoder.hpp \
    && PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig \
       meson setup /tmp/rpicam-apps/build /tmp/rpicam-apps \
        --buildtype=release \
        -Denable_drm=disabled \
        -Denable_egl=disabled \
        -Denable_qt=disabled \
        -Denable_opencv=disabled \
        -Denable_tflite=disabled \
        -Denable_libav=disabled \
        -Dcpp_args="-DDISABLE_RPI_FEATURES" \
    && ninja -C /tmp/rpicam-apps/build -j$(nproc) \
    && ninja -C /tmp/rpicam-apps/build install \
    && rm -rf /tmp/rpicam-apps \
    && ldconfig

# picamera2 for Python camera access (libcamera Python bindings provided above)
# libopenexr-dev is required to build the OpenEXR wheel which picamera2 depends on.
# pykms stub: headless containers have no KMS/DRM display; stub prevents ImportError.
RUN apt-get update && apt-get install -y --no-install-recommends libopenexr-dev \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages picamera2 \
    && echo "# headless stub" > /usr/local/lib/python3.12/dist-packages/pykms.py

# ── Initialize rosdep ─────────────────────────────────────────────────────────
RUN rosdep init || true && rosdep update

WORKDIR /ros2_ws
RUN mkdir -p src

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY ros2_ws/docker/entrypoint.robot.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-lc", "echo '[entrypoint] Container ready. Start ROS nodes manually with ros2 run ...'; exec sleep infinity"]
