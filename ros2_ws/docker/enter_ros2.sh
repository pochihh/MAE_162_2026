#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  ros2_ws/docker/enter_ros2.sh [rpi|vm|jetson|/path/to/docker-compose.yml]

Behavior:
  - If COMPOSE is set, it takes precedence.
  - Otherwise:
      rpi    -> ros2_ws/docker/docker-compose.rpi.yml    (service: ros2_runtime)
      vm     -> ros2_ws/docker/docker-compose.vm.yml     (service: ros2_runtime)
      jetson -> ros2_ws/docker/docker-compose.jetson.yml (service: global_gps)
  - Default is rpi.
  - If /ros2_ws/install/setup.bash is missing inside the container, the script
    runs a first-time colcon build before entering the shell.

Examples:
  ./ros2_ws/docker/enter_ros2.sh
  ./ros2_ws/docker/enter_ros2.sh rpi
  ./ros2_ws/docker/enter_ros2.sh jetson
  COMPOSE=ros2_ws/docker/docker-compose.rpi.yml ./ros2_ws/docker/enter_ros2.sh
EOF
}

target="${1:-rpi}"

if [[ "${target}" == "-h" || "${target}" == "--help" ]]; then
    usage
    exit 0
fi

service="ros2_runtime"

if [[ -n "${COMPOSE:-}" ]]; then
    compose_file="${COMPOSE}"
else
    case "${target}" in
        rpi)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.rpi.yml"
            ;;
        vm)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.vm.yml"
            ;;
        jetson)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.jetson.yml"
            service="global_gps"
            ;;
        *.yml|*.yaml)
            compose_file="${target}"
            ;;
        *)
            echo "Unknown target: ${target}" >&2
            usage >&2
            exit 1
            ;;
    esac
fi

exec docker compose -f "${compose_file}" exec "${service}" bash -lc '
source /opt/ros/jazzy/setup.bash
cd /ros2_ws

if [[ ! -f /ros2_ws/install/setup.bash ]]; then
    echo "[enter_ros2] /ros2_ws/install/setup.bash not found; running first-time build..."
    colcon build \
        --symlink-install \
        --cmake-args -DBUILD_TESTING=OFF
fi

source /ros2_ws/install/setup.bash
exec bash -i
'
