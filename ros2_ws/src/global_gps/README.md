# Global GPS — Field Localizer

Runs on the **Jetson Nano** (not on the robot RPis).

Uses a RealSense D4xx camera mounted with a top-down view of the field to
detect ArUco markers and publish 2-D world-frame positions for all rover robots
simultaneously.

## How it works

1. **Calibration**: On startup, the node waits until all four corner anchor
   markers (IDs 0–3 by default) are visible at once. It fits a ground plane
   through their 3-D positions and builds a world coordinate frame. Calibration
   is automatic — just make sure the camera can see all four corners when the
   node starts.

2. **Tracking**: Once calibrated, the node detects any rover markers (IDs 11–18
   by default) in every frame and publishes their 2-D poses (x, y, theta) on
   `/global_gps/tag_detections` across the network.

3. **Robot side**: Each robot runs a `robot_gps` node (in the `sensors` package)
   that subscribes to the Jetson's global topic and re-publishes detections
   locally so the robot's main node can use them.

## Marker setup

| Role | IDs | Size |
|---|---|---|
| Field corner anchors | 0, 1, 2, 3 | configurable (`marker_size` param) |
| Rover markers | 11–18 | same size |

Place corner markers at the four corners of the field with the **same physical
size** as the rover markers. The world frame origin is placed at marker 0, with
the X axis pointing toward marker 1 and the Y axis toward marker 2.

## ROS topic

| Topic | Type | Publisher |
|---|---|---|
| `/global_gps/tag_detections` | `bridge_interfaces/TagDetectionArray` | Jetson |
| `/tag_detections` | `bridge_interfaces/TagDetectionArray` | `robot_gps` (RPi) |

Each `TagDetection` in the array contains:

```
int32   tag_id   # ArUco marker ID
float32 x        # world-frame X (metres)
float32 y        # world-frame Y (metres)
float32 theta    # heading (radians, CCW from world X)
```

---

## Jetson setup

### Prerequisites (one-time, on the Jetson host)

1. **JetPack 6.x** (Ubuntu 22.04) installed and working.
2. **Docker** installed:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   # JetPack may not create the docker group automatically — create it if missing:
   sudo groupadd docker 2>/dev/null || true
   sudo usermod -aG docker $USER
   newgrp docker
   docker --version
   ```
3. Clone your fork of the repository:
   ```bash
   git clone https://github.com/<your-username>/Project-NUEVO.git
   cd Project-NUEVO
   ```

### Build and start the container

```bash
COMPOSE=ros2_ws/docker/docker-compose.jetson.yml

docker compose -f $COMPOSE build       # first time only (~5 min)
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f global_gps   # watch colcon build
```

Wait until the log shows `[entrypoint] Container ready`.

### Launch the localizer

```bash
docker compose -f $COMPOSE exec global_gps bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch global_gps global_gps.launch.py
```

To override default parameters:

```bash
ros2 launch global_gps global_gps.launch.py \
    marker_size:=0.15 \
    corner_ids:=[0,1,2,3] \
    rover_ids:=[11,12,13]
```

### Capture a photo of the camera's field of view

A standalone script is included that saves a single JPEG from the RealSense
camera without needing the full ROS2 stack running.  It uses `pyrealsense2`
directly via the libusb/V4L2 backend and automatically repairs missing device
nodes inside the container after a USB re-enumeration.

**One-time setup** — install `pyrealsense2` inside the container:
```bash
docker compose -f $COMPOSE exec global_gps \
    pip3 install --break-system-packages pyrealsense2
```

**Capture a photo** (saves to `/tmp/gps_snapshot_<timestamp>.jpg` by default):
```bash
# From the Jetson host:
docker exec docker-global_gps-1 \
    python3 /ros2_ws/src/global_gps/capture_photo.py

# Or from inside the container shell:
python3 /ros2_ws/src/global_gps/capture_photo.py

# Custom output path:
python3 /ros2_ws/src/global_gps/capture_photo.py --output /tmp/field.jpg
```

The script waits for 15 warm-up frames (~0.5 s) so auto-exposure settles
before saving.  Additional options:

```
--output / -o   Output file path
--warmup        Warm-up frame count (default: 15)
--width         Stream width  (default: 640)
--height        Stream height (default: 480)
--fps           Stream FPS    (default: 30)
```

> **Note:** The GPS launch (`ros2 launch global_gps global_gps.launch.py`)
> must **not** be running when you capture a photo — both the launch and the
> script compete for exclusive camera access.  Stop the launch first, capture,
> then restart it if needed.

### Verify detections from any machine on the network

```bash
# Quick test: connect to the Jetson TCP server directly (no ROS needed)
# nc -v 192.168.8.120 7777   # should show JSON lines when markers are visible

# Or from inside the robot container:
ros2 topic echo /tag_detections
```

---

## Robot-side setup (RPi per robot)

The `robot_gps` node in the `sensors` package bridges the Jetson's global
topic into the robot's local domain.

### Start robot_gps

Open a terminal inside the robot's container and run it separately from
the bridge/robot nodes:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml

docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run sensors robot_gps
```

No special environment variables are needed.  The node connects to the
Jetson's TCP server directly, which works through the lab WiFi NAT.

Once running, the topic `/tag_detections` becomes available locally and other
nodes (robot node, etc.) can subscribe to it with `ROS_LOCALHOST_ONLY=1` as
normal.

### Subscribe in the robot node

```python
from bridge_interfaces.msg import TagDetectionArray

self.create_subscription(
    TagDetectionArray,
    "/tag_detections",
    self._on_gps_update,
    10,
)

def _on_gps_update(self, msg: TagDetectionArray) -> None:
    for det in msg.detections:
        print(f"Robot {det.tag_id}: ({det.x:.2f}, {det.y:.2f})")
```

---

## Troubleshooting

**No detections published / calibration never completes**
- Make sure all four corner marker IDs (default 0–3) are visible in the camera
  frame simultaneously when the node starts.
- Check that `corner_ids` and `rover_ids` parameters match the printed markers.

**RealSense camera not found**
- Confirm the camera is connected: `rs-enumerate-devices` (inside the container).
- The container runs with `privileged: true` which is required for the
  librealsense2 USB driver. If the camera still isn't found, try
  `docker compose down` and `up -d` with the camera already plugged in.

**Robot cannot see `/tag_detections`**
- Confirm `robot_gps` is running with `ROS_LOCALHOST_ONLY=0`.
- Confirm the Jetson and the RPi are on the same network subnet.
- Confirm `ROS_DOMAIN_ID` is the same on Jetson and RPi (default: 0).
- From the RPi (with `ROS_LOCALHOST_ONLY=0`): `ros2 topic list` should show
  `/global_gps/tag_detections`.

**Multiple `/bridge` nodes visible**
- See `ros2_ws/TROUBLE_SHOOT.md`.
