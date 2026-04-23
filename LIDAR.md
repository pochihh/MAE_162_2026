# Outside Docker
1. Pull the latest repo
Inside your project root directory (Project-NUEVO)
```git pull```

2. Connect your lidar and Check if the lidar device is present
```sudo chmod 777 /dev/ttyUSB0```

3. Stop any running Docker containers
```docker compose -f $COMPOSE down```

4. Rebuild the container (one-time)
```docker compose -f $COMPOSE build```

5. Restart Docker and enter docker terminal
```bash
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
docker compose -f $COMPOSE exec ros2_runtime bash
```

# Inside Docker
1. Source ROS2 environment
```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

2. Start ROS2 node of the Lidar
```bash
ros2 run rplidar_ros rplidar_node --ros-args   -p channel_type:="serial"   -p serial_port:="/dev/rplidar"   -p serial_baudrate:=460800   -p frame_id:="laser_frame"   -p angle_compensate:=true   -p scan_mode:="Standard"
```

# Open Docker in the Second terminal
1. Source ROS2 environment
```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

2. Start ROS2 sensor node
```ros2 run sensors sensor_node```

# In the Third terminal
1. Copy the lidar result Inside your project root directory (Project-NUEVO)
```docker cp docker-ros2_runtime-1:/data/plot.png .```
