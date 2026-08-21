# karaburan

Load ROS 2:
`source /opt/ros/jazzy/setup.bash`

Navigate to ROS workspace:
`cd ros_ws`

# Temperature sensor
Build and run temperature sensor node (my SensorID):
`colcon build && source ./install/setup.bash && ros2 run tempreader tempreaderNode --ros-args -p sensorId:=28.C23646D48524 &`

Launch GPSD client node:
`ros2 launch gpsd_client gpsd_client-launch.py &`

Install ros-jazzy-gpsd-client

Read fixes from GPS:
`ros2 topic echo /fix`

Read temperature:
`ros2 topic echo /temperature`

The boat has three DS18B20 sensors. When temperature measurement is enabled,
their readings are published as follows:

| Sensor ID | Topic |
| --- | --- |
| `28.C23646D48524` | `/temperature` |
| `28.5CD456B5013C` | `/temperature/sensor_2` |
| `28.F95856B5013C` | `/temperature/sensor_3` |

# Boat total setup launch file

This currently works on My Machine TM (use the generic instructions at the top first):
`colcon build && source install/setup.bash && ros2 launch karaburan_bringup boat.launch.py`

Clone the repository with its MPU9250 driver reference, or initialise it after
an existing clone:

```bash
git submodule update --init --recursive
```

The physical boat enables LiDAR by default. Disable it explicitly with
`with_lidar:=false` when the sensor is disconnected. The GPS status workaround
also remains enabled by default; use `fix_status_override_enabled:=false` to
relay `/fix` to `/fix/valid` without changing the receiver-provided status.

The motor controller uses `/dev/ttyS0` at 115200 baud by default. Override the
serial settings when needed:

```bash
ros2 launch karaburan_bringup boat.launch.py \
  motor_serial_port:=/dev/ttyS0 \
  motor_baud_rate:=115200
```

GPSD must expose only the ZED-F9P on `/dev/ttyUSB1`; the older u-blox receiver
must remain disabled for ROS navigation. Configure `/etc/default/gpsd` as:

```ini
DEVICES="/dev/ttyUSB1"
GPSD_OPTIONS="-n"
USBAUTO="false"
```

Apply the GPSD configuration with:

```bash
sudo systemctl restart gpsd.socket gpsd.service
```

Start the laptop simulation from its dedicated package:

```bash
ros2 launch karaburan_simulation sim.launch.py
```

The deterministic Ravensberg navigation-test world has its own launch file:

```bash
ros2 launch karaburan_simulation ravensberg.launch.py
```

Generate its metadata and debug map without Gazebo with:

```bash
ros2 run karaburan_simulation generate_ravensberg_scenario \
  --debug-only --output-dir /tmp/ravensberg-mvp
```

See [`docs/ravensberg_world.md`](docs/ravensberg_world.md) for the geometry
source, coordinate system, generated artifacts, and known limitations.

Use `headless:=true with_rviz:=false` when no GUI is required.

## GPS shuttle route

After the boat launch is running and `/fix/valid` has a valid GPS fix, start
the predefined shuttle route in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/karaburan/ros_ws
source install/setup.bash
ros2 run navigation gps_shuttle
```

The boat continuously alternates between these WGS84 positions:

1. `52.018599, 4.708720`
2. `52.018962, 4.708961`

Press `Ctrl+C` to cancel the active route. The command waits if Nav2 has not
finished starting yet. It requires `navsat_transform_node` to have received a
valid GPS fix before the waypoints can be converted to the local map frame.

An additional four-point route is available with:

```bash
ros2 run navigation gps_four_point_route
```

It continuously visits these WGS84 positions in order and then returns to the
first position:

1. `52.042296, 4.747545`
2. `52.042648, 4.748077`
3. `52.042143, 4.748323`
4. `52.041718, 4.747608`

## OpenStreetMap simulation view

The Leaflet map displays the simulated GPS position, sailed track, and current
Nav2 plan over an OpenStreetMap base layer. Start it alongside an already
running simulator:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/karaburan/ros_ws
source install/setup.bash
ros2 launch karaburan_ui leaflet_map.launch.py
```

Alternatively, let the simulator start the map after ten seconds:

```bash
ros2 launch karaburan_simulation sim.launch.py with_map:=true
```

The map opens locally and is also available at `http://<sim-host>:8088`. Its
configured WGS84 origin matches
`karaburan_simulation/worlds/world.sdf`. Map tiles are visual context only:
navigation continues to use ROS topics and local `map`/`odom` frames if the
internet connection or tile service is unavailable. The default
OpenStreetMap tile service is intended only for normal interactive viewing; do
not use it for bulk download or offline prefetching.

## Manual control via partial setup

The current setup does not provide `/cmd_vel` commands other than 'stop'.
So you can try to do that yourself:

```
# Forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
# Turn left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
# Stop
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Useful checks:
ros2 topic info /cmd_vel
ros2 interface show geometry_msgs/msg/Twist
ros2 topic echo /cmd_vel
ros2 topic list | grep cmd_vel

# Keyboard-control:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

## Installation of ROS2 packages

Currently the following packages have been installed (these will introduce many dependencies, list generated with `apt-mark showmanual > ~/manual-packages.txt`:
ros-dev-tools
ros-jazzy-gps-tools
ros-jazzy-gpsd-client
ros-jazzy-mapviz-interfaces
ros-jazzy-nav2-bringup
ros-jazzy-nav2-waypoint-follower
ros-jazzy-navigation2
ros-jazzy-robot-localization
ros-jazzy-ros-base
ros-jazzy-ros2-control
ros-jazzy-ros2-controllers
ros-jazzy-tf-transformations
ros-jazzy-ros-gz
ros-jazzy-rviz2
ros2-apt-source
## MCAP recording

Install the MCAP storage plugin alongside ROS 2 Jazzy on the boat:

```bash
sudo apt install ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap
sudo mkdir -p /data/karaburan/bags
sudo chown "$USER":"$USER" /data/karaburan/bags
```

Recording is disabled by default. Start the physical boat with the standard
`navigation` profile without LiDAR:

```bash
ros2 launch karaburan_bringup boat.launch.py \
  record_enabled:=true \
  record_profile:=navigation \
  record_include_scan:=false \
  record_output_dir:=/data/karaburan/bags
```

For the simulator:

```bash
ros2 launch karaburan_simulation sim.launch.py \
  record_enabled:=true \
  record_profile:=navigation \
  record_include_scan:=false \
  record_output_dir:=./bags
```

Available profiles:

| Profile | Topics |
|---|---|
| `minimal` | GPS, temperature, sonar, VL53L0X time-of-flight (ToF), and BT785 |
| `navigation` | `minimal` plus IMU, odometry, commands, TF, and TF-static |
| `full` | `navigation` plus `/scan` |

`record_include_scan:=true` also adds `/scan` to `minimal` or `navigation`.
Use `record_extra_topics` to add comma-separated topics:

```bash
ros2 launch karaburan_bringup boat.launch.py \
  record_enabled:=true \
  record_profile:=navigation \
  record_extra_topics:="/diagnostics,/battery_state"
```

Additional recording arguments:

| Argument | Default | Meaning |
|---|---:|---|
| `record_enabled` | `false` | Enable or disable MCAP recording |
| `record_profile` | `navigation` | `minimal`, `navigation`, or `full` |
| `record_include_scan` | `false` | Explicitly include LiDAR |
| `record_output_dir` | boat: `/data/karaburan/bags`; sim: `./bags` | Recording root directory |
| `record_max_bag_duration` | `900` | Start a new segment after 15 minutes; `0` disables duration-based rotation |
| `record_max_bag_size` | `2147483648` | Start a new segment after 2 GiB; `0` disables size-based rotation |
| `record_start_delay` | `5.0` | Delay recording to allow topic discovery |

The recorder uses MCAP with the `zstd_fast` profile. Each launch creates a
UTC-dated directory. Inspect a recording with:

```bash
ros2 bag info /data/karaburan/bags/<recording-directory>
```

### Storage estimate

The estimate uses the agreed frequencies for the physical boat:

| Data stream | Frequency |
|---|---:|
| GPS | 1 Hz |
| IMU | 30 Hz |
| Sonar | 1 Hz |
| Temperature | 0.2 Hz |
| Physical LiDAR | approximately 5.8 Hz |
| VL53L0X ToF sensor | 20 Hz |
| BT785 | 0.1 Hz |
| Filtered odometry | 40 Hz |

Assumptions: one sailing day contains eight recording hours and local retention
covers two sailing days, or 16 recording hours. The calculation uses estimated
serialized ROS message sizes plus a 25% allowance for MCAP indexes, metadata,
variation, and filesystem overhead. It conservatively assumes no reduction from
Zstd compression.

| Recording | Planning rate per hour | 8 hours | 2 sailing days / 16 hours |
|---|---:|---:|---:|
| `navigation`, excluding `/scan` | approximately 0.23 GB | approximately 1.8 GB | approximately 3.6 GB; reserve 4 GB |
| Physical `/scan` only, additional | approximately 0.13 GB | approximately 1.0 GB | approximately 2.0 GB |
| `navigation` including `/scan` | approximately 0.36 GB | approximately 2.8 GB | approximately 5.6 GB; reserve 6 GB |

The physical `LaserScan` contains 576 ranges and 576 intensities per scan.
Consequently, `/scan` adds approximately 2 GB over two sailing days, despite its
5.8 Hz rate. Actual compression depends heavily on the environment and intensity
values. Measure the first real voyage with `du -sh` and adjust this estimate.

If two days later means **48 hours of continuous recording**, reserve
approximately 12 GB without `/scan` or 18 GB with `/scan`. In practice, keep at
least 16 GB free without LiDAR and 32 GB when recording LiDAR. This allows space
for segments awaiting upload and prevents the system disk from filling up.

### Optional measurement instruments

The physical LiDAR driver is enabled by default for `boat.launch.py`; the other
measurement drivers remain disabled. Configure the instruments connected to
the boat as needed:

```bash
ros2 launch karaburan_bringup boat.launch.py \
  with_temperature:=true \
  temperature_sensor_id:=28.C23646D48524 \
  temperature_sensor_id_2:=28.5CD456B5013C \
  temperature_sensor_id_3:=28.F95856B5013C \
  with_sonar:=true \
  sonar_device:=D3:01:01:02:2F:C6 \
  with_lidar:=true \
  lidar_device:=/dev/ttyUSB0 \
  with_tof:=true \
  tof_rate_hz:=20.0 \
  with_bt785:=true \
  bt785_device:=AA:BB:CC:DD:EE:FF
```

The same arguments are available on `sim.launch.py`, but enabling them starts
the physical hardware drivers. This is intended for hardware-in-the-loop tests.
They remain disabled by default because the simulator already publishes
`/scan`, `/imu/data`, and `/fix/valid`.

### Testing with Rancher Desktop and Docker

The launch files and MCAP recording can be tested on this laptop without a
local ROS installation. Start Rancher Desktop, select a Docker-compatible
container engine, and run:

```bash
docker context use default
docker build -f docker/Dockerfile.ros-jazzy-test -t karaburan-ros-test .
docker run --rm karaburan-ros-test
```

The container builds the ROS workspace and performs a smoke test that:

1. starts the storage launch file with a short rotation interval;
2. publishes sample `sensor_msgs/NavSatFix` messages on `/fix`;
3. verifies that multiple MCAP segments are created;
4. inspects the recording with `ros2 bag info`;
5. verifies that the `/fix` topic and message count are present;
6. checks that disabled storage creates no files; and
7. imports all launch files to catch Python syntax and dependency errors.

This test does not emulate I2C, serial, Bluetooth, or 1-Wire hardware. Testing
the physical sensor drivers still requires the boat or explicit device
passthrough into the container.
