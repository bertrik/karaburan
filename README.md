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

# Boat total setup launch file

This currently works on My Machine TM (use the generic instructions at the top first):
`colcon build && source install/setup.bash && ros2 launch navigation boat.launch.py`

Replace `boat.launch.py` with `sim.launch.py` for the simulated stuff (laptop only! Requires GUI!)

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


## MCAP-opnamen

Installeer op de boot naast ROS 2 Jazzy ook de MCAP-storageplugin:

```bash
sudo apt install ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap
sudo mkdir -p /data/karaburan/bags
sudo chown "$USER":"$USER" /data/karaburan/bags
```

Opnemen staat standaard uit. Start de echte boot met het standaard
`navigation`-profiel zonder LiDAR:

```bash
ros2 launch navigation boat.launch.py \
  record_enabled:=true \
  record_profile:=navigation \
  record_include_scan:=false \
  record_output_dir:=/data/karaburan/bags
```

Voor de simulator:

```bash
ros2 launch navigation sim.launch.py \
  record_enabled:=true \
  record_profile:=navigation \
  record_include_scan:=false \
  record_output_dir:=./bags
```

Beschikbare profielen:

| Profiel | Topics |
|---|---|
| `minimal` | GPS, temperatuur, sonar, ToF en BT785 |
| `navigation` | `minimal` plus IMU, odometrie, commando, TF en TF-static |
| `full` | `navigation` plus `/scan` |

`record_include_scan:=true` voegt `/scan` ook aan `minimal` of `navigation` toe.
Met `record_extra_topics` kunnen kommagescheiden topics worden toegevoegd:

```bash
ros2 launch navigation boat.launch.py \
  record_enabled:=true \
  record_profile:=navigation \
  record_extra_topics:="/diagnostics,/battery_state"
```

Overige opnameargumenten:

| Argument | Standaard | Betekenis |
|---|---:|---|
| `record_enabled` | `false` | MCAP-recorder aan/uit |
| `record_profile` | `navigation` | `minimal`, `navigation` of `full` |
| `record_include_scan` | `false` | LiDAR expliciet toevoegen |
| `record_output_dir` | boot: `/data/karaburan/bags`; sim: `./bags` | Hoofdmap voor opnamen |
| `record_max_bag_duration` | `900` | Nieuw segment na 15 minuten; `0` schakelt dit uit |
| `record_max_bag_size` | `2147483648` | Nieuw segment na 2 GiB; `0` schakelt dit uit |
| `record_start_delay` | `5.0` | Wachttijd voor topic-discovery |

De recorder gebruikt MCAP met het `zstd_fast`-profiel. Iedere start maakt een
UTC-gedateerde map. Controleer een opname met:

```bash
ros2 bag info /data/karaburan/bags/<opnamemap>
```

### Opslagschatting

De schatting gebruikt de afgestemde frequenties voor de echte boot:

| Datastroom | Frequentie |
|---|---:|
| GPS | 1 Hz |
| IMU | 30 Hz |
| Sonar | 1 Hz |
| Temperatuur | 0,2 Hz |
| Fysieke LiDAR | circa 5,8 Hz |
| ToF/VL53L0X | 20 Hz |
| BT785 | 0,1 Hz |
| Gefilterde odometrie | 40 Hz |

Aannames: één vaardag is 8 opname-uren en lokale retentie is twee vaardagen,
dus 16 opname-uren. De berekening gebruikt de geschatte geserialiseerde
ROS-berichtgrootte plus 25% marge voor MCAP-indexen, metadata, variatie en
filesystemruimte. Er wordt conservatief geen voordeel van Zstd-compressie
ingeboekt.

| Opname | Rekenwaarde per uur | 8 uur | 2 vaardagen / 16 uur |
|---|---:|---:|---:|
| `navigation`, zonder `/scan` | circa 0,23 GB | circa 1,8 GB | circa 3,6 GB; reserveer 4 GB |
| Alleen fysieke `/scan` extra | circa 0,13 GB | circa 1,0 GB | circa 2,0 GB |
| `navigation` met `/scan` | circa 0,36 GB | circa 2,8 GB | circa 5,6 GB; reserveer 6 GB |

De fysieke `LaserScan` bevat 576 afstanden en 576 intensiteiten per scan. Daarom
voegt `/scan` ondanks 5,8 Hz ongeveer 2 GB toe over twee vaardagen. De werkelijke
compressie hangt sterk af van de omgeving en intensiteitswaarden. Meet na de
eerste vaart met `du -sh` en pas de raming daarop aan.

Als twee dagen later **48 uur continu opnemen** betekent, reserveer dan ongeveer
12 GB zonder `/scan` of 18 GB met `/scan`. Praktisch advies: houd minimaal 16 GB
vrij voor het profiel zonder LiDAR en 32 GB wanneer LiDAR wordt opgenomen. Dit
geeft ruimte voor segmenten die nog worden geüpload en voorkomt een volle
systeemschijf.

## Optionele meetinstrumenten

`measurement_instruments.launch.py` bevat de fysieke instrumenten die nog niet in
`boat.launch.py` stonden. Alle instrumenten staan standaard uit en kunnen
onafhankelijk worden aangezet:

```bash
ros2 launch navigation boat.launch.py \
  with_temperature:=true \
  temperature_sensor_id:=28.C23646D48524 \
  with_sonar:=true \
  sonar_device:=D3:01:01:02:2F:C6 \
  with_lidar:=true \
  lidar_device:=/dev/ttyUSB0 \
  with_tof:=true \
  tof_rate_hz:=20.0 \
  with_bt785:=true \
  bt785_device:=AA:BB:CC:DD:EE:FF
```

Dezelfde argumenten bestaan op `sim.launch.py`, maar starten daar nog steeds de
**fysieke drivers**. Ze zijn bedoeld voor hardware-in-the-loop-tests en blijven
standaard uit. De simulator levert zelf al `/scan`, `/imu/data` en `/fix/valid`;
start daarom niet ook de fysieke LiDAR op hetzelfde `/scan`-topic tenzij dat
bewust de testopzet is.

Voor opname hoeven instrumenten niet aan te staan. De recorder neemt alleen data
op voor topics waarop werkelijk wordt gepubliceerd.

## Testen met Rancher Desktop / Docker

De launchbestanden en MCAP-recorder kunnen op een beheerde Windows-laptop worden
getest zonder ROS 2 lokaal te installeren. Zorg dat Rancher Desktop draait en de
Docker CLI de `default` context gebruikt:

```powershell
docker context use default
docker build -f docker/Dockerfile.ros-jazzy-test -t karaburan-ros-test .
docker run --rm karaburan-ros-test
```

De image:

1. bouwt de relevante ROS 2 Jazzy-packages;
2. compileert alle Pythonbestanden;
3. controleert de argumenten van beide nieuwe launchbestanden;
4. controleert de executables van de vijf meetinstrumenten;
5. publiceert test-GPS-data;
6. maakt een echte, gesegmenteerde MCAP-opname;
7. valideert die opname met `ros2 bag info`.

Deze smoke-test valideert geen seriële, Bluetooth- of I2C-hardware. Daarvoor zijn
mocks of een latere hardware-in-the-loop-test met device-passthrough nodig. Ook
de volledige Gazebo/RViz-GUI wordt niet in deze compacte testimage gestart; de
sim-launch wordt wel door Python gecompileerd.
