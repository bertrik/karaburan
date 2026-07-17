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

