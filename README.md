# karaburan

Load ROS 2:
`source /opt/ros/jazzy/setup.bash`

Navigate to ROS workspace:
`cd ros_ws`

# Temerature sensor
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
ros2-apt-source

