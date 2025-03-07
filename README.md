# karaburan

Load ROS 2:
`source /opt/ros/jazzy/setup.bash`

Build and run temperature sensor node (my SensorID):
`colcon build && source ./install/setup.bash && ros2 run tempreader tempreaderNode --ros-args -p sensorId:=28.C23646D48524 &`

Launch GPSD client node:
`ros2 launch gpsd_client gpsd_client-launch.py &`

Read fixes from GPS:
`ros2 topic echo /fix`

Read temperature:
`ros2 topic echo /temperature`
