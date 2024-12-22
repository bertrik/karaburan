The temperature reader node is responsible for publishing the OWFS One Wire based temperature sensors.

Please make sure that OWFS is configured correctly before running this node.

Probably running will be fine using the command (or similar sensor ID):
`ros2 run tempreader tempreaderNode --ros-args -p sensorId:=28.C23646D48524 &`
