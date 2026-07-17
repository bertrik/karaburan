#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /karaburan/ros_ws/install/setup.bash
set -u

python3 -m compileall -q /karaburan/ros_ws/src
ros2 launch navigation storage.launch.py --show-args >/tmp/storage-args.txt
ros2 launch navigation measurement_instruments.launch.py --show-args >/tmp/instrument-args.txt

for executable in \
    "tempreader tempreaderNode" \
    "sonar sonar_node" \
    "lidar lidar_node" \
    "vl53l0x vl53l0x_node" \
    "bt785 bt785_node"; do
    package="${executable%% *}"
    name="${executable##* }"
    ros2 pkg executables "$package" | grep -q "$name"
done

bag_root=/tmp/karaburan-test-bags
rm -rf "$bag_root"
mkdir -p "$bag_root"

ros2 topic pub -r 5 /fix sensor_msgs/msg/NavSatFix \
    "{header: {frame_id: gps_link}, latitude: 52.0, longitude: 5.0, altitude: 0.0}" \
    >/tmp/test-publisher.log 2>&1 &
publisher_pid=$!

set +e
timeout --signal=INT --kill-after=5s 9s \
    ros2 launch navigation storage.launch.py \
        enabled:=true \
        profile:=minimal \
        output_dir:="$bag_root" \
        bag_prefix:=docker-test \
        start_delay:=0.5 \
        max_bag_duration:=4 \
        max_bag_size:=0 \
        >/tmp/storage-launch.log 2>&1
launch_status=$?
set -e

kill -INT "$publisher_pid" 2>/dev/null || true
wait "$publisher_pid" 2>/dev/null || true

if [[ "$launch_status" -ne 0 && "$launch_status" -ne 124 ]]; then
    cat /tmp/storage-launch.log
    exit "$launch_status"
fi

bag_dir="$(find "$bag_root" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
if [[ -z "$bag_dir" ]] || ! find "$bag_dir" -name '*.mcap' -type f -size +0c | grep -q .; then
    cat /tmp/storage-launch.log
    echo 'No non-empty MCAP file was produced' >&2
    exit 1
fi

ros2 bag info "$bag_dir"
echo "Docker ROS 2 launch and MCAP smoke test passed: $bag_dir"
