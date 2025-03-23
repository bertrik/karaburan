import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.wait_for_message import wait_for_message
import launch
import launch_ros.actions
import launch_testing
import sys
import os

@pytest.mark.rostest
def generate_test_description():
    """Start the sensorfusion node directly in the test."""
    sensorfusion_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(os.path.dirname(__file__), '../sensorfusion/sensorfusion_node.py')],
    )

    return launch.LaunchDescription([
        sensorfusion_node,
        launch_testing.actions.ReadyToTest()
    ]), {"sensorfusion_node": sensorfusion_node}


@pytest.mark.launch_test
def test_sensorfusion_publishes_fused_data(launch_service):
    """Test if sensorfusion correctly fuses GPS and compass data."""
    rclpy.init()
    test_node = Node("test_sensorfusion")

    # Create publishers for GPS & Compass
    gps_pub = test_node.create_publisher(NavSatFix, "gps", 10)
    compass_pub = test_node.create_publisher(Float32, "compass", 10)

    # Ensure the node is up before sending messages
    rclpy.spin_once(test_node, timeout_sec=2.0)

    # Publish fake GPS data
    gps_msg = NavSatFix()
    gps_msg.latitude = 37.7749
    gps_msg.longitude = -122.4194
    gps_msg.altitude = 10.0
    gps_pub.publish(gps_msg)

    # Publish fake Compass data
    compass_msg = BoatHeading()
    compass_msg.heading = 90
    compass_pub.publish(compass_msg)

    # 🕒 Wait for `sensorfusion` to publish a PoseStamped message
    fused_pose = wait_for_message(PoseStamped, test_node, "fused_pose", timeout=5.0)

    assert fused_pose is not None, "SensorFusion did not publish a fused Pose!"
    assert fused_pose.pose.position.x != 0, "Expected nonzero position"
    assert fused_pose.pose.position.y != 0, "Expected nonzero position"
    assert fused_pose.pose.orientation.z != 0, "Expected valid heading"

    test_node.destroy_node()
    rclpy.shutdown()

