import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.wait_for_message import wait_for_message
import launch
import launch_ros.actions
import launch_testing
import sys
import os
import unittest
from boat_interfaces.msg import BoatHeading

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


class TestSensorFusion(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    # Initialize the ROS context for the test node
    rclpy.init()

  @classmethod
  def tearDownClass(cls):
    # Shutdown the ROS context
    rclpy.shutdown()

  def setUp(self):
    # Create a ROS node for tests
    self.node = rclpy.create_node('test_sensorfusion')
    # Create publishers for GPS & Compass
    self.gps_pub = self.node.create_publisher(NavSatFix, "gps", 10)
    self.compass_pub = self.node.create_publisher(BoatHeading, "compass", 10)

  def tearDown(self):
    self.node.destroy_node()

  def test_sensorfusion_publishes_fused_data(self, launch_service):
    """Test if sensorfusion correctly fuses GPS and compass data."""

    # Ensure the node is up before sending messages
    rclpy.spin_once(self.node, timeout_sec=2.0)

    # Publish fake GPS data
    gps_msg = NavSatFix()
    gps_msg.latitude = 37.7749
    gps_msg.longitude = -122.4194
    gps_msg.altitude = 10.0
    self.gps_pub.publish(gps_msg)

    # Publish fake Compass data
    compass_msg = BoatHeading()
    compass_msg.heading = 90
    self.compass_pub.publish(compass_msg)

    # 🕒 Wait for `sensorfusion` to publish a PoseStamped message
    fused_pose = wait_for_message(PoseWithCovarianceStamped, self.node, "/amcl_pose", time_to_wait= 5.0)

    assert fused_pose is not None, "SensorFusion did not publish a fused Pose!"
    assert fused_pose.pose is not None, "SensorFusion did not publish a fused Pose!"
    assert fused_pose.pose.pose is not None, "SensorFusion did not publish a fused Pose!"
    assert fused_pose.pose.pose.x != 0, "Expected nonzero position"
    assert fused_pose.pose.pose.y != 0, "Expected nonzero position"
    assert fused_pose.pose.orientation.z != 0, "Expected valid heading"

