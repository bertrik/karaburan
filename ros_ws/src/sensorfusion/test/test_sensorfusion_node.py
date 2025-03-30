import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.wait_for_message import wait_for_message
import launch
import launch_ros.actions
import launch_testing
from launch_testing_ros.wait_for_topics import WaitForTopics
import sys
import time
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
        launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()])
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
    self.msgs = []
    # Create a ROS node for tests
    self.node = rclpy.create_node('test_sensorfusion')
    # Create publishers for GPS & Compass
    self.gps_pub = self.node.create_publisher(NavSatFix, "/fix", 10)
    self.compass_pub = self.node.create_publisher(BoatHeading, "/compass", 10)
    self.pose_sub = self.node.create_subscription(PoseStamped, "/amcl_pose", lambda msg: self.msgs.append(msg), 10)

  def tearDown(self):
    self.node.destroy_subscription(self.pose_sub)
    self.node.destroy_node()

  def test_sensorfusion_publishes_fused_data(self, launch_service, proc_info, sensorfusion_node):
    """Test if sensorfusion correctly fuses GPS and compass data."""
    proc_info.assertWaitForStartup(sensorfusion_node, timeout=5.0)

    # Ensure the node is up before sending messages
    rclpy.spin_once(self.node, timeout_sec=2.0)
    rclpy.spin_once(self.node, timeout_sec=2.0)
    rclpy.spin_once(self.node, timeout_sec=2.0)

    # Publish fake GPS data
    gps_msg = NavSatFix()
    gps_msg.latitude = 37.7749
    gps_msg.longitude = -122.4194
    gps_msg.altitude = 10.0
    self.gps_pub.publish(gps_msg)

    rclpy.spin_once(self.node, timeout_sec=2.0)
    # Publish fake Compass data
    compass_msg = BoatHeading()
    compass_msg.heading = 90
    self.compass_pub.publish(compass_msg)
    
    # Listen to the pose topic for 10 s
    end_time = time.time() + 10
    while time.time() < end_time:
      # spin to get subscriber callback executed
      rclpy.spin_once(self.node, timeout_sec=1)

    assert len(self.msgs) > 0, self.msgs
    
    # 🕒 Wait for `sensorfusion` to publish a PoseStamped message
    fused_pose = self.msgs[-1]
    assert fused_pose is not None, f"SensorFusion did not publish a fused Pose! {fused_pose}"
    assert fused_pose.pose is not None, f"SensorFusion did not include a Pose! {fused_pose}"
    assert fused_pose.pose.position is not None, f"SensorFusion did not include a Position! {fused_pose}"
    assert fused_pose.pose.position.x != 0, "Expected nonzero position"
    assert fused_pose.pose.position.y != 0, "Expected nonzero position"
    assert fused_pose.pose.orientation.z != 0, "Expected valid heading"


