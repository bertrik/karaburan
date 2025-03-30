import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import time
import os
import unittest
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

class TestPubError(unittest.TestCase):

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
    self.node = rclpy.create_node('test_pub_error')
    # Create publishers for GPS & Compass
    self.pub = self.node.create_publisher(PoseStamped, "/topic", 10)

  def tearDown(self):
    self.node.destroy_node()

  def test_error(self):
    """Test that reproduces issue with strange Python behavior."""
    # Ensure the node is up before sending messages
    rclpy.spin_once(self.node, timeout_sec=2.0)

    hdr = Header(stamp=self.node.get_clock().now().to_msg())

    # Convert GPS to XY coordinates (this will need to be done with a transform library)
    # Simplified; typically requires a transform
    position = Point(x = 52.3, y = 6.8, z = 0.0)

    # Convert compass heading to quaternion for orientation
    orientation = Quaternion()
    pose = Pose(position = position, orientation = orientation)
    pose_msg = PoseStamped(header = hdr, pose = pose)
    self.node.get_logger().info(f"Pose with cov: {pose_msg}")

    self.pub.publish(pose_msg)

