import pytest
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from boat_interfaces.msg import Waypoints
import time
import launch
import launch_ros.actions
import launch_testing
from launch_testing_ros.wait_for_topics import WaitForTopics
import sys
import time
import os

@pytest.mark.rostest
def generate_test_description():
    """Start the sensorfusion node directly in the test."""
    boatnavigator_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(os.path.dirname(__file__), '../navigation/boatnavigator.py')],
    )

    return launch.LaunchDescription([
        boatnavigator_node,
        launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()])
    ]), {"boatnavigator_node": boatnavigator_node}

class TestBoatNavigatorNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.headings = []
        self.speeds = []
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_boatnavigator')
        # Create publishers for GPS & Compass
        self.waypoints_pub = self.node.create_publisher(Waypoints, "/waypoints", 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, "/amcl_pose", 10)
        self.heading_sub = self.node.create_subscription(Float64, "/desired_heading", lambda msg: self.headings.append(msg), 10)
        self.speed_sub = self.node.create_subscription(Float64, "/desired_speed", lambda msg: self.speeds.append(msg), 10)

    def tearDown(self):
        self.node.destroy_subscription(self.heading_sub)
        self.node.destroy_subscription(self.speed_sub)
        self.node.destroy_node()

    def test_navigator_publishes_heading_and_speed(self, launch_service, proc_info, boatnavigator_node):
        """Publish a waypoint and check if heading & speed are correctly published."""
        timeout = 5.0
        start_time = time.time()
        proc_info.assertWaitForStartup(boatnavigator_node, timeout=5.0)

        # Ensure the node is up before sending messages
        rclpy.spin_once(self.node, timeout_sec=2.0)

        # Send a test waypoint
        waypoint_msg = Waypoints()
        waypoint_msg.gps_poses = []
        waypoint_msg.gps_poses.append(PoseStamped())
        self.waypoints_pub.publish(waypoint_msg)

        # Wait for a response
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.speeds and self.headings:
                break

        self.assertGreater(len(self.speeds), 0, "No messages received!")
        self.assertGreater(len(self.headings), 0, "No messages received!")

        self.assertIsInstance(self.headings[0], float, "Heading should be a float")
        self.assertIsInstance(self.speeds[0], float, "Speed should be a float")


if __name__ == '__main__':
    unittest.main()

