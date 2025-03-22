import pytest
from boat_interfaces.msg import BoatHeading
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from sensorfusion.sensorfusion_node import SensorFusionNode


@pytest.fixture
def node():
    """Fixture to initialize the ROS2 node for testing."""
    rclpy.init()
    test_node = SensorFusionNode()
    yield test_node
    test_node.destroy_node()
    rclpy.shutdown()


def test_gps_input(node):
    """Test if sensorfusion correctly receives and processes GPS data."""
    gps_msg = NavSatFix()
    gps_msg.latitude = 37.7749
    gps_msg.longitude = -122.4194
    gps_msg.altitude = 10.0

    node.gps_callback(gps_msg)  # Simulate receiving GPS data

    assert node.current_lat == 37.7749
    assert node.current_lon == -122.4194


def test_compass_input(node):
    """Test if sensorfusion correctly receives and processes compass heading."""
    compass_msg = BoatHeading()
    compass_msg.heading = 90  # East

    node.compass_callback(compass_msg)  # Simulate receiving compass data

    assert node.current_heading == 90

