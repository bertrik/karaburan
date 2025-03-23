import math

from boat_interfaces.msg import BoatHeading
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


def heading_to_quaternion(heading_deg):
    # Convert compass heading from degrees to radians
    heading_rad = math.radians(heading_deg)

    # Calculate quaternion components
    qx = 0.0
    qy = 0.0
    qz = math.sin(heading_rad / 2)
    qw = math.cos(heading_rad / 2)

    # Return quaternion
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.compass_sub = self.create_subscription(BoatHeading, '/compass', self.compass_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 100)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0  # Heading from your custom compass

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.publish_pose()

    def compass_callback(self, heading):
        self.current_heading = heading.heading
        self.publish_pose()

    def publish_pose(self):
        hdr = Header(stamp=self.get_clock().now().to_msg())

        # Convert GPS to XY coordinates (this will need to be done with a transform library)
        # Simplified; typically requires a transform
        position = Point(x = self.current_lon, y = self.current_lat, z = 0.0)

        # Convert compass heading to quaternion for orientation
        orientation = heading_to_quaternion(self.current_heading)
        pose = Pose(position = position, orientation = orientation)

        pose_msg = PoseWithCovarianceStamped(header = hdr, pose = pose, covariance = [0] * 36)
        self.get_logger().info(f"Pose with cov: {pose_msg}")
        self.pose_pub.publish(pose_msg)

def main():
    try:
        rclpy.init()
        node = SensorFusionNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
