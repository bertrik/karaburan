import math

import rclpy
from rclpy.node import Node
from boat_interfaces.msg import BoatHeading
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.compass_sub = self.create_subscription(BoatHeading, '/compass', self.compass_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None  # Heading from your custom compass

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.publish_pose()

    def compass_callback(self, heading):
        self.current_heading = heading
        self.publish_pose()

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
    
    def publish_pose(self):
        if self.current_lat is not None and self.current_lon is not None and self.current_heading is not None:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'odom'

            # Convert GPS to XY coordinates (this will need to be done with a transform library)
            pose_msg.pose.pose.position.x = self.current_lon  # Simplified; typically requires a transform
            pose_msg.pose.pose.position.y = self.current_lat

            # Convert compass heading to quaternion for orientation
            pose_msg.pose.pose.orientation = heading_to_quaternion(self.current_heading.heading)

            self.pose_pub.publish(pose_msg)


def main():
    rclpy.init()
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
