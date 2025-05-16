import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float64

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
        self.compass_sub = self.create_subscription(Float64, '/compass', self.compass_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 100)
        self.current_heading = 0  # Heading from your custom compass

    def compass_callback(self, heading):
        self.current_heading = heading.data
        self.publish_pose()

    def publish_pose(self):
        hdr = Header(stamp=self.get_clock().now().to_msg(), frame_id='imu_link')

        # Convert compass heading to quaternion for orientation
        orientation = heading_to_quaternion(self.current_heading)
        cov = [ 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu = Imu(header = hdr, orientation = orientation, orientation_covariance = cov, angular_velocity_covariance = cov, linear_acceleration_covariance = cov)

        self.get_logger().debug(f"Pose with cov: {imu}")
        self.imu_pub.publish(imu)

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
