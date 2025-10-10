#python 3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class SimControlNode(Node):

    def __init__(self):
        super().__init__('sim_control_node')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.props_callback, 10)
        self.left_pub = self.create_publisher(Float64, '/left', 10)
        self.right_pub = self.create_publisher(Float64, '/right', 10)

    # Controls the propellors for the boat via duty cycle control.
    def props_callback(self, cmd_vel):
        # Converting twist message to differntial drive control, includes clipping
        # This is an electronic speed controller
        v = cmd_vel.linear.x     # (m/s)
        w = cmd_vel.angular.z    # (rad/s)

        B = 0.3                  # (m)
        K = 1                    # To be determined!
        left  = (v - w*B/2) / K
        right = (v + w*B/2) / K

        self.left_pub.publish(Float64(data = left))
        self.right_pub.publish(Float64(data = right))

def main():
    rclpy.init()
    node = SimControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

