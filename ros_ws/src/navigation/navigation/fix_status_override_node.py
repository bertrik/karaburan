import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

class FixStatusOverrideNode(Node):
    def __init__(self):
        super().__init__('fix_status_override_node')
        self.sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.callback,
            10)
        self.pub = self.create_publisher(NavSatFix, '/fix/valid', 10)
        self.get_logger().info('Fix status override node started')

    def callback(self, msg):
        if msg.status.status < 0:
            msg.status.status = NavSatStatus.STATUS_FIX
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixStatusOverrideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

