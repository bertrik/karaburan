import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class FixStatusOverrideNode(Node):

    def __init__(self):
        super().__init__('fix_status_override_node')
        self.declare_parameter('override_invalid_status', True)
        self.override_invalid_status = self.get_parameter(
            'override_invalid_status').value
        self.sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.callback,
            10)
        self.pub = self.create_publisher(NavSatFix, '/fix/valid', 10)
        state = 'enabled' if self.override_invalid_status else 'disabled'
        self.get_logger().info(
            f'Fix status override node started; status override is {state}')

    def callback(self, msg):
        if self.override_invalid_status and msg.status.status < 0:
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
