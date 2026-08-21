# python 3
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimControlNode(Node):

    def __init__(self):
        super().__init__('sim_control_node')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('yaw_force_gain', 5.0)
        self.declare_parameter('reference_speed', 0.25)
        self.command_timeout = self.get_parameter(
            'command_timeout').get_parameter_value().double_value
        self.yaw_force_gain = self.get_parameter(
            'yaw_force_gain').get_parameter_value().double_value
        self.reference_speed = self.get_parameter(
            'reference_speed').get_parameter_value().double_value
        self.last_command_time = None
        self.command_active = False
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.props_callback, 10)
        self.left_pub = self.create_publisher(Float64, '/left', 10)
        self.right_pub = self.create_publisher(Float64, '/right', 10)
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)

    def publish_props(self, left, right):
        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))

    def watchdog_callback(self):
        if not self.command_active or self.last_command_time is None:
            return
        command_age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if command_age > self.command_timeout:
            self.publish_props(0.0, 0.0)
            self.command_active = False

    def shaft_speed(self, requested_force):
        """Invert Gazebo's quadratic propeller force around cruise speed."""
        if requested_force == 0.0:
            return 0.0
        return math.copysign(
            math.sqrt(abs(requested_force) * self.reference_speed),
            requested_force,
        )

    # Controls the propellors for the boat via duty cycle control.
    def props_callback(self, cmd_vel):
        # Converting twist message to differntial drive control, includes clipping
        # This is an electronic speed controller
        v = cmd_vel.linear.x     # (m/s)
        w = cmd_vel.angular.z    # (rad/s)
        self.last_command_time = self.get_clock().now()
        self.command_active = True

        # Mix desired forces first. Converting shaft velocities directly would
        # couple total surge force to steering because Gazebo squares them.
        left_force = v - w * self.yaw_force_gain
        right_force = v + w * self.yaw_force_gain
        left = self.shaft_speed(left_force)
        # The starboard propeller axis is reversed in the simulation model so
        # equal forward thrust uses the opposite shaft rotation direction.
        right = -self.shaft_speed(right_force)

        self.publish_props(left, right)


def main():
    rclpy.init()
    node = SimControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
