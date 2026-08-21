"""Translate simulated velocity commands to Gazebo thruster forces."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def mix_thruster_forces(linear, angular, yaw_force_gain):
    """Return symmetric port and starboard force commands."""
    left_force = linear - angular * yaw_force_gain
    right_force = linear + angular * yaw_force_gain
    return left_force, right_force


class SimControlNode(Node):

    def __init__(self):
        super().__init__('sim_control_node')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('yaw_force_gain', 5.0)
        self.command_timeout = self.get_parameter(
            'command_timeout').get_parameter_value().double_value
        self.yaw_force_gain = self.get_parameter(
            'yaw_force_gain').get_parameter_value().double_value
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

    # Controls the propellors for the boat via duty cycle control.
    def props_callback(self, cmd_vel):
        # Converting twist message to differntial drive control, includes clipping
        # This is an electronic speed controller
        v = cmd_vel.linear.x     # (m/s)
        w = cmd_vel.angular.z    # (rad/s)
        self.last_command_time = self.get_clock().now()
        self.command_active = True

        # Both thrusters use the same +X axis and positive coefficient. Only
        # this differential mix determines translation and yaw.
        left, right = mix_thruster_forces(v, w, self.yaw_force_gain)

        self.publish_props(left, right)


def main():
    rclpy.init()
    node = SimControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
