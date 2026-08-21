import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(orientation):
    """Return planar yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class ReverseArcController(Node):
    """Hold one fast reverse arc instead of accepting an early path cusp."""

    def __init__(self):
        super().__init__('reverse_arc_controller')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('heading_change', math.pi / 2.0)
        self.declare_parameter('reverse_speed', 1.0)
        self.declare_parameter('angular_feedforward_gain', 1.0)
        self.declare_parameter('heading_gain', 0.5)
        self.declare_parameter('max_angular_command', 1.0)
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('forward_handoff_timeout', 15.0)
        self.declare_parameter('minimum_trigger_angular', 0.05)
        self.declare_parameter('heading_tolerance', 0.01)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('idle_reset_time', 1.0)
        self.declare_parameter('maneuver_timeout', 25.0)

        self.radius = self._parameter('radius')
        self.heading_change = self._parameter('heading_change')
        self.reverse_speed = self._parameter('reverse_speed')
        self.angular_feedforward_gain = self._parameter(
            'angular_feedforward_gain')
        self.heading_gain = self._parameter('heading_gain')
        self.max_angular_command = self._parameter('max_angular_command')
        self.forward_speed = self._parameter('forward_speed')
        self.forward_handoff_timeout = self._parameter(
            'forward_handoff_timeout')
        self.minimum_trigger_angular = self._parameter(
            'minimum_trigger_angular')
        self.heading_tolerance = self._parameter('heading_tolerance')
        self.distance_tolerance = self._parameter('distance_tolerance')
        self.idle_reset_time = self._parameter('idle_reset_time')
        self.maneuver_timeout = self._parameter('maneuver_timeout')

        self.command_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.command_sub = self.create_subscription(
            Twist, '/cmd_vel_planner', self.command_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.latest_planner_command = Twist()
        self.have_odometry = False
        self.position = None
        self.yaw = 0.0
        self.state = 'armed'
        self.turn_sign = 1.0
        self.distance = 0.0
        self.heading = 0.0
        self.previous_position = None
        self.previous_yaw = None
        self.start_time = None
        self.idle_since = None

    def _parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def odom_callback(self, odometry):
        pose = odometry.pose.pose
        new_position = (pose.position.x, pose.position.y)
        new_yaw = yaw_from_quaternion(pose.orientation)
        self.position = new_position
        self.yaw = new_yaw
        self.have_odometry = True

        if self.state != 'arc' or self.previous_position is None:
            return

        step = math.hypot(
            new_position[0] - self.previous_position[0],
            new_position[1] - self.previous_position[1],
        )
        if step < 0.5:
            self.distance += step
        yaw_step = normalize_angle(new_yaw - self.previous_yaw)
        self.heading += self.turn_sign * yaw_step
        self.previous_position = new_position
        self.previous_yaw = new_yaw

    def command_callback(self, command):
        self.latest_planner_command = command
        moving = abs(command.linear.x) > 0.01 or abs(command.angular.z) > 0.01
        now = self.get_clock().now()

        if self.state == 'completed':
            if moving:
                self.idle_since = None
            elif self.idle_since is None:
                self.idle_since = now
            elif (now - self.idle_since).nanoseconds / 1e9 >= self.idle_reset_time:
                self.state = 'armed'
                self.get_logger().info('Reverse arc re-armed for the next goal')

            # The planner may keep advertising its old reverse segment until
            # the next distance-triggered replan. Do not start a second cusp.
            if command.linear.x < -0.01:
                return

        if self.state == 'forward':
            if command.linear.x > 0.01:
                self.state = 'completed'
                self.get_logger().info('Forward path accepted from Nav2')
                self.command_pub.publish(command)
            elif not moving:
                self.command_pub.publish(Twist())
                self.state = 'completed'
            return

        if (
            self.state == 'armed'
            and command.linear.x < -0.01
            and abs(command.angular.z) >= self.minimum_trigger_angular
            and self.have_odometry
        ):
            self.start_arc(command.angular.z)

        if self.state != 'arc':
            self.command_pub.publish(command)

    def start_arc(self, requested_angular_velocity):
        self.turn_sign = 1.0 if requested_angular_velocity >= 0.0 else -1.0
        self.distance = 0.0
        self.heading = 0.0
        self.previous_position = self.position
        self.previous_yaw = self.yaw
        self.start_time = self.get_clock().now()
        self.state = 'arc'
        self.get_logger().info(
            'Starting reverse arc: radius %.2f m, heading %.1f deg, direction %s'
            % (
                self.radius,
                math.degrees(self.heading_change),
                'left' if self.turn_sign > 0.0 else 'right',
            )
        )

    def timer_callback(self):
        if self.state == 'forward':
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.forward_handoff_timeout:
                self.get_logger().error(
                    'Nav2 did not provide a forward path after the reverse arc')
                self.command_pub.publish(Twist())
                self.state = 'completed'
                return
            command = Twist()
            command.linear.x = self.forward_speed
            self.command_pub.publish(command)
            return

        if self.state != 'arc':
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.maneuver_timeout:
            self.get_logger().error('Reverse arc timed out; stopping the boat')
            self.command_pub.publish(Twist())
            self.state = 'completed'
            return

        target_distance = self.radius * self.heading_change
        distance_done = self.distance >= target_distance - self.distance_tolerance
        heading_done = self.heading >= self.heading_change - self.heading_tolerance
        if distance_done and heading_done:
            self.get_logger().info(
                'Reverse arc complete: %.2f m, %.1f deg in %.1f s'
                % (self.distance, math.degrees(self.heading), elapsed)
            )
            self.state = 'forward'
            self.start_time = self.get_clock().now()
            command = Twist()
            command.linear.x = self.forward_speed
            self.command_pub.publish(command)
            return

        desired_heading = min(self.distance / self.radius, self.heading_change)
        heading_error = desired_heading - self.heading
        nominal_angular = self.reverse_speed / self.radius
        angular_command = (
            self.angular_feedforward_gain * nominal_angular
            + self.heading_gain * heading_error
        )
        angular_command = max(
            0.0, min(self.max_angular_command, angular_command))

        command = Twist()
        command.linear.x = -self.reverse_speed
        command.angular.z = self.turn_sign * angular_command
        self.command_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ReverseArcController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
