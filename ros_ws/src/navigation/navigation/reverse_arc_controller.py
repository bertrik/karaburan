import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


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
        self.declare_parameter('arc_sample_step', 0.20)
        self.declare_parameter('footprint_half_length', 0.30)
        self.declare_parameter('footprint_half_width', 0.17)
        self.declare_parameter('front_obstacle_distance', 1.5)
        self.declare_parameter('front_obstacle_half_width', 0.4)

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
        self.arc_sample_step = self._parameter('arc_sample_step')
        self.footprint_half_length = self._parameter(
            'footprint_half_length')
        self.footprint_half_width = self._parameter('footprint_half_width')
        self.front_obstacle_distance = self._parameter(
            'front_obstacle_distance')
        self.front_obstacle_half_width = self._parameter(
            'front_obstacle_half_width')

        self.command_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.command_sub = self.create_subscription(
            Twist, '/cmd_vel_planner', self.command_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap_raw',
            self.costmap_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.plan_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
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
        self.costmap = None
        self.scan = None
        self.plan_generation = 0
        self.required_plan_generation = 0
        self.arc_start_plan_generation = 0

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

    def costmap_callback(self, costmap):
        self.costmap = costmap

    def scan_callback(self, scan):
        self.scan = scan

    def plan_callback(self, _plan):
        self.plan_generation += 1

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
            fresh_plan = self.plan_generation >= self.required_plan_generation
            if fresh_plan and command.linear.x > 0.01:
                self.state = 'completed'
                self.get_logger().info(
                    'Fresh forward path accepted from Nav2')
                self.command_pub.publish(command)
            return

        if (
            self.state == 'armed'
            and abs(command.linear.x) > 0.01
            and self.have_odometry
            and self.front_blocked()
        ):
            self.start_arc(command.angular.z)

        if self.state != 'arc':
            self.command_pub.publish(command)

    def start_arc(self, requested_angular_velocity):
        if abs(requested_angular_velocity) >= self.minimum_trigger_angular:
            requested_sign = (
                1.0 if requested_angular_velocity >= 0.0 else -1.0)
        else:
            requested_sign = self.obstacle_avoidance_sign()
        port_score = self.arc_cost(-1.0)
        starboard_score = self.arc_cost(1.0)
        if port_score < starboard_score:
            self.turn_sign = -1.0
        elif starboard_score < port_score:
            self.turn_sign = 1.0
        else:
            self.turn_sign = requested_sign
        self.distance = 0.0
        self.heading = 0.0
        self.previous_position = self.position
        self.previous_yaw = self.yaw
        self.start_time = self.get_clock().now()
        self.arc_start_plan_generation = self.plan_generation
        self.state = 'arc'
        self.get_logger().info(
            'Starting reverse arc: radius %.2f m, heading %.1f deg, direction %s'
            % (
                self.radius,
                math.degrees(self.heading_change),
                'starboard' if self.turn_sign > 0.0 else 'port',
            )
        )
        self.get_logger().info(
            'Reverse arc clearance scores: port=%s, starboard=%s'
            % (port_score, starboard_score)
        )

    def obstacle_avoidance_sign(self):
        """Choose the reverse arc away from the lidar obstacle centroid."""
        if self.scan is None:
            return 1.0
        lateral_sum = 0.0
        obstacle_count = 0
        angle = self.scan.angle_min
        for distance in self.scan.ranges:
            if math.isfinite(distance) and self.scan.range_min <= distance:
                forward = distance * math.cos(angle)
                lateral = distance * math.sin(angle)
                if (
                    0.0 < forward <= self.front_obstacle_distance
                    and abs(lateral) <= self.front_obstacle_half_width
                ):
                    lateral_sum += lateral
                    obstacle_count += 1
            angle += self.scan.angle_increment
        if obstacle_count == 0:
            return 1.0
        # A positive (port) obstacle requires the stern to arc starboard, and
        # vice versa. turn_sign describes that stern displacement convention.
        return 1.0 if lateral_sum >= 0.0 else -1.0

    def arc_cost(self, turn_sign):
        """Return the sampled costmap cost of one complete reverse arc."""
        if self.costmap is None or self.position is None:
            return (0, 0, 0)

        target_distance = self.radius * self.heading_change
        sample_count = max(
            1, math.ceil(target_distance / self.arc_sample_step))
        lethal_samples = 0
        maximum_cost = 0
        total_cost = 0
        footprint_points = (
            (0.0, 0.0),
            (self.footprint_half_length, self.footprint_half_width),
            (self.footprint_half_length, -self.footprint_half_width),
            (-self.footprint_half_length, self.footprint_half_width),
            (-self.footprint_half_length, -self.footprint_half_width),
        )

        for sample in range(1, sample_count + 1):
            angle = self.heading_change * sample / sample_count
            arc_x = -self.radius * math.sin(angle)
            arc_y = -turn_sign * self.radius * (1.0 - math.cos(angle))
            arc_yaw = turn_sign * angle
            for body_x, body_y in footprint_points:
                local_x = (
                    arc_x
                    + math.cos(arc_yaw) * body_x
                    - math.sin(arc_yaw) * body_y
                )
                local_y = (
                    arc_y
                    + math.sin(arc_yaw) * body_x
                    + math.cos(arc_yaw) * body_y
                )
                world_x = (
                    self.position[0]
                    + math.cos(self.yaw) * local_x
                    - math.sin(self.yaw) * local_y
                )
                world_y = (
                    self.position[1]
                    + math.sin(self.yaw) * local_x
                    + math.cos(self.yaw) * local_y
                )
                cost = self.cost_at(world_x, world_y)
                maximum_cost = max(maximum_cost, cost)
                total_cost += cost
                if cost >= 90:
                    lethal_samples += 1

        return (lethal_samples, maximum_cost, total_cost)

    def front_blocked(self):
        """Return whether lethal cost occupies the recovery corridor ahead."""
        if self.scan_front_blocked():
            return True
        if self.costmap is None or self.position is None:
            return False
        step = max(self.costmap.info.resolution, 0.05)
        forward = step
        while forward <= self.front_obstacle_distance:
            lateral = -self.front_obstacle_half_width
            while lateral <= self.front_obstacle_half_width:
                world_x = self.position[0] + (
                    math.cos(self.yaw) * forward
                    - math.sin(self.yaw) * lateral)
                world_y = self.position[1] + (
                    math.sin(self.yaw) * forward
                    + math.cos(self.yaw) * lateral)
                if self.cost_at(world_x, world_y) >= 90:
                    return True
                lateral += step
            forward += step
        return False

    def scan_front_blocked(self):
        """Detect an obstacle in the forward corridor directly from lidar."""
        if self.scan is None:
            return False
        angle = self.scan.angle_min
        for distance in self.scan.ranges:
            if math.isfinite(distance) and self.scan.range_min <= distance:
                forward = distance * math.cos(angle)
                lateral = distance * math.sin(angle)
                if (
                    0.0 < forward <= self.front_obstacle_distance
                    and abs(lateral) <= self.front_obstacle_half_width
                ):
                    return True
            angle += self.scan.angle_increment
        return False

    def cost_at(self, world_x, world_y):
        """Read one world-coordinate cell from the rolling occupancy grid."""
        info = self.costmap.info
        origin = info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        dx = world_x - origin.position.x
        dy = world_y - origin.position.y
        grid_x = math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy
        grid_y = -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy
        column = math.floor(grid_x / info.resolution)
        row = math.floor(grid_y / info.resolution)
        if column < 0 or row < 0 or column >= info.width or row >= info.height:
            return 100
        value = self.costmap.data[row * info.width + column]
        return 20 if value < 0 else value

    def timer_callback(self):
        if self.state == 'forward':
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.forward_handoff_timeout:
                self.get_logger().error(
                    'Nav2 did not provide a forward path after the reverse arc')
                self.command_pub.publish(Twist())
                self.state = 'completed'
                return
            # Wait for a freshly computed path. Moving blindly during planner
            # recovery can undo the clearance just created by the arc.
            self.command_pub.publish(Twist())
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
            # A plan generated during the forced arc starts from an obsolete
            # pose. Wait for the BT's next plan so Pure Pursuit resets its
            # pruning state and cannot latch onto an old Reeds-Shepp loop.
            # BackUp may complete just before the physical arc settles, so its
            # retry can publish the new path during these final tenths. That
            # path is already based on the recovered pose and is valid.
            self.required_plan_generation = self.arc_start_plan_generation + 1
            self.command_pub.publish(Twist())
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
