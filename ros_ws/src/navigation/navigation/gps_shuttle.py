import math

from geographic_msgs.msg import GeoPose
from nav2_msgs.action import FollowGPSWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


SHUTTLE_ROUTE = [
    (52.018599, 4.708720),
    (52.018962, 4.708961),
]
FOUR_POINT_ROUTE = [
    (52.042296, 4.747545),
    (52.042648, 4.748077),
    (52.042143, 4.748323),
    (52.041718, 4.747608),
]
MAX_ADDITIONAL_LOOPS = (2 ** 32) - 1


def bearing_radians(start, destination):
    """Return the initial WGS84 bearing, clockwise from north."""
    latitude_1 = math.radians(start[0])
    latitude_2 = math.radians(destination[0])
    longitude_delta = math.radians(destination[1] - start[1])
    east = math.sin(longitude_delta) * math.cos(latitude_2)
    north = (
        math.cos(latitude_1) * math.sin(latitude_2)
        - math.sin(latitude_1) * math.cos(latitude_2)
        * math.cos(longitude_delta)
    )
    return math.atan2(east, north)


def make_geo_pose(position, face_towards):
    pose = GeoPose()
    pose.position.latitude = position[0]
    pose.position.longitude = position[1]
    pose.position.altitude = 0.0

    # Geographic bearings start at north and turn clockwise. ROS ENU yaw starts
    # at east and turns counter-clockwise.
    yaw = (math.pi / 2.0) - bearing_radians(position, face_towards)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class GpsRoute(Node):

    def __init__(self, node_name, route_name, route):
        super().__init__(node_name)
        self._route_name = route_name
        self._route = route
        self._client = ActionClient(
            self, FollowGPSWaypoints, 'follow_gps_waypoints'
        )
        self._goal_handle = None
        self._current_waypoint = None

    def run(self):
        self.get_logger().info(
            'Waiting for the /follow_gps_waypoints action server...'
        )
        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 is not ready yet; still waiting...')

        if not rclpy.ok():
            return

        goal = FollowGPSWaypoints.Goal()
        goal.gps_poses = [
            make_geo_pose(point, self._route[(index + 1) % len(self._route)])
            for index, point in enumerate(self._route)
        ]
        goal.number_of_loops = MAX_ADDITIONAL_LOOPS
        goal.goal_index = 0

        self.get_logger().info(
            'Starting GPS route %s with %d waypoints. Press Ctrl+C to stop.'
            % (self._route_name, len(self._route))
        )
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()

        if self._goal_handle is None or not self._goal_handle.accepted:
            raise RuntimeError('Nav2 rejected the GPS route goal')

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if result_future.done():
            result = result_future.result()
            if result is not None:
                self.get_logger().info(
                    'GPS route stopped with action status %d' % result.status
                )

    def cancel(self):
        if self._goal_handle is None or not rclpy.ok():
            return
        self.get_logger().info('Cancelling the active GPS route goal...')
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)

    def _feedback_callback(self, feedback_message):
        waypoint = feedback_message.feedback.current_waypoint
        if waypoint == self._current_waypoint:
            return
        self._current_waypoint = waypoint
        self.get_logger().info('Navigating to point %d' % (waypoint + 1))


def run_route(node_name, route_name, route, args=None):
    # Keep Python's normal SIGINT behavior so the action can be cancelled
    # before the ROS context is shut down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GpsRoute(node_name, route_name, route)
    try:
        node.run()
    except KeyboardInterrupt:
        node.cancel()
    except RuntimeError as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    run_route('gps_shuttle', 'two-point shuttle', SHUTTLE_ROUTE, args)


def main_four_point(args=None):
    run_route(
        'gps_four_point_route', 'four-point route', FOUR_POINT_ROUTE, args
    )


if __name__ == '__main__':
    main()
