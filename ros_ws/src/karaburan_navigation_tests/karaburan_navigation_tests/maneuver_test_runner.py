"""Run one short, machine-verifiable navigation manoeuvre scenario."""

import argparse
import json
import math
from pathlib import Path as FilePath
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from karaburan_navigation_tests.maneuver_metrics import (
    obstacle_report,
    path_length,
    planner_direct_report,
    straight_report,
    turn_report,
)
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


SCENARIOS = (
    'actuator_straight',
    'actuator_turn_left',
    'actuator_turn_right',
    'follow_straight',
    'follow_arc_left',
    'follow_arc_right',
    'planner_direct',
    'obstacle_port',
    'obstacle_starboard',
)


def yaw_from_quaternion(orientation):
    """Return planar yaw from a quaternion."""
    return math.atan2(
        2.0 * orientation.w * orientation.z,
        1.0 - 2.0 * orientation.z * orientation.z,
    )


def quaternion_from_yaw(yaw):
    """Create the z/w parts of a planar quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ManeuverTestNode(Node):
    """Collect trace data and execute one isolated manoeuvre."""

    def __init__(self):
        super().__init__('maneuver_test_runner')
        self.odometry = None
        self.command = Twist()
        self.samples = []
        self.latest_plan = None
        self.reference_path = []
        self.markers = []
        self.initial_plan_length = None
        self.command_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.follow_client = ActionClient(self, FollowPath, '/follow_path')
        self.navigate_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.planner_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')

    def odometry_callback(self, message):
        self.odometry = message
        pose = message.pose.pose
        stamp = message.header.stamp
        self.samples.append({
            'time': stamp.sec + stamp.nanosec / 1e9,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw_from_quaternion(pose.orientation),
            'linear': self.command.linear.x,
            'angular': self.command.angular.z,
        })

    def command_callback(self, message):
        self.command = message

    def plan_callback(self, message):
        self.latest_plan = message
        if len(message.poses) > 1:
            self.reference_path = [
                {'x': pose.pose.position.x, 'y': pose.pose.position.y}
                for pose in message.poses
            ]
        if self.initial_plan_length is None and len(message.poses) > 1:
            self.initial_plan_length = path_length(self.reference_path)

    def wait_for_odometry(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while self.odometry is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.odometry is None:
            raise RuntimeError('No filtered odometry received')
        self.samples = []

    def publish_for(self, command, stop_condition, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.command_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
            if stop_condition():
                break
        self.command_pub.publish(Twist())
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_action(self, client, goal, timeout=45.0):
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('Navigation action server is unavailable')
        goal_future = client.send_goal_async(goal)
        self._spin_until(goal_future, 10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Navigation goal was rejected')
        result_future = goal_handle.get_result_async()
        self._spin_until(result_future, timeout)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return False
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def compute_path(self, goal, timeout=15.0):
        if not self.planner_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('Planner action server is unavailable')
        goal_future = self.planner_client.send_goal_async(goal)
        self._spin_until(goal_future, 10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Planner goal was rejected')
        result_future = goal_handle.get_result_async()
        self._spin_until(result_future, timeout)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise RuntimeError('Planner action timed out')
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'Planner action failed with status {result.status}')
        return result.result.path

    def _spin_until(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def current_pose(self):
        return self.odometry.pose.pose


def actuator_straight(node):
    start = node.current_pose().position
    command = Twist()
    command.linear.x = 0.25
    node.publish_for(
        command,
        lambda: math.hypot(
            node.current_pose().position.x - start.x,
            node.current_pose().position.y - start.y,
        ) >= 3.0,
    )
    return straight_report(node.samples, target_distance=3.0)


def actuator_turn(node, sign):
    start_yaw = yaw_from_quaternion(node.current_pose().orientation)
    command = Twist()
    command.linear.x = 0.25
    command.angular.z = sign * 0.08
    node.publish_for(
        command,
        lambda: abs(_yaw_since(node, start_yaw)) >= math.radians(50.0),
    )
    return turn_report(node.samples, sign)


def _yaw_since(node, start_yaw):
    current_yaw = yaw_from_quaternion(node.current_pose().orientation)
    return math.atan2(
        math.sin(current_yaw - start_yaw),
        math.cos(current_yaw - start_yaw),
    )


def follow_path(node, arc_sign=0.0):
    path = _make_path(node.current_pose(), arc_sign)
    node.reference_path = [
        {'x': pose.pose.position.x, 'y': pose.pose.position.y}
        for pose in path.poses
    ]
    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = 'FollowPath'
    goal.goal_checker_id = 'goal_checker'
    succeeded = node.send_action(node.follow_client, goal, timeout=35.0)
    if arc_sign == 0.0:
        report = straight_report(node.samples, target_distance=3.0)
    else:
        report = turn_report(node.samples, arc_sign)
    report['checks']['action_succeeded'] = succeeded
    report['passed'] = all(report['checks'].values())
    return report


def _make_path(start, arc_sign):
    result = Path()
    result.header.frame_id = 'odom'
    start_yaw = yaw_from_quaternion(start.orientation)
    if arc_sign == 0.0:
        local_poses = [
            (distance, 0.0, 0.0)
            for distance in [index * 0.25 for index in range(13)]
        ]
    else:
        radius = 5.0
        local_poses = [
            (
                radius * math.sin(angle),
                arc_sign * radius * (1.0 - math.cos(angle)),
                arc_sign * angle,
            )
            for angle in [index * math.radians(5.0) for index in range(10)]
        ]
    for local_x, local_y, local_yaw in local_poses:
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = (
            start.position.x
            + math.cos(start_yaw) * local_x
            - math.sin(start_yaw) * local_y
        )
        pose.pose.position.y = (
            start.position.y
            + math.sin(start_yaw) * local_x
            + math.cos(start_yaw) * local_y
        )
        pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_yaw(
            start_yaw + local_yaw)
        result.poses.append(pose)
    return result


def planner_direct(node):
    """Request aligned global plans at representative lattice headings."""
    distance = 30.0
    current = node.current_pose()
    base_yaw = yaw_from_quaternion(current.orientation)
    cases = []
    for offset_degrees in (0.0, 3.0, 13.0, 37.0, 91.0, 143.0):
        yaw = base_yaw + math.radians(offset_degrees)
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = current.position.x
        start.pose.position.y = current.position.y
        start.pose.orientation.z, start.pose.orientation.w = (
            quaternion_from_yaw(yaw))
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = (
            start.pose.position.x + distance * math.cos(yaw))
        goal_pose.pose.position.y = (
            start.pose.position.y + distance * math.sin(yaw))
        goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = (
            quaternion_from_yaw(yaw))

        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal_pose
        request.planner_id = 'GridBased'
        request.use_start = True
        path = node.compute_path(request)
        points = [
            {'x': pose.pose.position.x, 'y': pose.pose.position.y}
            for pose in path.poses
        ]
        case = planner_direct_report(
            points,
            (start.pose.position.x, start.pose.position.y),
            (goal_pose.pose.position.x, goal_pose.pose.position.y),
        )
        case['offset_degrees'] = offset_degrees
        case['points'] = points
        case['goal'] = (
            goal_pose.pose.position.x, goal_pose.pose.position.y)
        cases.append(case)

    worst = max(cases, key=lambda case: case['metrics'].get(
        'length_ratio', math.inf))
    node.reference_path = worst.pop('points')
    goal_x, goal_y = worst.pop('goal')
    node.markers = [
        {'x': current.position.x, 'y': current.position.y, 'label': 'start'},
        {'x': goal_x, 'y': goal_y, 'label': 'goal'},
    ]
    checks = {
        name: all(case['checks'].get(name, False) for case in cases)
        for name in worst['checks']
    }
    return {
        'passed': all(checks.values()),
        'checks': checks,
        'metrics': {
            'worst_offset_degrees': worst['offset_degrees'],
            'cases': {
                str(case['offset_degrees']): case['metrics']
                for case in cases
            },
        },
    }


def obstacle(node, side):
    block_y = 0.45 if side == 'port' else -0.45
    _spawn_block(block_y)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = 8.0
    goal.pose.pose.orientation.w = 1.0
    succeeded = node.send_action(node.navigate_client, goal, timeout=45.0)
    report = obstacle_report(
        node.samples,
        goal=(8.0, 0.0),
        obstacle=(1.5, block_y),
        planned_length=node.initial_plan_length,
    )
    report['checks']['action_succeeded'] = succeeded
    report['passed'] = all(report['checks'].values())
    return report


def _spawn_block(y_position):
    share = get_package_share_directory('karaburan_navigation_tests')
    model = FilePath(share) / 'scenarios' / 'maneuver_test_block.sdf'
    request = (
        'sdf_filename: "' + str(model) + '", '
        'pose: {position: {x: 1.5, y: ' + str(y_position) + '}}'
    )
    completed = subprocess.run(
        [
            'gz', 'service', '-s', '/world/ocean/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
            '--req', request,
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    if completed.returncode != 0 or 'data: true' not in completed.stdout:
        raise RuntimeError(
            'Could not create test block: '
            + completed.stdout + completed.stderr)


def execute(node, scenario):
    if scenario == 'actuator_straight':
        return actuator_straight(node)
    if scenario == 'actuator_turn_left':
        return actuator_turn(node, 1.0)
    if scenario == 'actuator_turn_right':
        return actuator_turn(node, -1.0)
    if scenario == 'follow_straight':
        return follow_path(node)
    if scenario == 'follow_arc_left':
        return follow_path(node, 1.0)
    if scenario == 'follow_arc_right':
        return follow_path(node, -1.0)
    if scenario == 'planner_direct':
        return planner_direct(node)
    if scenario == 'obstacle_port':
        return obstacle(node, 'port')
    if scenario == 'obstacle_starboard':
        return obstacle(node, 'starboard')
    raise ValueError('Unknown scenario: ' + scenario)


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=SCENARIOS, required=True)
    parser.add_argument('--output', type=FilePath, required=True)
    return parser.parse_args(args)


def scenario_markers(scenario):
    """Return fixed points that help interpret a scenario plot."""
    if scenario == 'obstacle_port':
        return [
            {'x': 1.5, 'y': 0.45, 'label': 'obstacle'},
            {'x': 8.0, 'y': 0.0, 'label': 'goal'},
        ]
    if scenario == 'obstacle_starboard':
        return [
            {'x': 1.5, 'y': -0.45, 'label': 'obstacle'},
            {'x': 8.0, 'y': 0.0, 'label': 'goal'},
        ]
    return []


def main(args=None):
    options = parse_args(args)
    started = time.monotonic()
    rclpy.init()
    node = ManeuverTestNode()
    try:
        node.wait_for_odometry()
        report = execute(node, options.scenario)
    except Exception as error:  # noqa: BLE001 - test report must survive failures
        report = {
            'passed': False,
            'checks': {'runner_error': False},
            'metrics': {},
            'error': str(error),
        }
    finally:
        node.command_pub.publish(Twist())
        report['scenario'] = options.scenario
        report['samples'] = node.samples
        report['reference_path'] = node.reference_path
        report['markers'] = node.markers or scenario_markers(options.scenario)
        report['duration_seconds'] = time.monotonic() - started
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({
        'scenario': options.scenario,
        'passed': report['passed'],
        'checks': report['checks'],
        'output': str(options.output),
    }, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
