import math

from karaburan_navigation_tests.maneuver_metrics import (
    obstacle_report,
    path_tracking_report,
    planner_direct_report,
    planner_island_report,
    straight_report,
    turn_report,
)


def sample(time, x, y, yaw, linear, angular=0.0):
    return {
        'time': time,
        'x': x,
        'y': y,
        'yaw': yaw,
        'linear': linear,
        'angular': angular,
    }


def test_straight_trace_passes_and_circle_fails():
    straight = [
        sample(index, index * 0.1, 0.01 * math.sin(index), 0.0, 0.25)
        for index in range(31)
    ]
    circle = [
        sample(index, math.sin(index / 5.0), math.cos(index / 5.0),
               index / 5.0, 0.25, 0.1)
        for index in range(31)
    ]

    assert straight_report(straight)['passed']
    assert not straight_report(circle)['passed']


def test_turn_sign_is_checked_in_both_directions():
    left = [sample(index, 0.0, 0.0, index * 0.1, 0.25, 0.1)
            for index in range(10)]
    right = [sample(index, 0.0, 0.0, -index * 0.1, 0.25, -0.1)
             for index in range(10)]

    assert turn_report(left, 1.0)['passed']
    assert turn_report(right, -1.0)['passed']
    assert not turn_report(left, -1.0)['passed']


def test_path_tracking_rejects_reverse_escape_from_a_normal_arc():
    reference = [
        {'x': index * 0.1, 'y': index * index * 0.002}
        for index in range(31)
    ]
    passing = [
        sample(index, point['x'], point['y'], index * 0.01, 0.25)
        for index, point in enumerate(reference)
    ]
    escaped = passing + [
        sample(31, 1.0, 2.0, -0.5, -1.0, -0.5),
    ]

    assert path_tracking_report(passing, reference, 1.0)['passed']
    failed = path_tracking_report(escaped, reference, 1.0)
    assert not failed['passed']
    assert not failed['checks']['controller_forward_only']
    assert not failed['checks']['controller_cross_track']


def test_direct_planner_path_passes_and_v_path_fails():
    direct = [{'x': index * 0.25, 'y': 0.0} for index in range(33)]
    v_path = [
        {'x': 0.0, 'y': 0.0},
        {'x': -2.0, 'y': 2.0},
        {'x': 8.0, 'y': 0.0},
    ]

    assert planner_direct_report(direct, (0.0, 0.0), (8.0, 0.0))['passed']
    failed = planner_direct_report(v_path, (0.0, 0.0), (8.0, 0.0))
    assert not failed['passed']
    assert not failed['checks']['planner_path_length']
    assert not failed['checks']['planner_cross_track']
    assert not failed['checks']['planner_monotonic']


def test_island_plan_requires_one_smooth_side_without_a_cusp():
    route = [
        {'x': index * 0.5, 'y': 3.0 * math.sin(math.pi * index / 40.0)}
        for index in range(41)
    ]
    report = planner_island_report(
        route, (0.0, 0.0), (20.0, 0.0), (8.0, 0.0), 2.5)

    assert report['passed']
    loop = route[:20] + list(reversed(route[10:20])) + route[20:]
    failed = planner_island_report(
        loop, (0.0, 0.0), (20.0, 0.0), (8.0, 0.0), 2.5)
    assert not failed['passed']
    assert not failed['checks']['planner_monotonic']
    assert not failed['checks']['planner_no_cusps']


def test_obstacle_trace_requires_one_switch_and_rejects_a_loop():
    reverse = [
        sample(index * 0.1,
               -2.0 * math.sin(index * math.pi / 40.0),
               2.0 * (1.0 - math.cos(index * math.pi / 40.0)),
               -index * math.pi / 40.0,
               -1.0,
               -0.5)
        for index in range(21)
    ]
    forward = [
        sample(2.1 + index * 0.1, -2.0 + index * 0.2,
               2.0 * (1.0 - index / 50.0),
               -math.pi / 2.0 + min(index * 0.08, math.pi / 2.0),
               0.25)
        for index in range(51)
    ]
    passing = reverse + forward

    report = obstacle_report(
        passing,
        goal=(8.0, 0.0),
        obstacle=(1.5, 0.0),
    )
    assert report['passed']

    loop = reverse + [
        sample(2.1 + index * 0.1,
               1.5 + 3.5 * math.cos(index * 0.08),
               3.5 * math.sin(index * 0.08),
               index * 0.08,
               0.25,
               0.08)
        for index in range(100)
    ]
    failed = obstacle_report(
        loop,
        goal=(8.0, 0.0),
        obstacle=(1.5, 0.0),
    )
    assert not failed['passed']
    assert not failed['checks']['no_forward_loop']
