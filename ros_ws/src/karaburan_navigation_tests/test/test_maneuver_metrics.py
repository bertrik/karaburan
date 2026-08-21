import math

from karaburan_navigation_tests.maneuver_metrics import (
    obstacle_report,
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
