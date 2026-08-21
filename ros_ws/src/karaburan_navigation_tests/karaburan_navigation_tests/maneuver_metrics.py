"""Deterministic metrics for short boat-manoeuvre acceptance tests."""

import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def path_length(samples):
    """Return travelled distance through x/y trace samples."""
    return sum(
        math.hypot(current['x'] - previous['x'],
                   current['y'] - previous['y'])
        for previous, current in zip(samples, samples[1:])
    )


def cumulative_yaw(samples):
    """Return accumulated absolute yaw change."""
    return sum(
        abs(normalize_angle(current['yaw'] - previous['yaw']))
        for previous, current in zip(samples, samples[1:])
    )


def direction_segments(samples, threshold=0.01):
    """Return non-stationary forward/reverse segments as -1 or 1."""
    segments = []
    for sample in samples:
        velocity = sample['linear']
        direction = 1 if velocity > threshold else -1 if velocity < -threshold else 0
        if direction and (not segments or segments[-1] != direction):
            segments.append(direction)
    return segments


def arc_metrics(samples, direction=-1):
    """Measure path length, heading change, and effective radius of an arc."""
    selected = [
        sample for sample in samples
        if sample['linear'] * direction > 0.01
    ]
    distance = path_length(selected)
    heading = cumulative_yaw(selected)
    radius = distance / heading if heading > 1e-6 else math.inf
    return {
        'distance': distance,
        'heading_change': heading,
        'radius': radius,
        'sample_count': len(selected),
    }


def straight_report(samples, target_distance=3.0):
    """Evaluate an open-loop or FollowPath straight-line trace."""
    if len(samples) < 2:
        return _report(False, {'samples': 'at least two samples required'})
    start = samples[0]
    end = samples[-1]
    dx = end['x'] - start['x']
    dy = end['y'] - start['y']
    forward = math.cos(start['yaw']) * dx + math.sin(start['yaw']) * dy
    lateral = -math.sin(start['yaw']) * dx + math.cos(start['yaw']) * dy
    heading_error = abs(normalize_angle(end['yaw'] - start['yaw']))
    travelled = path_length(samples)
    checks = {
        'forward_progress': forward >= target_distance - 0.25,
        'lateral_error': abs(lateral) <= 0.25,
        'heading_error': heading_error <= math.radians(5.0),
        'path_efficiency': travelled <= target_distance * 1.15,
    }
    return _report(all(checks.values()), checks, {
        'forward': forward,
        'lateral': lateral,
        'heading_error': heading_error,
        'travelled': travelled,
    })


def turn_report(samples, expected_sign, minimum_heading=math.radians(45.0)):
    """Evaluate the sign and magnitude of an open-loop turn."""
    if len(samples) < 2:
        return _report(False, {'samples': 'at least two samples required'})
    heading = sum(
        normalize_angle(current['yaw'] - previous['yaw'])
        for previous, current in zip(samples, samples[1:])
    )
    checks = {
        'turn_sign': heading * expected_sign > 0.0,
        'heading_change': abs(heading) >= minimum_heading,
    }
    return _report(all(checks.values()), checks, {'heading_change': heading})


def planner_direct_report(points, start, goal):
    """Require a direct, monotonic plan for an aligned start and goal."""
    if len(points) < 2:
        return _report(False, {'plan_available': False})
    direct_x = goal[0] - start[0]
    direct_y = goal[1] - start[1]
    direct_distance = math.hypot(direct_x, direct_y)
    if direct_distance < 1e-6:
        return _report(False, {'direct_distance': False})

    direction_x = direct_x / direct_distance
    direction_y = direct_y / direct_distance
    progress = [
        (point['x'] - start[0]) * direction_x
        + (point['y'] - start[1]) * direction_y
        for point in points
    ]
    cross_track = [
        abs((point['x'] - start[0]) * direction_y
            - (point['y'] - start[1]) * direction_x)
        for point in points
    ]
    backwards_steps = sum(
        later < earlier - 0.02
        for earlier, later in zip(progress, progress[1:])
    )
    cusp_count = _cusp_count(points)
    planned_distance = path_length(points)
    length_excess = planned_distance - direct_distance
    end_error = math.hypot(
        points[-1]['x'] - goal[0], points[-1]['y'] - goal[1])
    checks = {
        'plan_available': True,
        'planner_path_length': length_excess <= 0.05,
        'planner_cross_track': max(cross_track) <= 0.50,
        'planner_monotonic': backwards_steps == 0,
        'planner_no_cusps': cusp_count == 0,
        'planner_goal_reached': end_error <= 0.10,
    }
    return _report(all(checks.values()), checks, {
        'direct_distance': direct_distance,
        'planned_distance': planned_distance,
        'length_excess': length_excess,
        'length_ratio': planned_distance / direct_distance,
        'max_cross_track': max(cross_track),
        'backwards_steps': backwards_steps,
        'cusp_count': cusp_count,
        'end_error': end_error,
    })


def _cusp_count(points):
    segments = []
    for previous, current in zip(points, points[1:]):
        dx = current['x'] - previous['x']
        dy = current['y'] - previous['y']
        length = math.hypot(dx, dy)
        if length > 1e-3:
            segments.append((dx / length, dy / length))
    return sum(
        previous[0] * current[0] + previous[1] * current[1] < 0.0
        for previous, current in zip(segments, segments[1:])
    )


def obstacle_report(samples, goal, obstacle, planned_length=None):
    """Evaluate the complete one-reverse obstacle manoeuvre."""
    if len(samples) < 2:
        return _report(False, {'samples': 'at least two samples required'})
    segments = direction_segments(samples)
    reverse = arc_metrics(samples, direction=-1)
    forward_start = next(
        (
            index for index, sample in enumerate(samples)
            if sample['linear'] > 0.01
            and any(item['linear'] < -0.01 for item in samples[:index])
        ),
        len(samples),
    )
    forward_samples = samples[forward_start:]
    goal_distances = [
        math.hypot(sample['x'] - goal[0], sample['y'] - goal[1])
        for sample in forward_samples
    ]
    obstacle_distances = [
        math.hypot(sample['x'] - obstacle[0], sample['y'] - obstacle[1])
        for sample in forward_samples
    ]
    forward_yaw = cumulative_yaw(forward_samples)
    returned_to_obstacle = _returned_to_obstacle(obstacle_distances)
    decreasing_ratio = _decreasing_ratio(goal_distances)
    travelled = path_length(samples)
    final_goal_distance = (
        goal_distances[-1] if goal_distances else math.inf)
    checks = {
        'one_reverse_then_forward': segments == [-1, 1],
        'reverse_radius': abs(reverse['radius'] - 2.0) <= 0.20,
        'reverse_heading': abs(
            reverse['heading_change'] - math.pi / 2.0
        ) <= math.radians(3.0),
        'no_obstacle_return': not returned_to_obstacle,
        'goal_distance_decreases': decreasing_ratio >= 0.70,
        'no_forward_loop': forward_yaw <= math.radians(150.0),
        'goal_reached': final_goal_distance <= 0.30,
    }
    if planned_length is not None:
        checks['path_efficiency'] = travelled <= planned_length * 1.15
    return _report(all(checks.values()), checks, {
        'segments': segments,
        'reverse': reverse,
        'forward_cumulative_yaw': forward_yaw,
        'goal_decreasing_ratio': decreasing_ratio,
        'travelled': travelled,
        'planned_length': planned_length,
        'final_goal_distance': final_goal_distance,
    })


def _decreasing_ratio(values, stride=5):
    if len(values) <= stride:
        return 0.0
    comparisons = [
        later < earlier
        for earlier, later in zip(values, values[stride:])
    ]
    return sum(comparisons) / len(comparisons)


def _returned_to_obstacle(distances):
    was_near = False
    moved_clear = False
    for distance in distances:
        if distance < 3.0:
            if moved_clear:
                return True
            was_near = True
        elif was_near:
            moved_clear = True
    return False


def _report(passed, checks, metrics=None):
    return {
        'passed': passed,
        'checks': checks,
        'metrics': metrics or {},
    }
