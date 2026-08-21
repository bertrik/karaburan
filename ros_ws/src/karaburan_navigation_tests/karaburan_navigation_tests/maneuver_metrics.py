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
        'heel_angle': max(
            abs(sample.get('roll', 0.0)) for sample in samples
        ) <= math.radians(8.0),
    }
    return _report(all(checks.values()), checks, {
        'heading_change': heading,
        'maximum_roll': max(
            abs(sample.get('roll', 0.0)) for sample in samples),
        'maximum_pitch': max(
            abs(sample.get('pitch', 0.0)) for sample in samples),
    })


def path_tracking_report(samples, reference, expected_sign=None):
    """Evaluate whether a forward controller trace follows its given path."""
    if len(samples) < 2 or len(reference) < 2:
        return _report(False, {'samples': 'trace and path are required'})
    cross_track = [
        _distance_to_path(sample, reference) for sample in samples]
    endpoint = reference[-1]
    final_distance = math.hypot(
        samples[-1]['x'] - endpoint['x'],
        samples[-1]['y'] - endpoint['y'],
    )
    heading = sum(
        normalize_angle(current['yaw'] - previous['yaw'])
        for previous, current in zip(samples, samples[1:])
    )
    travelled = path_length(samples)
    reference_length = path_length(reference)
    checks = {
        'action_path_endpoint': final_distance <= 0.30,
        'controller_cross_track': max(cross_track) <= 0.35,
        'controller_forward_only': all(
            sample['linear'] >= -0.01 for sample in samples),
        'controller_path_efficiency': (
            travelled <= reference_length * 1.25),
        'heel_angle': max(
            abs(sample.get('roll', 0.0)) for sample in samples
        ) <= math.radians(8.0),
    }
    if expected_sign is not None:
        checks['turn_sign'] = heading * expected_sign > 0.0
    return _report(all(checks.values()), checks, {
        'final_path_distance': final_distance,
        'maximum_cross_track': max(cross_track),
        'travelled': travelled,
        'reference_length': reference_length,
        'heading_change': heading,
        'maximum_roll': max(
            abs(sample.get('roll', 0.0)) for sample in samples),
        'maximum_pitch': max(
            abs(sample.get('pitch', 0.0)) for sample in samples),
    })


def _distance_to_path(point, path):
    return min(
        _distance_to_segment(point, start, end)
        for start, end in zip(path, path[1:])
    )


def _distance_to_segment(point, start, end):
    dx = end['x'] - start['x']
    dy = end['y'] - start['y']
    length_squared = dx * dx + dy * dy
    if length_squared < 1e-12:
        return math.hypot(point['x'] - start['x'], point['y'] - start['y'])
    projection = max(0.0, min(1.0, (
        (point['x'] - start['x']) * dx
        + (point['y'] - start['y']) * dy
    ) / length_squared))
    closest_x = start['x'] + projection * dx
    closest_y = start['y'] + projection * dy
    return math.hypot(point['x'] - closest_x, point['y'] - closest_y)


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


def planner_island_report(points, start, goal, island, radius):
    """Require one smooth, forward route around a known island."""
    if len(points) < 2:
        return _report(False, {'plan_available': False})
    direct_x = goal[0] - start[0]
    direct_y = goal[1] - start[1]
    direct_distance = math.hypot(direct_x, direct_y)
    direction_x = direct_x / direct_distance
    direction_y = direct_y / direct_distance
    progress = [
        (point['x'] - start[0]) * direction_x
        + (point['y'] - start[1]) * direction_y
        for point in points
    ]
    lateral = [
        (point['x'] - start[0]) * direction_y
        - (point['y'] - start[1]) * direction_x
        for point in points
    ]
    significant_sides = {
        1 if value > 0.10 else -1
        for value in lateral if abs(value) > 0.10
    }
    island_clearance = min(
        math.hypot(point['x'] - island[0], point['y'] - island[1])
        for point in points
    )
    heading_steps = _path_heading_steps(points)
    planned_distance = path_length(points)
    backwards_steps = sum(
        later < earlier - 0.02
        for earlier, later in zip(progress, progress[1:])
    )
    end_error = math.hypot(
        points[-1]['x'] - goal[0], points[-1]['y'] - goal[1])
    checks = {
        'plan_available': True,
        'island_avoided': island_clearance >= radius + 0.25,
        'planner_one_side': len(significant_sides) == 1,
        'planner_monotonic': backwards_steps == 0,
        'planner_no_cusps': _cusp_count(points) == 0,
        'planner_smooth': (
            not heading_steps
            or max(abs(step) for step in heading_steps) <= math.radians(15.0)),
        'planner_reasonable_length': planned_distance <= direct_distance * 1.35,
        'planner_goal_reached': end_error <= 0.10,
    }
    return _report(all(checks.values()), checks, {
        'direct_distance': direct_distance,
        'planned_distance': planned_distance,
        'length_ratio': planned_distance / direct_distance,
        'minimum_island_clearance': island_clearance,
        'maximum_lateral_offset': max(abs(value) for value in lateral),
        'backwards_steps': backwards_steps,
        'cusp_count': _cusp_count(points),
        'maximum_heading_step': (
            max(abs(step) for step in heading_steps) if heading_steps else 0.0),
        'end_error': end_error,
    })


def _path_heading_steps(points):
    headings = []
    for previous, current in zip(points, points[1:]):
        dx = current['x'] - previous['x']
        dy = current['y'] - previous['y']
        if math.hypot(dx, dy) > 1e-3:
            headings.append(math.atan2(dy, dx))
    return [
        normalize_angle(current - previous)
        for previous, current in zip(headings, headings[1:])
    ]


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


def open_obstacle_report(samples, goal, obstacle, planned_length=None):
    """Require a forward detour around an isolated open-water obstacle."""
    if len(samples) < 2:
        return _report(False, {'samples': 'at least two samples required'})
    segments = direction_segments(samples)
    goal_distances = [
        math.hypot(sample['x'] - goal[0], sample['y'] - goal[1])
        for sample in samples
    ]
    obstacle_distances = [
        math.hypot(sample['x'] - obstacle[0], sample['y'] - obstacle[1])
        for sample in samples
    ]
    decreasing_ratio = _decreasing_ratio(goal_distances)
    travelled = path_length(samples)
    final_goal_distance = goal_distances[-1]
    checks = {
        'forward_only': segments == [1],
        'obstacle_clearance': min(obstacle_distances) >= 0.70,
        'goal_distance_decreases': decreasing_ratio >= 0.70,
        'no_forward_loop': cumulative_yaw(samples) <= math.radians(180.0),
        'goal_reached': final_goal_distance <= 0.30,
    }
    if planned_length is not None:
        checks['path_efficiency'] = travelled <= planned_length * 1.25
    return _report(all(checks.values()), checks, {
        'segments': segments,
        'minimum_obstacle_clearance': min(obstacle_distances),
        'cumulative_yaw': cumulative_yaw(samples),
        'goal_decreasing_ratio': decreasing_ratio,
        'travelled': travelled,
        'planned_length': planned_length,
        'final_goal_distance': final_goal_distance,
    })


def harbour_departure_report(samples, expected_maneuver):
    """Check the initial reverse departure selected inside a berth."""
    if len(samples) < 2:
        return _report(False, {'samples': 'at least two samples required'})
    reverse_samples = [sample for sample in samples if sample['linear'] < -0.01]
    if len(reverse_samples) < 2:
        return _report(False, {'reverse_started': False})

    segments = direction_segments(samples)
    reverse = arc_metrics(samples, direction=-1)
    signed_heading = sum(
        normalize_angle(current['yaw'] - previous['yaw'])
        for previous, current in zip(reverse_samples, reverse_samples[1:])
    )
    start = reverse_samples[0]
    end = reverse_samples[-1]
    dx = end['x'] - start['x']
    dy = end['y'] - start['y']
    lateral = -math.sin(start['yaw']) * dx + math.cos(start['yaw']) * dy
    expected_sign = {
        'stern_port': -1.0,
        'stern_starboard': 1.0,
    }.get(expected_maneuver, 0.0)
    moving_samples = [
        sample for sample in samples if abs(sample['linear']) > 0.01]
    checks = {
        'reverse_only': segments == [-1],
        'single_reverse_maneuver': len(segments) == 1,
        'heel_angle': max(
            abs(sample.get('roll', 0.0)) for sample in moving_samples
        ) <= math.radians(8.0),
    }
    if expected_maneuver == 'straight':
        checks.update({
            'reverse_distance': reverse['distance'] >= 2.50,
            'reverse_straight': abs(signed_heading) <= math.radians(8.0),
            'reverse_lateral_error': abs(lateral) <= 0.35,
        })
    else:
        checks.update({
            'reverse_radius': abs(reverse['radius'] - 2.0) <= 0.25,
            'reverse_heading': abs(
                abs(signed_heading) - math.pi / 2.0
            ) <= math.radians(5.0),
            'reverse_turn_sign': signed_heading * expected_sign > 0.0,
            'reverse_stern_side': lateral * expected_sign < -0.50,
        })
    return _report(all(checks.values()), checks, {
        'expected_maneuver': expected_maneuver,
        'segments': segments,
        'reverse': reverse,
        'signed_heading_change': signed_heading,
        'stern_lateral_displacement': lateral,
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
