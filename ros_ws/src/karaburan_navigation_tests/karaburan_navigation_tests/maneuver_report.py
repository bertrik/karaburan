"""Create concise JUnit and static manoeuvre diagnostic artifacts."""

import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ElementTree

from karaburan_navigation_tests.junit_html_report import render_html


ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
DIAGNOSTIC_LINE = re.compile(
    r'\b(error|warn(?:ing)?|fail(?:ed|ure)?|reject(?:ed)?|abort(?:ed)?|'
    r'timeout|exception|traceback)\b', re.IGNORECASE)
SCENARIOS = (
    'actuator_straight', 'actuator_turn_left', 'actuator_turn_right',
    'follow_straight', 'follow_arc_left', 'follow_arc_right',
    'planner_direct',
    'planner_island',
    'island_navigation',
    'obstacle_port', 'obstacle_starboard',
)
CHECK_REQUIREMENTS = {
    'action_succeeded': 'the ROS action must finish successfully',
    'action_path_endpoint': 'the boat must finish within 0.30 metres of the path endpoint',
    'controller_cross_track': 'cross-track error must stay within 0.35 metres',
    'controller_forward_only': 'ordinary path tracking must not reverse',
    'controller_path_efficiency': 'travel must be at most 125% of path length',
    'forward_progress': 'forward progress must reach the scenario target',
    'goal_reached': 'the controller must report a reached goal',
    'heading_change': 'absolute heading change must be at least 45 degrees',
    'heading_error': 'absolute heading error must be at most 5 degrees',
    'heel_angle': 'absolute simulated roll must stay within 8 degrees',
    'lateral_error': 'absolute lateral error must be at most 0.25 metres',
    'goal_distance_decreases': (
        'goal distance must decrease in at least 70% of comparisons'),
    'no_forward_loop': 'forward cumulative yaw must be at most 150 degrees',
    'no_obstacle_return': 'the boat must not return after clearing the obstacle',
    'one_reverse_then_forward': 'drive direction must change exactly once',
    'path_efficiency': 'travel must be at most 115% of the reference path',
    'plan_available': 'the planner must return at least two path poses',
    'planner_cross_track': 'the plan must stay within 0.50 metres of the line',
    'planner_goal_reached': 'the plan endpoint must be within 0.10 metres',
    'island_avoided': 'the planned hull centre must clear the island by 0.25 metres',
    'planner_monotonic': 'the plan must not move away from the goal',
    'planner_no_cusps': 'the plan must not contain a direction cusp',
    'planner_one_side': 'the route must pass the island on one consistent side',
    'planner_path_length': (
        'the plan may be at most 0.05 metres longer than the direct line'),
    'planner_reasonable_length': 'the island route must be at most 135% of direct distance',
    'planner_smooth': 'successive path headings must change by at most 15 degrees',
    'reverse_heading': 'reverse turn must be 90 +/- 3 degrees',
    'reverse_radius': 'reverse turn radius must be 2.0 +/- 0.2 metres',
    'scenario_result_readable': 'the scenario JSON must be valid',
    'scenario_result_written': 'the scenario must write its JSON result',
    'runner_error': 'the scenario runner must finish without an exception',
    'samples': 'at least two pose samples must be recorded',
    'turn_sign': 'heading change must match the requested steering direction',
}


def scenario_layer(scenario):
    """Separate propulsion, local control, and global planning scenarios."""
    if scenario.startswith('actuator_'):
        return 'actuator'
    if scenario.startswith('follow_'):
        return 'controller'
    return 'planner'


def strip_ansi(value):
    """Remove terminal control sequences from text."""
    return ANSI_ESCAPE.sub('', value).replace('\r', '')


def clean_launch_log(path, maximum_diagnostics=30):
    """Remove colour in place and return a short list of relevant lines."""
    if not path.exists():
        return []
    cleaned = strip_ansi(path.read_text(errors='replace'))
    path.write_text(cleaned)
    diagnostics = []
    signatures = set()
    for line in cleaned.splitlines():
        compact = line.strip()
        signature = re.sub(r'\d+(?:\.\d+)?', '#', compact)
        if (compact and DIAGNOSTIC_LINE.search(compact)
                and signature not in signatures):
            diagnostics.append(compact)
            signatures.add(signature)
    return diagnostics[-maximum_diagnostics:]


def _failed_checks(report):
    return [name for name, passed in report.get('checks', {}).items()
            if passed is not True]


def _load_report(report_root, scenario):
    path = report_root / f'{scenario}.json'
    if not path.exists():
        return {
            'scenario': scenario, 'passed': False,
            'checks': {'scenario_result_written': False},
            'metrics': {}, 'samples': [],
            'error': 'The scenario did not produce a JSON result.',
        }
    try:
        report = json.loads(path.read_text())
    except (json.JSONDecodeError, OSError) as error:
        return {
            'scenario': scenario, 'passed': False,
            'checks': {'scenario_result_readable': False},
            'metrics': {}, 'samples': [], 'error': str(error),
        }
    report['scenario'] = scenario
    report['passed'] = report.get('passed') is True
    return report


def _flatten(value, prefix=''):
    result = []
    for name, item in value.items():
        key = f'{prefix}{name}'
        if isinstance(item, dict):
            result.extend(_flatten(item, key + '.'))
        else:
            result.append((key, item))
    return result


def _format_value(value):
    if isinstance(value, float):
        return f'{value:.6g}' if math.isfinite(value) else str(value)
    return json.dumps(value)


def _failure_text(report):
    lines = []
    if report.get('error'):
        lines.append('Runner error: ' + report['error'])
    lines.append('Failed acceptance checks:')
    lines.extend(
        f'  {name}: {CHECK_REQUIREMENTS.get(name, "acceptance check must pass")}'
        for name in _failed_checks(report))
    if report.get('metrics'):
        lines.extend(['', 'Observed metrics:'])
        lines.extend(
            f'  {name}: {_format_value(value)}'
            for name, value in _flatten(report['metrics']))
    lines.extend([
        '', 'Artifacts in this report directory:',
        f"  trajectory: {report['scenario']}.svg",
        f"  complete trace: {report['scenario']}.json",
        f"  launch log: {report['scenario']}.launch.log",
        f"  runner log: {report['scenario']}.runner.log",
    ])
    return '\n'.join(lines)


def _write_junit(report_root, reports):
    failures = sum(not report['passed'] for report in reports)
    duration = sum(float(report.get('duration_seconds', 0.0))
                   for report in reports)
    suite = ElementTree.Element('testsuite', {
        'name': 'karaburan_navigation_maneuvers',
        'tests': str(len(reports)), 'failures': str(failures),
        'errors': '0', 'skipped': '0', 'time': f'{duration:.3f}',
    })
    for report in reports:
        case = ElementTree.SubElement(suite, 'testcase', {
            'classname': (
                'karaburan_navigation_tests.'
                + scenario_layer(report['scenario'])),
            'name': report['scenario'],
            'time': f"{float(report.get('duration_seconds', 0.0)):.3f}",
        })
        details = _failure_text(report)
        if not report['passed']:
            failed = ', '.join(_failed_checks(report))
            message = report.get('error') or f'Failed checks: {failed}'
            failure = ElementTree.SubElement(case, 'failure', {
                'message': message, 'type': 'ManeuverAcceptanceFailure'})
            failure.text = details
        output = ElementTree.SubElement(case, 'system-out')
        if report['passed']:
            output.text = (
                f"Trajectory: {report['scenario']}.svg\n"
                f"Complete trace: {report['scenario']}.json")
        else:
            diagnostic_text = '\n'.join(report.get('diagnostics', []))
            output.text = 'Relevant log lines:\n' + diagnostic_text
    ElementTree.ElementTree(suite).write(
        report_root / 'junit.xml', encoding='utf-8', xml_declaration=True)


def _bounds(points):
    if not points:
        return -1.0, 1.0, -1.0, 1.0
    xs = [float(point['x']) for point in points]
    ys = [float(point['y']) for point in points]
    margin_x = max((max(xs) - min(xs)) * 0.08, 0.25)
    margin_y = max((max(ys) - min(ys)) * 0.08, 0.25)
    return (min(xs) - margin_x, max(xs) + margin_x,
            min(ys) - margin_y, max(ys) + margin_y)


def _svg_points(points, bounds):
    min_x, max_x, min_y, max_y = bounds
    width, height, padding = 720.0, 400.0, 45.0
    scale_x = (width - 2 * padding) / (max_x - min_x)
    scale_y = (height - 2 * padding) / (max_y - min_y)
    return ' '.join(
        f"{padding + (point['x'] - min_x) * scale_x:.1f},"
        f"{height - padding - (point['y'] - min_y) * scale_y:.1f}"
        for point in points)


def _write_trajectory_svg(report_root, report):
    samples = report.get('samples', [])
    reference = report.get('reference_path', [])
    markers = report.get('markers', [])
    marker_extents = []
    for marker in markers:
        radius = float(marker.get('radius', 0.0))
        marker_extents.extend([
            {'x': marker['x'] - radius, 'y': marker['y'] - radius},
            {'x': marker['x'] + radius, 'y': marker['y'] + radius},
        ])
    bounds = _bounds(samples + reference + markers + marker_extents)
    actual_points = _svg_points(samples, bounds)
    reference_points = _svg_points(reference, bounds)
    status = 'PASS' if report['passed'] else 'FAIL'
    status_colour = '#16834b' if report['passed'] else '#c43d35'
    failed = ', '.join(_failed_checks(report)) or '-'
    marker_elements = []
    min_x, max_x, min_y, max_y = bounds
    scale_x = 630.0 / (max_x - min_x)
    scale_y = 310.0 / (max_y - min_y)
    for marker in markers:
        x = 45.0 + (marker['x'] - min_x) * scale_x
        y = 355.0 - (marker['y'] - min_y) * scale_y
        radius = float(marker.get('radius', 0.10)) * min(scale_x, scale_y)
        marker_elements.extend([
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{max(radius, 4.0):.1f}" '
            'fill="#ef444455" stroke="#dc2626" stroke-width="2"/>',
            f'<text x="{x + 6:.1f}" y="{y - 6:.1f}" '
            'font-family="sans-serif" font-size="12" fill="#991b1b">'
            f'{marker.get("label", "marker")}</text>',
        ])
    svg = '\n'.join([
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 440">',
        '<rect width="720" height="440" fill="white"/>',
        f'<text x="45" y="25" font-family="sans-serif" font-size="18" '
        f'fill="{status_colour}">{status} - {report["scenario"]}</text>',
        '<rect x="45" y="45" width="630" height="310" fill="#f8fafc" '
        'stroke="#cbd5e1"/>',
        f'<polyline points="{reference_points}" fill="none" '
        'stroke="#8b5cf6" stroke-width="2" stroke-dasharray="8 5"/>',
        f'<polyline points="{actual_points}" fill="none" '
        'stroke="#1677b8" stroke-width="3"/>',
        *marker_elements,
        '<text x="45" y="385" font-family="sans-serif" font-size="14" '
        'fill="#1677b8">actual trajectory</text>',
        '<text x="220" y="385" font-family="sans-serif" font-size="14" '
        'fill="#8b5cf6">reference path</text>',
        '<text x="45" y="415" font-family="monospace" font-size="13" '
        f'fill="#334155">Failed checks: {failed}</text>',
        '</svg>',
    ])
    (report_root / f"{report['scenario']}.svg").write_text(svg)


def generate_report(report_root, scenarios):
    """Generate suite artifacts and return the structured summary."""
    report_root.mkdir(parents=True, exist_ok=True)
    reports = []
    for scenario in scenarios:
        report = _load_report(report_root, scenario)
        report['diagnostics'] = (
            clean_launch_log(report_root / f'{scenario}.launch.log')
            + clean_launch_log(report_root / f'{scenario}.runner.log'))[-30:]
        reports.append(report)
        _write_trajectory_svg(report_root, report)
    passed = sum(report['passed'] for report in reports)
    summary = {
        'passed': passed == len(reports), 'scenario_count': len(reports),
        'passed_count': passed, 'failed_count': len(reports) - passed,
        'scenarios': [{
            'name': report['scenario'], 'passed': report['passed'],
            'layer': scenario_layer(report['scenario']),
            'failed_checks': _failed_checks(report),
            'error': report.get('error'),
        } for report in reports],
    }
    (report_root / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n')
    _write_junit(report_root, reports)
    render_html(report_root / 'junit.xml', report_root / 'report.html')
    return summary


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--report-root', type=Path, required=True)
    parser.add_argument('--scenario', action='append', choices=SCENARIOS)
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    summary = generate_report(
        options.report_root, options.scenario or list(SCENARIOS))
    print('')
    print('Navigation manoeuvre test summary')
    print('RESULT  LAYER       SCENARIO                    FAILED CHECKS')
    for result in summary['scenarios']:
        status = 'PASS' if result['passed'] else 'FAIL'
        detail = ', '.join(result['failed_checks']) or '-'
        if result['error']:
            detail = result['error']
        print(
            f"{status:<6}  {result['layer']:<10}  "
            f"{result['name']:<28} {detail}")
    print('')
    print(f"{summary['failed_count']} failed, "
          f"{summary['passed_count']} passed")
    print(f"HTML:  {options.report_root / 'report.html'}")
    print(f"JUnit: {options.report_root / 'junit.xml'}")
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
