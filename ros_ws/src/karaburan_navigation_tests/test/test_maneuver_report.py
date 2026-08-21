import json
import math
import xml.etree.ElementTree as ElementTree

from karaburan_navigation_tests.maneuver_report import generate_report, main


def write_result(root, scenario, passed):
    checks = {'goal_reached': passed, 'path_efficiency': True}
    result = {
        'scenario': scenario,
        'passed': passed,
        'checks': checks,
        'metrics': {'travelled': 3.25},
        'samples': [
            {
                'time': float(index),
                'x': float(index),
                'y': 0.1 * index,
                'yaw': 0.0,
                'linear': 0.25,
                'angular': 0.0,
            }
            for index in range(4)
        ],
        'reference_path': [{'x': 0.0, 'y': 0.0}, {'x': 3.0, 'y': 0.0}],
        'markers': [],
        'duration_seconds': 2.5,
    }
    (root / f'{scenario}.json').write_text(json.dumps(result))


def test_failed_scenario_produces_red_junit_and_graphical_report(tmp_path):
    write_result(tmp_path, 'actuator_straight', False)
    result_path = tmp_path / 'actuator_straight.json'
    result = json.loads(result_path.read_text())
    result['metrics']['radius'] = math.inf
    result_path.write_text(json.dumps(result))
    coloured_log = '\x1b[31m[ERROR] controller failed\x1b[0m\nordinary line\n'
    launch_log = tmp_path / 'actuator_straight.launch.log'
    launch_log.write_text(coloured_log)

    summary = generate_report(tmp_path, ['actuator_straight'])

    assert not summary['passed']
    assert summary['failed_count'] == 1
    suite = ElementTree.parse(tmp_path / 'junit.xml').getroot()
    assert suite.attrib['failures'] == '1'
    assert suite.find('./testcase/failure') is not None
    document = (tmp_path / 'report.html').read_text()
    assert 'XY trajectory' in document
    assert 'Command velocity over simulation time' in document
    assert 'controller failed' in document
    assert 'Infinity' not in document
    assert '\x1b[' not in launch_log.read_text()
    assert main([
        '--report-root', str(tmp_path),
        '--scenario', 'actuator_straight',
    ]) == 1


def test_missing_scenario_result_is_a_failure(tmp_path):
    summary = generate_report(tmp_path, ['follow_straight'])

    assert not summary['passed']
    assert summary['scenarios'][0]['failed_checks'] == [
        'scenario_result_written']
    suite = ElementTree.parse(tmp_path / 'junit.xml').getroot()
    assert suite.attrib['failures'] == '1'


def test_passing_scenario_produces_green_junit(tmp_path):
    write_result(tmp_path, 'actuator_turn_left', True)

    summary = generate_report(tmp_path, ['actuator_turn_left'])

    assert summary['passed']
    suite = ElementTree.parse(tmp_path / 'junit.xml').getroot()
    assert suite.attrib['failures'] == '0'
    assert suite.find('./testcase/failure') is None
    assert main([
        '--report-root', str(tmp_path),
        '--scenario', 'actuator_turn_left',
    ]) == 0
