import xml.etree.ElementTree as ElementTree

from karaburan_navigation_tests.junit_html_report import merge_junit


def _write_suite(path, name, failed=False):
    suite = ElementTree.Element('testsuite', {'name': name})
    case = ElementTree.SubElement(suite, 'testcase', {
        'classname': 'example', 'name': 'does_work', 'time': '0.25'})
    if failed:
        ElementTree.SubElement(
            case, 'failure', {'message': 'expected failure'})
    ElementTree.ElementTree(suite).write(path)


def test_merge_junit_preserves_cases_and_failure_details(tmp_path):
    first = tmp_path / 'first.xml'
    second = tmp_path / 'second.xml'
    output = tmp_path / 'combined.xml'
    _write_suite(first, 'actuator', failed=True)
    _write_suite(second, 'static')

    merge_junit([first, second], output)

    suite = ElementTree.parse(output).getroot()
    assert suite.attrib['tests'] == '2'
    assert suite.attrib['failures'] == '1'
    assert suite.attrib['time'] == '0.500'
    assert [case.attrib['classname'] for case in suite] == [
        'actuator.example', 'static.example']
    assert suite.find('./testcase/failure').attrib['message'] == (
        'expected failure')


def test_merge_junit_turns_missing_input_into_red_test(tmp_path):
    output = tmp_path / 'combined.xml'

    merge_junit([tmp_path / 'missing.xml'], output)

    suite = ElementTree.parse(output).getroot()
    assert suite.attrib['tests'] == '1'
    assert suite.attrib['failures'] == '1'
    failure = suite.find('./testcase/failure')
    assert failure.attrib['type'] == 'MissingTestArtifact'
    assert 'colcon.log' in failure.text
