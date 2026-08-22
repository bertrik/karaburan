"""Merge JUnit files and render a static report with junit2html."""

import argparse
import base64
from copy import deepcopy
import html
from pathlib import Path
import re
import xml.etree.ElementTree as ElementTree

from junit2htmlreport.runner import run as run_junit2html


def _suites(root):
    if root.tag == 'testsuite':
        return [root]
    return list(root.findall('./testsuite'))


def merge_junit(inputs, output, name='Karaburan navigation tests'):
    """Merge test cases from JUnit files into one portable test suite."""
    merged = ElementTree.Element('testsuite', {'name': name})
    for source in inputs:
        if not source.exists():
            case = ElementTree.SubElement(merged, 'testcase', {
                'classname': 'test_artifacts',
                'name': f'{source.name}_was_created',
                'time': '0.0',
            })
            failure = ElementTree.SubElement(case, 'failure', {
                'message': f'Missing JUnit file: {source}',
                'type': 'MissingTestArtifact',
            })
            failure.text = (
                'The test command did not create its expected JUnit file. '
                'Inspect colcon.log in this report directory.')
            continue
        root = ElementTree.parse(source).getroot()
        for suite in _suites(root):
            suite_name = suite.get('name', Path(source).stem)
            for original in suite.findall('.//testcase'):
                case = deepcopy(original)
                class_name = case.get('classname', '')
                case.set('classname', '.'.join(
                    item for item in (suite_name, class_name) if item))
                merged.append(case)
    cases = merged.findall('./testcase')
    merged.set('tests', str(len(cases)))
    merged.set('failures', str(sum(
        case.find('failure') is not None for case in cases)))
    merged.set('errors', str(sum(
        case.find('error') is not None for case in cases)))
    merged.set('skipped', str(sum(
        case.find('skipped') is not None for case in cases)))
    merged.set('time', f'{sum(float(case.get("time", 0.0)) for case in cases):.3f}')
    ElementTree.ElementTree(merged).write(
        output, encoding='utf-8', xml_declaration=True)


def render_html(junit_path, html_path):
    """Render one self-contained report with all SVG evidence embedded."""
    run_junit2html([str(junit_path), str(html_path)])
    document = html_path.read_text()
    evidence = _evidence_gallery(html_path.parent)
    if evidence:
        closing_body = document.lower().rfind('</body>')
        if closing_body >= 0:
            document = (
                document[:closing_body] + evidence + document[closing_body:])
        else:
            document += evidence
        html_path.write_text(document)


def _evidence_gallery(report_root):
    cards = []
    for svg_path in sorted(report_root.glob('*.svg')):
        svg = svg_path.read_bytes()
        match = re.search(
            rb'>(PASS|FAIL) - ([^<]+)</text>', svg, re.IGNORECASE)
        status = match.group(1).decode().upper() if match else 'EVIDENCE'
        scenario = (
            match.group(2).decode(errors='replace')
            if match else svg_path.stem)
        encoded = base64.b64encode(svg).decode('ascii')
        cards.append(
            '<figure style="margin:0 0 2rem 0">'
            f'<figcaption><strong>{html.escape(scenario)}</strong> '
            f'&mdash; {html.escape(status)}</figcaption>'
            f'<img src="data:image/svg+xml;base64,{encoded}" '
            f'alt="Evidence for {html.escape(scenario)} ({status})" '
            'style="display:block;max-width:100%;height:auto;'
            'border:1px solid #cbd5e1;margin-top:.5rem">'
            f'<div><a href="{html.escape(svg_path.name)}">'
            'Open standalone SVG</a></div></figure>')
    if not cards:
        return ''
    return (
        '<section id="evidence" style="margin:3rem auto;max-width:900px">'
        '<h1>Evidence images</h1>'
        '<p>Plots are included for successful and failed manoeuvre tests.</p>'
        + ''.join(cards) + '</section>')


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, action='append', required=True)
    parser.add_argument('--output-xml', type=Path, required=True)
    parser.add_argument('--output-html', type=Path, required=True)
    parser.add_argument('--name', default='Karaburan navigation tests')
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    merge_junit(options.input, options.output_xml, options.name)
    render_html(options.output_xml, options.output_html)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
