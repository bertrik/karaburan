"""Create concise machine-readable and graphical manoeuvre test reports."""

import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ElementTree

ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
DIAGNOSTIC_LINE = re.compile(
    r'\b(error|warn(?:ing)?|fail(?:ed|ure)?|reject(?:ed)?|abort(?:ed)?|'
    r'timeout|exception|traceback)\b',
    re.IGNORECASE,
)
SCENARIOS = (
    'actuator_straight',
    'actuator_turn_left',
    'actuator_turn_right',
    'follow_straight',
    'follow_arc_left',
    'follow_arc_right',
    'obstacle_port',
    'obstacle_starboard',
)


def strip_ansi(value):
    """Remove terminal control sequences from text."""
    return ANSI_ESCAPE.sub('', value).replace('\r', '')


def clean_launch_log(path, maximum_diagnostics=120):
    """Remove colour in place and return a short list of relevant lines."""
    if not path.exists():
        return []
    cleaned = strip_ansi(path.read_text(errors='replace'))
    path.write_text(cleaned)
    diagnostics = []
    previous = None
    for line in cleaned.splitlines():
        compact = line.strip()
        if compact and DIAGNOSTIC_LINE.search(compact) and compact != previous:
            diagnostics.append(compact)
            previous = compact
    return diagnostics[-maximum_diagnostics:]


def _failed_checks(report):
    return [name for name, passed in report.get('checks', {}).items()
            if passed is not True]


def _load_report(report_root, scenario):
    path = report_root / f'{scenario}.json'
    if not path.exists():
        return {
            'scenario': scenario,
            'passed': False,
            'checks': {'scenario_result_written': False},
            'metrics': {},
            'samples': [],
            'error': 'The scenario did not produce a JSON result.',
        }
    try:
        report = json.loads(path.read_text())
    except (json.JSONDecodeError, OSError) as error:
        return {
            'scenario': scenario,
            'passed': False,
            'checks': {'scenario_result_readable': False},
            'metrics': {},
            'samples': [],
            'error': str(error),
        }
    report['scenario'] = scenario
    report['passed'] = report.get('passed') is True
    return report


def _write_junit(report_root, reports):
    failures = sum(not report['passed'] for report in reports)
    duration = sum(float(report.get('duration_seconds', 0.0))
                   for report in reports)
    suite = ElementTree.Element('testsuite', {
        'name': 'karaburan_navigation_maneuvers',
        'tests': str(len(reports)),
        'failures': str(failures),
        'errors': '0',
        'time': f'{duration:.3f}',
    })
    for report in reports:
        case = ElementTree.SubElement(suite, 'testcase', {
            'classname': 'karaburan_navigation_tests.maneuver',
            'name': report['scenario'],
            'time': f"{float(report.get('duration_seconds', 0.0)):.3f}",
        })
        if not report['passed']:
            failed = _failed_checks(report)
            message = report.get('error') or (
                'Failed checks: ' + ', '.join(failed))
            failure = ElementTree.SubElement(case, 'failure', {
                'message': message,
                'type': 'ManeuverAcceptanceFailure',
            })
            failure.text = json.dumps({
                'failed_checks': failed,
                'metrics': report.get('metrics', {}),
                'error': report.get('error'),
            }, indent=2)
        output = ElementTree.SubElement(case, 'system-out')
        output.text = json.dumps(report.get('metrics', {}), indent=2)
    ElementTree.ElementTree(suite).write(
        report_root / 'junit.xml', encoding='utf-8', xml_declaration=True)


def _downsample(items, limit=600):
    if len(items) <= limit:
        return items
    return [items[round(index * (len(items) - 1) / (limit - 1))]
            for index in range(limit)]


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def _report_for_browser(report):
    return _json_safe({
        'scenario': report['scenario'],
        'passed': report['passed'],
        'checks': report.get('checks', {}),
        'metrics': report.get('metrics', {}),
        'error': report.get('error'),
        'duration_seconds': report.get('duration_seconds'),
        'samples': _downsample(report.get('samples', [])),
        'reference_path': _downsample(report.get('reference_path', [])),
        'markers': report.get('markers', []),
        'diagnostics': report.get('diagnostics', []),
    })


def _write_html(report_root, reports):
    payload = json.dumps(
        [_report_for_browser(report) for report in reports],
        separators=(',', ':'),
    ).replace('</', '<\\/')
    passed = sum(report['passed'] for report in reports)
    failed = len(reports) - passed
    document = f'''<!doctype html><!-- # noqa: E501, Q001 -->
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Karaburan navigation manoeuvre report</title>
<style>
:root {{ color-scheme: light dark; --ok: #16834b; --bad: #c43d35;
  --actual: #1677b8; --reference: #8b5cf6; --reverse: #d97706;
  --grid: color-mix(in srgb, CanvasText 18%, transparent);
  font: 15px/1.45 system-ui, sans-serif; }}
body {{ max-width: 1180px; margin: 0 auto; padding: 24px; background: Canvas; color: CanvasText; }}
h1 {{ margin-bottom: .25rem; }} .summary {{ font-size: 1.15rem; margin: 0 0 1.5rem; }}
.ok {{ color: var(--ok); }} .bad {{ color: var(--bad); }}
table {{ border-collapse: collapse; width: 100%; }} th, td {{ text-align: left; padding: .45rem .6rem; border-bottom: 1px solid var(--grid); }}
section {{ margin: 2rem 0; padding-top: .5rem; border-top: 3px solid var(--grid); }}
section.failed {{ border-color: var(--bad); }} section.passed {{ border-color: var(--ok); }}
.plots {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); gap: 18px; }}
svg {{ width: 100%; min-height: 280px; border: 1px solid var(--grid); background: color-mix(in srgb, Canvas 96%, CanvasText); }}
.grid {{ stroke: var(--grid); stroke-width: 1; }} .axis {{ fill: CanvasText; font-size: 12px; }}
.actual {{ fill: none; stroke: var(--actual); stroke-width: 3; }}
.reference {{ fill: none; stroke: var(--reference); stroke-width: 2; stroke-dasharray: 8 5; }}
.forward {{ fill: none; stroke: var(--ok); stroke-width: 2; }} .reverse {{ fill: none; stroke: var(--reverse); stroke-width: 2; }}
.angular {{ fill: none; stroke: var(--actual); stroke-width: 2; }}
.marker {{ fill: var(--bad); stroke: Canvas; stroke-width: 2; }}
.legend {{ display: flex; flex-wrap: wrap; gap: 1rem; font-size: .9rem; }}
.swatch {{ display: inline-block; width: 1.3rem; height: .2rem; vertical-align: middle; margin-right: .3rem; background: var(--actual); }}
code, pre {{ font-family: ui-monospace, monospace; }} pre {{ white-space: pre-wrap; max-height: 18rem; overflow: auto; padding: .8rem; background: color-mix(in srgb, CanvasText 7%, Canvas); }}
@media (max-width: 520px) {{ body {{ padding: 12px; }} .plots {{ grid-template-columns: 1fr; }} }}
</style>
</head>
<body>
<h1>Navigation manoeuvre report</h1>
<p class="summary"><strong class="{'bad' if failed else 'ok'}">{failed} failed</strong>, <span class="ok">{passed} passed</span></p>
<div id="overview"></div><main id="scenarios"></main>
<script id="report-data" type="application/json">{payload}</script>
<script>
const reports = JSON.parse(document.getElementById('report-data').textContent);
const esc = value => String(value).replace(/[&<>"']/g, c => ({{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}}[c]));
const failedChecks = report => Object.entries(report.checks || {{}}).filter(([, value]) => value !== true).map(([name]) => name);
const flatten = (value, prefix='') => Object.entries(value || {{}}).flatMap(([key, item]) => item && typeof item === 'object' && !Array.isArray(item) ? flatten(item, prefix + key + '.') : [[prefix + key, item]]);
const fmt = value => typeof value === 'number' ? (Number.isFinite(value) ? value.toFixed(3) : String(value)) : JSON.stringify(value);
function bounds(points, xKey, yKey) {{
  const xs = points.map(p => Number(p[xKey])).filter(Number.isFinite), ys = points.map(p => Number(p[yKey])).filter(Number.isFinite);
  if (!xs.length) return {{minX:0,maxX:1,minY:0,maxY:1}};
  let minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
  const px=Math.max((maxX-minX)*.08,.25), py=Math.max((maxY-minY)*.08,.25);
  return {{minX:minX-px,maxX:maxX+px,minY:minY-py,maxY:maxY+py}};
}}
function plot(points, series, xKey, yKeys, label) {{
  const width=640, height=300, pad=42, b=bounds(points,xKey,yKeys[0]);
  for (const key of yKeys.slice(1)) {{ const other=bounds(points,xKey,key); b.minY=Math.min(b.minY,other.minY); b.maxY=Math.max(b.maxY,other.maxY); }}
  const sx=x=>pad+(x-b.minX)/(b.maxX-b.minX)*(width-2*pad), sy=y=>height-pad-(y-b.minY)/(b.maxY-b.minY)*(height-2*pad);
  const line=(items,x,y,cls)=>items.length>1?`<polyline class="${{cls}}" points="${{items.map(p=>`${{sx(p[x])}},${{sy(p[y])}}`).join(' ')}}"/>`:'';
  let content=`<line class="grid" x1="${{pad}}" y1="${{height-pad}}" x2="${{width-pad}}" y2="${{height-pad}}"/><line class="grid" x1="${{pad}}" y1="${{pad}}" x2="${{pad}}" y2="${{height-pad}}"/>`;
  for (const item of series) content += line(item.points,xKey,item.key,item.cls);
  content += `<text class="axis" x="${{pad}}" y="${{height-10}}">${{b.minX.toFixed(1)}}</text><text class="axis" x="${{width-pad-25}}" y="${{height-10}}">${{b.maxX.toFixed(1)}}</text><text class="axis" x="4" y="${{pad}}">${{b.maxY.toFixed(1)}}</text><text class="axis" x="4" y="${{height-pad}}">${{b.minY.toFixed(1)}}</text>`;
  return `<figure><figcaption><strong>${{esc(label)}}</strong></figcaption><svg viewBox="0 0 ${{width}} ${{height}}" role="img" aria-label="${{esc(label)}}">${{content}}</svg></figure>`;
}}
function trajectory(report) {{
  const points=[...report.samples,...report.reference_path,...report.markers];
  const width=640,height=300,pad=42,b=bounds(points,'x','y'),sx=x=>pad+(x-b.minX)/(b.maxX-b.minX)*(width-2*pad),sy=y=>height-pad-(y-b.minY)/(b.maxY-b.minY)*(height-2*pad);
  const poly=(items,cls)=>items.length>1?`<polyline class="${{cls}}" points="${{items.map(p=>`${{sx(p.x)}},${{sy(p.y)}}`).join(' ')}}"/>`:'';
  return `<figure><figcaption><strong>XY trajectory</strong></figcaption><svg viewBox="0 0 ${{width}} ${{height}}" role="img" aria-label="Actual and reference XY trajectory"><line class="grid" x1="${{pad}}" y1="${{height-pad}}" x2="${{width-pad}}" y2="${{height-pad}}"/><line class="grid" x1="${{pad}}" y1="${{pad}}" x2="${{pad}}" y2="${{height-pad}}"/>${{poly(report.reference_path,'reference')}}${{poly(report.samples,'actual')}}${{report.markers.map(m=>`<circle class="marker" cx="${{sx(m.x)}}" cy="${{sy(m.y)}}" r="6"><title>${{esc(m.label)}}</title></circle>`).join('')}}</svg><div class="legend"><span><i class="swatch"></i>actual</span><span><i class="swatch" style="background:var(--reference)"></i>reference</span><span><i class="swatch" style="background:var(--bad)"></i>marker</span></div></figure>`;
}}
document.getElementById('overview').innerHTML=`<table><thead><tr><th>Scenario</th><th>Result</th><th>Failed checks</th></tr></thead><tbody>${{reports.map(r=>`<tr><td><a href="#${{esc(r.scenario)}}">${{esc(r.scenario)}}</a></td><td class="${{r.passed?'ok':'bad'}}">${{r.passed?'PASS':'FAIL'}}</td><td>${{esc(failedChecks(r).join(', ') || '—')}}</td></tr>`).join('')}}</tbody></table>`;
document.getElementById('scenarios').innerHTML=reports.map(r=>{{
  const velocity=plot(r.samples,[{{points:r.samples,key:'linear',cls:'forward'}},{{points:r.samples,key:'angular',cls:'angular'}}],'time',['linear','angular'],'Command velocity over simulation time');
  const checks=Object.entries(r.checks||{{}}).map(([name,value])=>`<tr><td>${{esc(name)}}</td><td class="${{value===true?'ok':'bad'}}">${{value===true?'PASS':'FAIL'}}</td></tr>`).join('');
  const metrics=flatten(r.metrics).map(([name,value])=>`<tr><td>${{esc(name)}}</td><td><code>${{esc(fmt(value))}}</code></td></tr>`).join('');
  const diagnostics=r.diagnostics.length?`<details><summary>Relevant launch diagnostics (${{r.diagnostics.length}} lines)</summary><pre>${{esc(r.diagnostics.join('\n'))}}</pre></details>`:'';
  return `<section id="${{esc(r.scenario)}}" class="${{r.passed?'passed':'failed'}}"><h2 class="${{r.passed?'ok':'bad'}}">${{r.passed?'PASS':'FAIL'}} — ${{esc(r.scenario)}}</h2>${{r.error?`<p class="bad"><strong>${{esc(r.error)}}</strong></p>`:''}}<div class="plots">${{trajectory(r)}}${{velocity}}</div><h3>Acceptance checks</h3><table><tbody>${{checks}}</tbody></table><h3>Metrics</h3><table><tbody>${{metrics||'<tr><td>No metrics recorded</td></tr>'}}</tbody></table>${{diagnostics}}</section>`;
}}).join('');
</script>
</body></html>'''
    (report_root / 'report.html').write_text(document)


def generate_report(report_root, scenarios):
    """Generate all suite-level artifacts and return whether the suite passed."""
    report_root.mkdir(parents=True, exist_ok=True)
    reports = []
    for scenario in scenarios:
        report = _load_report(report_root, scenario)
        report['diagnostics'] = (
            clean_launch_log(report_root / f'{scenario}.launch.log')
            + clean_launch_log(report_root / f'{scenario}.runner.log')
        )[-120:]
        reports.append(report)
    passed = sum(report['passed'] for report in reports)
    summary = {
        'passed': passed == len(reports),
        'scenario_count': len(reports),
        'passed_count': passed,
        'failed_count': len(reports) - passed,
        'scenarios': [{
            'name': report['scenario'],
            'passed': report['passed'],
            'failed_checks': _failed_checks(report),
            'error': report.get('error'),
        } for report in reports],
    }
    (report_root / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n')
    _write_junit(report_root, reports)
    _write_html(report_root, reports)
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
    print('RESULT  SCENARIO                    FAILED CHECKS')
    for result in summary['scenarios']:
        status = 'PASS' if result['passed'] else 'FAIL'
        detail = ', '.join(result['failed_checks']) or '-'
        if result['error']:
            detail = result['error']
        print(f"{status:<6}  {result['name']:<28} {detail}")
    print('')
    print(f"{summary['failed_count']} failed, "
          f"{summary['passed_count']} passed")
    print(f"HTML:  {options.report_root / 'report.html'}")
    print(f"JUnit: {options.report_root / 'junit.xml'}")
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
