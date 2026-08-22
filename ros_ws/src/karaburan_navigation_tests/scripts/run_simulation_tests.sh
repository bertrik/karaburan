#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
simulation_prefix="$(ros2 pkg prefix karaburan_simulation)"
workspace_root="$(dirname "$(dirname "$simulation_prefix")")"
repository_root="$(dirname "$workspace_root")"
result_parent="${1:-$repository_root}"
timestamp="$(date +%Y%m%d%H%M%S)"
report_root="$result_parent/test-results-$timestamp"

echo "Navigation test run: $timestamp"
echo "Results: $report_root"

status=0
set +e
bash "$script_dir/run_maneuver_tests.sh" \
  "$report_root"
maneuver_status=$?
set -e
if [[ $maneuver_status -ne 0 ]]; then
  status=1
fi

mv "$report_root/junit.xml" "$report_root/maneuver.junit.xml"

set +e
(
  cd "$workspace_root"
  RCUTILS_COLORIZED_OUTPUT=0 PY_COLORS=0 TERM=dumb \
    colcon test \
      --packages-select karaburan_navigation_tests \
      --event-handlers console_direct+
) >"$report_root/colcon.log" 2>&1
colcon_status=$?
set -e
if [[ $colcon_status -ne 0 ]]; then
  status=1
fi

if ! cp "$workspace_root/build/karaburan_navigation_tests/pytest.xml" \
  "$report_root/karaburan_navigation_tests.junit.xml"; then
  status=1
fi

ros2 run karaburan_navigation_tests junit_html_report \
  --input "$report_root/maneuver.junit.xml" \
  --input "$report_root/karaburan_navigation_tests.junit.xml" \
  --output-xml "$report_root/junit.xml" \
  --output-html "$report_root/report.html"

echo "Combined HTML:  $report_root/report.html"
echo "Combined JUnit: $report_root/junit.xml"
exit "$status"
