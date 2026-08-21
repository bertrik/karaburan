#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
result_parent="${1:-./simulation-test-results}"
timestamp="$(date +%Y%m%d%H%M%S)"
report_root="$result_parent/$timestamp"

echo "Simulation test run: $timestamp"
echo "Results: $report_root"

exec bash "$script_dir/run_maneuver_tests.sh" \
  "$report_root" \
  actuator_straight \
  actuator_turn_left \
  actuator_turn_right
