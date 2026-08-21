#!/usr/bin/env bash
set -eo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo 'Source ROS 2 and the Karaburan workspace before running this script.' >&2
  exit 2
fi

# Avoid a long-lived ros2 daemon carrying graph state between scenarios.
export ROS2CLI_NO_DAEMON=1
# ROS and Gazebo launch output is archived as plain text. This makes it safe to
# inspect in CI systems and editors that do not interpret terminal escapes.
export RCUTILS_COLORIZED_OUTPUT=0
export GZ_LOG_COLOR=0

report_root="${1:-./test-results-$(date +%Y%m%d%H%M%S)}"
shift || true
mkdir -p "$report_root"
status=0
launch_pid=''

stop_simulation() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  launch_pid=''
}
trap stop_simulation EXIT INT TERM

if (( $# > 0 )); then
  scenarios=("$@")
else
  scenarios=(
    actuator_straight
    actuator_turn_left
    actuator_turn_right
    follow_straight
    follow_arc_left
    follow_arc_right
    planner_direct
    planner_island
    island_navigation
    obstacle_port
    obstacle_starboard
  )
fi

valid_scenarios=' actuator_straight actuator_turn_left actuator_turn_right follow_straight follow_arc_left follow_arc_right planner_direct planner_island island_navigation obstacle_port obstacle_starboard '
for scenario in "${scenarios[@]}"; do
  if [[ "$valid_scenarios" != *" $scenario "* ]]; then
    echo "Unknown scenario: $scenario" >&2
    exit 2
  fi
done

scenario_index=0
report_arguments=()
for scenario in "${scenarios[@]}"; do
  report_arguments+=(--scenario "$scenario")
  # Give every fresh simulator a private ROS graph and Gazebo transport graph.
  # This also avoids DDS discovery remnants from the preceding scenario.
  if [[ -n "${KARABURAN_TEST_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID="$KARABURAN_TEST_DOMAIN_ID"
  else
    export ROS_DOMAIN_ID="$((20 + ($$ + scenario_index) % 80))"
  fi
  if [[ -n "${KARABURAN_TEST_GZ_PARTITION:-}" ]]; then
    export GZ_PARTITION="$KARABURAN_TEST_GZ_PARTITION"
  else
    export GZ_PARTITION="karaburan_maneuver_$$_${scenario_index}"
  fi
  scenario_index=$((scenario_index + 1))

  echo "Running $scenario"
  setsid ros2 launch karaburan_navigation_tests maneuver_test.launch.py \
    >"$report_root/$scenario.launch.log" 2>&1 &
  launch_pid=$!

  ready=false
  ready_deadline=$((SECONDS + 90))
  while (( SECONDS < ready_deadline )); do
    controller_state="$(timeout 3s ros2 lifecycle get /controller_server 2>/dev/null || true)"
    navigator_state="$(timeout 3s ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    if timeout 3s ros2 topic list 2>/dev/null | grep -q '^/odometry/filtered$' \
        && [[ "$controller_state" == *'active [3]'* ]] \
        && [[ "$navigator_state" == *'active [3]'* ]]; then
      ready=true
      break
    fi
    sleep 0.5
  done

  if [[ "$ready" != true ]]; then
    echo "Simulator did not become ready for $scenario" >&2
    status=1
  elif ! ros2 run karaburan_navigation_tests maneuver_test_runner \
      --scenario "$scenario" \
      --output "$report_root/$scenario.json" \
      >"$report_root/$scenario.runner.log" 2>&1; then
    status=1
  fi

  stop_simulation
  sleep 2
done

set +e
ros2 run karaburan_navigation_tests maneuver_test_report \
  --report-root "$report_root" "${report_arguments[@]}"
report_status=$?
set -e
if [[ "$report_status" -ne 0 ]]; then
  status=1
fi
exit "$status"
