# Navigation maneuver tests

This test set isolates propulsion, global path planning, local path following,
and complete obstacle avoidance. It is intentionally shorter than a GPS
shuttle run. Every execution produces per-scenario JSON plus suite-level
JUnit, JSON, and HTML reports.

## Run the complete set

Build and source the workspace, then run:

```bash
cd /home/michiel/karaburan/ros_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to karaburan_navigation_tests
source install/setup.bash
bash install/karaburan_navigation_tests/share/karaburan_navigation_tests/scripts/run_maneuver_tests.sh
```

Pass a directory to retain the reports at a specific location:

```bash
bash install/karaburan_navigation_tests/share/karaburan_navigation_tests/scripts/run_maneuver_tests.sh \
  /home/michiel/karaburan/test-results-manual-check
```

Add one or more scenario names after the report directory for a quick focused
run. This is the preferred feedback loop while tuning a controller:

```bash
bash install/karaburan_navigation_tests/share/karaburan_navigation_tests/scripts/run_maneuver_tests.sh \
  /home/michiel/karaburan/test-results-follow-straight \
  actuator_straight follow_straight
```

Each scenario gets a fresh headless Gazebo server. The dedicated world requests
four simulated seconds per wall-clock second. Actual acceleration depends on
the host CPU. RViz, Leaflet, recording, and physical instruments are disabled.
The script assigns each scenario a private ROS domain and Gazebo partition, so
a normal simulation can remain active without mixing clocks, topics, or world
services. Separate domains also prevent DDS discovery state from one fresh
simulator leaking into the next one.
Set `KARABURAN_TEST_DOMAIN_ID` or `KARABURAN_TEST_GZ_PARTITION` only when a CI
executor needs fixed identifiers.

## Results and diagnostics

The command exits with status `1` when any acceptance check fails, when a
runner crashes, or when a scenario does not write its result. The report
directory contains:

- `junit.xml`: one test case per scenario. CI systems display failed scenarios
  as red tests rather than treating a completed script as success.
- `summary.json`: a small suite summary suitable for automation.
- `report.html`: a self-contained `junit2html` report listing every test case,
  the exact failed acceptance checks, observed metrics, relevant log lines, and
  the names of the related artifacts.
- `<scenario>.svg`: a static plot of the actual XY trajectory and reference
  path, labelled with the scenario status and failed checks. Every plot is
  also embedded directly in the combined HTML report, for both passed and
  failed scenarios.
- `<scenario>.json`: the complete trace used to calculate the result.
- `<scenario>.launch.log` and `<scenario>.runner.log`: plain-text logs with ANSI
  terminal colour removed. The HTML report includes only relevant warning and
  error lines; the full logs remain available for deeper investigation.

Open `report.html` in a browser on the workstation used to inspect the test.
It has no external assets and does not need a running ROS graph or web server.

## Run every navigation test in one report

Run all manoeuvre scenarios and the normal `karaburan_navigation_tests`
package tests with one command:

```bash
bash install/karaburan_navigation_tests/share/karaburan_navigation_tests/scripts/run_simulation_tests.sh \
  /home/michiel/karaburan
```

This creates exactly one flat directory named
`test-results-YYYYMMDDHHMMSS`, for example
`test-results-20260821221557`. It contains one combined `report.html` and
`junit.xml`, plus the original JUnit files, embedded and standalone SVG plots,
JSON traces, and plain-text logs. No result subdirectories are created. A
failing test or missing result makes the command exit with status `1`.

## Scenarios and gates

The report separates three layers so a propulsion defect cannot be mistaken
for a route-planning defect:

- `actuator`: open-loop commands bypass Nav2 and validate simulation thrust.
- `controller`: a supplied local path validates tracking without global
  planning.
- `planner`: direct global-planning checks and complete obstacle execution.

- `actuator_straight`: three metres of open-loop forward thrust.
- `actuator_turn_left` and `actuator_turn_right`: symmetric steering signs.
- `follow_straight`: a three-metre path sent directly to `FollowPath`.
- `follow_arc_left` and `follow_arc_right`: five-metre-radius controller paths.
- `planner_direct`: six aligned 30-metre plans at representative headings,
  requested directly from `ComputePathToPose` without moving the boat. Every
  plan must be monotonic, cusp-free, within 0.5 metres of the straight line,
  and no more than 0.30 metres longer than that line. The 0.25-metre endpoint
  tolerance matches the planner tolerance and the 0.20-metre costmap grid;
  metre-scale lateral detours still fail independently.
- `planner_island`: plans a smooth, forward-only route around a 2.5-metre
  island and rejects loops, cusps, side changes, and excessive detours.
- `island_navigation`: sails that island route and checks endpoint,
  cross-track error, forward-only motion, efficiency, and heel angle.
- `open_obstacle_port` and `open_obstacle_starboard`: mirrored isolated blocks
  at 1.5 metres and an eight-metre navigation goal. Open water remains
  available around the block, so any reverse command fails the scenario.
- `harbour_reverse_stern_port` and `harbour_reverse_stern_starboard`: mirrored
  C-shaped berths. A wall blocks straight reverse travel and one side wall
  blocks the wrong arc, so the stern must leave through the named side.
- `harbour_reverse_straight`: a symmetric U-shaped berth in which both arcs
  meet a side wall and straight reverse is the shortest clear departure.
- `harbour_dock_stern_port` and `harbour_dock_stern_starboard`: approach the
  mirrored berth from open water, turn approximately 90 degrees, and finish
  bow-first at the original berth pose.
- `harbour_dock_straight`: approaches the symmetric berth directly and must
  not introduce an unnecessary turn.

The open-water reports require forward-only travel, obstacle clearance, no
loop, reasonable path efficiency, decreasing goal distance, and successful
goal completion. The harbour reports isolate the initial departure. They
require one uninterrupted reverse segment and either a straight 2.8-metre
escape or the selected 2-metre-radius, 90-degree arc. The plot draws the quay
geometry and start/end bow headings, so a wrong stern direction is visible as
well as machine-verifiable. Docking checks use the complete padded Nav2 hull
footprint at every recorded pose, reject quay overlap, and require a
collision-free forward approach, final position, final heading, route
efficiency, and realistic heel. Every plot overlays sampled 0.56 x 0.30 metre
hull footprints so the centre line is not mistaken for the space occupied by
the boat. A non-finite Gazebo, sensor, or EKF pose terminates the action early
and fails as `finite_simulation_state`; invalid samples are not written into
JSON or SVG artifacts.

## CI policy

The deterministic metric tests run in every normal `colcon test` and therefore
in the existing GitHub Actions job. They verify thresholds and ensure that a
recorded circle fails.

The complete Gazebo set is deliberately not a required push check. It needs the
Gazebo/Nav2 runtime, is sensitive to runner performance, and is substantially
more expensive than the current ROS base build. Run it on the simulation
machine while the controller is being tuned. Its non-zero exit status and
`junit.xml` are ready for a dedicated scheduled or manually dispatched CI job;
the deterministic report-generator tests remain part of every push check.
