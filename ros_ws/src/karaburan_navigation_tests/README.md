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
  /home/michiel/karaburan/maneuver-test-results/manual-check
```

Add one or more scenario names after the report directory for a quick focused
run. This is the preferred feedback loop while tuning a controller:

```bash
bash install/karaburan_navigation_tests/share/karaburan_navigation_tests/scripts/run_maneuver_tests.sh \
  /home/michiel/karaburan/maneuver-test-results/follow-straight \
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
  path, labelled with the scenario status and failed checks.
- `<scenario>.json`: the complete trace used to calculate the result.
- `<scenario>.launch.log` and `<scenario>.runner.log`: plain-text logs with ANSI
  terminal colour removed. The HTML report includes only relevant warning and
  error lines; the full logs remain available for deeper investigation.

Open `report.html` in a browser on the workstation used to inspect the test.
It has no external assets and does not need a running ROS graph or web server.

## Run the simulation subsystem tests

Use the focused simulation suite to verify straight thrust and both steering
directions without running the Nav2 controller scenarios:

```bash
bash install/karaburan_navigation_tests/share/karaburan_navigation_tests/scripts/run_simulation_tests.sh \
  /home/michiel/karaburan/simulation-test-results
```

This creates a new leaf directory named `YYYYMMDDHHMMSS` for every run. The
directory contains a combined `report.html` and `junit.xml` for the three
dynamic actuator scenarios, the `karaburan_simulation` unit tests, and the
navigation test harness. The original JUnit files, static trajectory plots,
JSON traces, and plain-text logs remain beside the combined report. A failing
test or missing result makes the command exit with status `1`.

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
  and no more than 0.05 metres longer than that line.
- `obstacle_port` and `obstacle_starboard`: mirrored blocks at 1.5 metres and
  an eight-metre navigation goal.

The obstacle reports require one reverse segment followed by one forward
segment, a `2.0 +/- 0.2 m` reverse radius, a `90 +/- 3 degree` heading change,
no return to the obstacle, no forward loop above 150 degrees, decreasing goal
distance, and successful goal completion.

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
