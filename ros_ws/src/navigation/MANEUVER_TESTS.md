# Navigation maneuver tests

This test set isolates propulsion, local path following, and complete obstacle
avoidance. It is intentionally shorter than a GPS shuttle run and produces a
machine-readable JSON report for every scenario.

## Run the complete set

Build and source the workspace, stop any other Karaburan simulation, and run:

```bash
cd /home/michiel/karaburan/ros_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select navigation boatcontrol
source install/setup.bash
bash install/navigation/share/navigation/scripts/run_maneuver_tests.sh
```

Pass a directory to retain the reports at a specific location:

```bash
bash install/navigation/share/navigation/scripts/run_maneuver_tests.sh \
  /home/michiel/karaburan/maneuver-test-results/manual-check
```

Add one or more scenario names after the report directory for a quick focused
run. This is the preferred feedback loop while tuning a controller:

```bash
bash install/navigation/share/navigation/scripts/run_maneuver_tests.sh \
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

## Scenarios and gates

- `actuator_straight`: three metres of open-loop forward thrust.
- `actuator_turn_left` and `actuator_turn_right`: symmetric steering signs.
- `follow_straight`: a three-metre path sent directly to `FollowPath`.
- `follow_arc_left` and `follow_arc_right`: five-metre-radius controller paths.
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
machine while the controller is being tuned. It can become a scheduled or
manually dispatched CI job after it passes reliably on that machine.
