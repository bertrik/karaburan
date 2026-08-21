# Ravensberg Gazebo world development plan

This file is the active development prompt and hand-off source for the
Ravensberg simulation. Read it, together with the relevant project
documentation, before doing work in a new session. Keep it concise and current:
move accepted design knowledge to project documentation and remove detailed
instructions for accepted milestones.

## Current status

Current milestone: Milestone 0 - project inspection

Status: Technical inspection complete; awaiting user review and explicit
acceptance. Do not implement Milestone 1 yet.

Last completed milestone: None. Milestone 0 is not accepted until the user says
so explicitly.

Working baseline:

- ROS 2 Jazzy workspace with a Gazebo Sim integration based on `ros_gz`.
- The archive targets the Gazebo Harmonic family through ROS 2 Jazzy, but does
  not pin or record an exact Gazebo patch version.
- The existing 0.5 m twin-thruster boat floats, accepts `/cmd_vel`, and publishes
  simulated IMU, NavSat, and GPU LiDAR data.
- The current world is only a 500 x 500 m visual water plane at `z = 0`; it has
  no Ravensberg shoreline, islands, bottom, or scenario representation.
- World coordinates use metres, ENU orientation, and a WGS84 origin at
  `52.018599, 4.708720`.
- No map source, shoreline geometry, meshes, or textures for Ravensberg are in
  the archive.

Important files:

- `ros_ws/src/karaburan_simulation/worlds/world.sdf`
- `ros_ws/src/karaburan_simulation/models/karaburan_boat.sdf`
- `ros_ws/src/karaburan_simulation/launch/sim.launch.py`
- `ros_ws/src/karaburan_simulation/karaburan_simulation/simcontrolnode.py`
- `ros_ws/src/navigation/config/karaburan.xacro`
- `ros_ws/src/navigation/launch/nav2_stack.launch.py`
- `ros_ws/src/karaburan_navigation_tests/`
- `docker/Dockerfile.ros-jazzy-test`
- `docker/ros-test.sh`
- `AGENTS.md`

Known issues:

- A Ravensberg source outline and its authoritative WGS84 reference are not in
  the repository. The existing world origin must not silently be assumed to be
  the desired Ravensberg origin.
- Gazebo is not installed or version-pinned by the repository's test image;
  current local CI validates source and launch structure, not a running Gazebo
  world. Runtime simulation tests live in a separate optional test workflow.
- The user has explicitly allowed use of the simulation machine for runtime
  verification in later approved milestones.
- The world has no bottom collision. Buoyancy is defined by the waterline and
  density change, not by the visible water plane.
- There is no scenario configuration, generated metadata, debug map, seed
  handling, or scenario route validation yet.
- The SDF model's physical root link is `hull`, while the ROS robot description
  uses `base_link`. Navigation currently derives `odom -> base_link` from sensor
  fusion rather than a Gazebo model-pose bridge. Avoid changing this as part of
  the MVP unless it blocks a verified use case.
- The repository already contains untracked `20260818-logfiles/`; they predate
  this work and must remain untouched.

Next proposed step: after explicit approval, implement only Milestone 1, the
minimal navigable and debuggable Ravensberg MVP described below.

Do not continue beyond this step without user approval.

## Non-negotiable working rules

- Work one explicitly approved milestone at a time. Feedback refines the current
  milestone and does not authorize the next one.
- Prefer progress over perfection and the smallest usable implementation.
- Keep fixed world geometry separate from scenario content where practical.
- Make every milestone independently usable, testable, debuggable, resumable,
  and performance-conscious.
- Use one Gazebo metre as one real metre.
- Make generated scenarios reproducible from a seed.
- Keep visual geometry, collision geometry, navigation/safety extent, physical
  properties, scenario semantics, and debug representation conceptually
  separate for relevant objects.
- Do not add expensive detail without first presenting concrete options,
  performance trade-offs, and a recommendation, then waiting for the user's
  choice. This especially applies to vegetation, high-resolution bathymetry,
  dynamic waves, many dynamic entities, and complex meshes/collisions.
- Never run `git commit`, `git add`, staging, unstaging, or any equivalent Git
  index mutation. Read-only Git inspection is allowed.
- Before handing off changes to ROS code, package metadata, launch files, Docker
  tests, or GitHub Actions, run the complete local CI check mandated by
  `AGENTS.md` with `--pull --no-cache`, and treat lint warnings as failures.
- At the end of a milestone, update relevant project documentation and this
  plan, report tests and performance, name changed files and known limitations,
  propose exactly one next step, and stop for approval.

## Milestone 0 inspection findings

### Technology and package structure

- ROS: ROS 2 Jazzy on Ubuntu Noble (`ros:jazzy-ros-base` in local CI).
- Simulator: Gazebo Sim, not Gazebo Classic and not legacy Ignition APIs.
  Evidence includes `ros_gz_sim`, `ros_gz_bridge`, `gz::sim::systems::*`, and
  SDF 1.10. ROS 2 Jazzy normally pairs with Gazebo Harmonic, but the repository
  does not pin the exact installed release.
- Build system: one ROS workspace under `ros_ws`, built with `colcon`.
- Packages: `boatcontrol`, `bt785`, `karaburan_bringup`, `karaburan_msgs`,
  `karaburan_navigation_tests`, `karaburan_simulation`, `karaburan_ui`, `lidar`,
  `mpu9250`, `navigation`, `sonar`, `tempreader`, and `vl53l0x`.
- All packages except the interface package `karaburan_msgs` use
  `ament_python`; `karaburan_msgs` uses `ament_cmake`.
- Simulation ownership is already sensibly concentrated in
  `karaburan_simulation`; reusable manoeuvre testing is in
  `karaburan_navigation_tests`.

### Existing world

- `world.sdf` defines world `ocean` using SDF 1.10.
- Physics engine: DART, `max_step_size = 0.001`,
  `real_time_update_rate = 1000`, targeting real-time simulation.
- Systems: physics, user commands, scene broadcaster, graded buoyancy, Ogre 2
  sensors, IMU, and NavSat.
- Waterline: `z = 0`. The graded buoyancy density is 1000 kg/m3 below the
  waterline and 1 kg/m3 above it.
- Water visualization: one transparent 500 x 500 m plane. It has no collision,
  wave model, current model, shoreline, or depth representation.
- Geographic frame: WGS84, ENU, heading zero, origin
  `52.018599, 4.708720`, elevation zero.
- There is currently no land, island, bathymetry, bottom, vegetation, obstacle,
  or Ravensberg geometry.

### Existing boat and movement

- The Gazebo boat is a primitive-only SDF model. No external mesh or texture is
  used.
- Hull visual and collision are a 0.5 x 0.25 x 0.13 m box with mass 2.0 kg.
- Two 0.05 m diameter propellers use independent Gazebo Thruster systems.
- The hull uses the Gazebo Hydrodynamics system with simple linear and quadratic
  damping. Coriolis and added mass are disabled.
- `simcontrol_node` translates `/cmd_vel` surge and yaw requests into `/left`
  and `/right` shaft commands and stops the propellers after a 0.5 s command
  timeout.
- `sim.launch.py` starts Gazebo, Nav2, the control bridge, and optional RViz and
  Leaflet UI, then spawns the boat after two seconds. It supports headless mode
  and alternate world/model paths.
- The existing manoeuvre-test package already demonstrates isolated headless
  worlds, private ROS domains/Gazebo partitions, runtime entity spawning, JSON
  results, and graphical test reports. Reuse the useful patterns without
  coupling the Ravensberg scenario generator to the manoeuvre runner.

### Sensors, bridges, and frames

- Simulated sensors on `hull`: IMU at 30 Hz (`/imu/data`, `imu_link`), NavSat at
  5 Hz (`/fix/valid`, `gps_link`), and forward 180-degree GPU LiDAR at 10 Hz with
  720 samples (`/scan`, `lidar_link`).
- `ros_gz_bridge` bridges those sensor topics, `/clock`, and the two propeller
  command topics.
- The ROS Xacro publishes `base_link` and fixed `imu_link`, `gps_link`, and
  `lidar_link` transforms. The EKF publishes `odom -> base_link`; with SLAM
  disabled, a static publisher supplies `map -> odom`.
- The map axes and Gazebo spherical coordinates are ENU: positive x is east,
  positive y is north, and positive z is up. SDF geometry dimensions are metres,
  so the existing project already follows the required 1:1 scale.

### Current test and performance baseline

- The committed local CI image builds every ROS package and runs Python compile,
  `colcon` tests, launch validation, and an MCAP smoke test.
- Inspection of the available `karaburan-ros-test:latest` image confirms its
  environment is ROS 2 Jazzy and that it contains no `gz` executable or
  `ros_gz_sim` installation.
- The simulation package's present tests are structural assertions; they do not
  start Gazebo or measure real-time factor.
- The optional navigation manoeuvre world requests a four-times accelerated
  simulation (`real_time_update_rate = 4000`) and launches a fresh headless
  Gazebo instance per test case. This is useful for later functional smoke
  tests, not a current performance measurement of the main world.
- Baseline main-world entity load is minimal: one static visual water model and
  one dynamic boat with three sensors. No entity/collision count telemetry is
  currently captured.
- Milestone 0 changes documentation only, so the expensive complete ROS CI run
  is not required. Milestone 1 will affect ROS simulation resources and must run
  the complete check from `AGENTS.md`.

## Smallest path to the Ravensberg MVP

The smallest coherent change is a simple, deterministic 2D geometry source that
drives both Gazebo and debug output. Do not hand-maintain separate shoreline
shapes in SDF and in a plotting script.

1. Add one compact Ravensberg MVP configuration containing the world origin,
   metre-scale water/land/island polygons, seed, ownship start, and goal.
2. Add a small generator in `karaburan_simulation` that validates the config and
   emits:
   - a generated SDF world with a water surface, flat bottom at about -2.0 m,
     and simplified static land/island geometry;
   - compact JSON metadata with seed, scale, origin, start, goal, and navigable
     geometry; and
   - a debug image showing water, land, islands, navigable area, origin, scale,
     start, and goal.
3. Keep the existing boat model, buoyancy, hydrodynamics, bridges, and
   `/cmd_vel` path unchanged. Spawn ownship at the configured start pose.
4. Support a debug-only command that validates and renders the same scenario
   without starting Gazebo.
5. Add focused tests for deterministic output, start/goal in water, geometry
   validity, metadata, and debug output.

This is the minimum path because one shared geometric source makes Ravensberg
recognizable, preserves the already working boat, introduces scenario
configuration, and makes the scenario immediately inspectable without adding a
generic framework or later-milestone objects.

## Proposed Milestone 1 - navigable and debuggable MVP

Implement only the following after explicit user approval:

- A deliberately simplified but recognizable Ravensberg outline with its main
  islands, stored as metre-scale source geometry in the repository.
- A 1:1 metre world with waterline `z = 0` and a flat bottom near `z = -2.0 m`.
- Existing ownship spawned at a configured valid start pose.
- A scenario configuration with `seed: 1`, start pose, and goal position.
- Generated scenario metadata.
- A debug-only workflow and debug map showing water, land, islands, start, goal,
  navigable water, world origin, north/east orientation, and scale.
- Tests for map processing/geometry, start and goal validity, deterministic
  generation, and existence/readability of debug and metadata output.
- A lightweight performance record: generated entity/collision counts,
  generation duration, and a short headless Gazebo real-time-factor observation
  if the runtime exposes it reliably.

Before implementation, resolve the missing source geometry using the smallest
reproducible option: add a repository-owned simplified outline with documented
provenance and coordinates. If obtaining or licensing a source requires a
material external choice, stop and ask the user rather than fabricating a
shoreline.

Milestone 1 acceptance criteria:

- The debug-only command succeeds from a built workspace without Gazebo.
- The same config drives the debug representation and generated Gazebo world.
- Start and goal are in navigable water and are visibly marked.
- Gazebo starts the generated world and the existing boat can move under
  `/cmd_vel` without changing its control or hydrodynamics implementation.
- Land and island collisions constrain the boat; the bottom does not interfere
  with normal floating at the configured start.
- Repeating generation with the same config and seed produces byte-stable
  metadata and equivalent geometry/debug output.
- The complete local CI check required by `AGENTS.md` succeeds.
- Performance remains usable and the reported entity/collision counts stay
  small; visual detail is not expanded in this milestone.

Explicitly excluded from Milestone 1: reeds, lily pads, peat chunks, buoys,
procedural placement, route validation/A*, variable bathymetry, birds, another
vessel, dynamic waves, detailed meshes, and preset difficulty levels.

## Remaining active roadmap

- Milestone 2: explicit, manually positioned reed zones, lily-pad fields, peat
  chunks, and buoys, each represented in configuration, metadata, navigation
  semantics, and debug output. Present implementation-cost options before adding
  visually dense vegetation.
- Milestone 3: seed-reproducible procedural placement, water/shore/obstacle
  distance queries, and occupancy-grid plus A* validation that start can reach
  goal before Gazebo starts.
- Milestone 4: smooth 1-3 m bathymetry, predominantly 1.8-2.3 m, with narrow
  corridors blended toward about 2.1 m and vegetation placement refined by
  depth.
- Milestone 5: swimming birds (`BIRD`, `DYNAMIC_WILDLIFE`) with distinct physical
  and disturbance radii, simple water-valid waypoint motion, and debug routes.
- Milestone 6: an approximately 3 m target motor vessel (`TARGET_VESSEL`,
  `DYNAMIC_VESSEL`) with low-cost collision/visual geometry and smooth waypoint
  motion; start with simple head-on, crossing, or overtaking situations rather
  than full COLREG logic.
- Milestone 7: `mvp`, `easy`, `medium`, and `hard` presets plus targeted visual
  and usability refinement.

Required semantic classes for later milestones:

| Semantic type | Navigation class |
| --- | --- |
| `LAND` | `HARD_OBSTACLE` |
| `REED_ZONE` | `AVOID_ZONE` |
| `LILY_PAD_FIELD` | `STATIC_SOFT_OBSTACLE` |
| `PEAT_CHUNK` | `STATIC_HARD_OBSTACLE` |
| `BUOY` | `STATIC_HARD_OBSTACLE` |
| `BIRD` | `DYNAMIC_WILDLIFE` |
| `TARGET_VESSEL` | `DYNAMIC_VESSEL` |

Later debug output should grow into geometry, scenario, navigation, and depth
views. Generate only information used by developers or consumers; avoid large
metadata dumps and over-documentation.
