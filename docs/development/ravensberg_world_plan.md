# Ravensberg Gazebo world development plan

This is the active development prompt and hand-off source for the Ravensberg
simulation. Read it and `docs/ravensberg_world.md` before doing work in a new
session. Keep accepted design knowledge in project documentation and keep this
file focused on current and future work.

## Current status

Current milestone: Milestone 1 - navigable and debuggable MVP

Status: Technical implementation and verification complete; awaiting explicit
user acceptance. Feedback remains part of Milestone 1. Do not implement
Milestone 2 yet.

Last completed milestone: Milestone 0 - project inspection, accepted when the
user explicitly authorized Milestone 1.

Working baseline:

- ROS 2 Jazzy and Gazebo Sim Harmonic (`gz sim` 8.11.0 in runtime verification).
- A deterministic, 1:1 metre Ravensberg world generated from repository-owned
  scenario configuration derived from OpenStreetMap relation `14671863`.
- Simplified water outline, twelve main islands/legakkers, a flat 2.0 m bottom,
  land collisions, ownship start, goal, semantic metadata, and an SVG debug map.
- The existing boat spawns at the configured start and moves through its
  unchanged `/cmd_vel`, buoyancy, hydrodynamics, and twin-thruster path.
- Debug-only generation works without Gazebo and the same configuration drives
  SDF, metadata, and debug geometry.

Important files:

- `docs/ravensberg_world.md`
- `ros_ws/src/karaburan_simulation/config/ravensberg_mvp.json`
- `ros_ws/src/karaburan_simulation/karaburan_simulation/ravensberg_scenario.py`
- `ros_ws/src/karaburan_simulation/launch/ravensberg.launch.py`
- `ros_ws/src/karaburan_simulation/generated/`
- `ros_ws/src/karaburan_simulation/models/karaburan_boat.sdf`
- `ros_ws/src/karaburan_simulation/test/test_ravensberg_scenario.py`
- `docker/ros-test.sh`
- `AGENTS.md`

Known issues:

- The MVP geometry is intentionally simplified and omits many small legakkers
  and narrow channels.
- Island visuals and navigation polygons follow the source outline, while
  Gazebo uses conservative box collisions because its buoyancy system rejects
  polyline collision geometry.
- The bottom is flat; bathymetry belongs to Milestone 4.
- The optional Leaflet UI still uses the legacy WGS84 origin. Leave `with_map`
  disabled for Ravensberg until the UI becomes scenario-origin aware.
- SVG labels overlap in the dense southern area, but metadata IDs and polygons
  remain unambiguous.
- The existing boat SDF emits parser warnings for Gazebo-specific `gz_frame_id`
  sensor elements. These predate Ravensberg and do not block the sensors.
- The repository contains unrelated existing/concurrent working-tree changes
  and untracked `20260818-logfiles/`; preserve them and do not stage them.

Next proposed step: after the user accepts Milestone 1 and selects a performance
option below, implement only Milestone 2 with explicit static reed, lily-pad,
peat-chunk, and buoy positions.

Do not continue beyond this step without user approval.

## Completed

- Milestone 0: repository, ROS/Gazebo, boat, physics, sensor, launch, frame, test,
  and performance-baseline inspection.
- Milestone 1 technical result: navigable and debuggable Ravensberg MVP. See
  `docs/ravensberg_world.md` for architecture, source provenance, coordinate
  frames, commands, performance decisions, and limitations.

Milestone 1 verification:

- Deterministic generation: repeated output is byte-stable.
- Debug-only generation: metadata and SVG succeed without Gazebo.
- Geometry: start and goal are valid navigable-water positions.
- Complexity: 3 static models, 37 collision shapes, and 38 visuals.
- Generator duration: approximately 6-21 ms in tested environments.
- Gazebo: generated world loads without world/geometry/plugin errors.
- Control: ten seconds of forward `/cmd_vel` moved ownship from x = -120.00 m
  to x = -117.45 m.
- Performance: 50-sample mean real-time factor 0.801 versus 0.831 for the old
  empty-water world in the same software-rendering container (about 3.6% lower).
- Focused validation: 7 targeted tests passed and `ament_flake8` was clean.
- Required complete CI: clean `--pull --no-cache` image build succeeded; 47
  tests ran with 0 errors and 0 failures, and launch/generator/MCAP checks passed.

## Non-negotiable working rules

- Work one explicitly approved milestone at a time. Feedback refines the current
  milestone and does not authorize the next one.
- Prefer progress over perfection and the smallest usable implementation.
- Keep fixed world geometry separate from scenario content where practical.
- Make every milestone independently usable, testable, debuggable, resumable,
  and performance-conscious.
- Use one Gazebo metre as one real metre and seed all generated scenarios.
- Keep visual, collision, navigation/safety, physical, semantic, and debug
  representations conceptually separate.
- Before implementing a potentially expensive representation, present concrete
  options and wait for the user's choice.
- Never commit, stage, unstage, or otherwise mutate the Git index. Read-only Git
  inspection is allowed.
- For changes covered by `AGENTS.md`, run both complete clean Docker CI commands
  before hand-off and treat lint warnings as failures.
- At each milestone end, update documentation and this plan, report tests and
  performance, propose exactly one next step, and stop for approval.

## Milestone 2 performance choice

Choose one option before implementing static scenario objects:

### Option A - cluster geometry (recommended)

- One semantic/configuration object per reed or lily patch.
- One inexpensive visual cluster and at most one simple collision/navigation
  shape per patch; no physics per stem or leaf.
- Peat chunks and buoys use low-poly visuals and simple hard collisions.
- Lowest entity and physics load; enough detail for planning and sensor tests.

### Option B - richer clustered visuals

- Multiple visual elements inside each reed/lily patch, still one semantic and
  collision/navigation shape per patch.
- Better appearance and LiDAR diversity at moderate rendering cost.
- No physics per stem or leaf.

### Option C - individual models

- Many separate reed stems or lily leaves with individual entities/collisions.
- Highest visual detail, but large rendering, physics, and spawn-time cost.
- Not recommended for autonomous-navigation testing.

## Remaining active roadmap

- Milestone 2: explicitly positioned reeds (`REED_ZONE`, `AVOID_ZONE`), lily-pad
  fields (`LILY_PAD_FIELD`, `STATIC_SOFT_OBSTACLE`), peat chunks (`PEAT_CHUNK`,
  `STATIC_HARD_OBSTACLE`), and buoys (`BUOY`, `STATIC_HARD_OBSTACLE`) in config,
  metadata, debug output, and navigation semantics.
- Milestone 3: seed-reproducible procedural placement, water/shore/obstacle
  distance queries, and occupancy-grid plus A* start-to-goal validation.
- Milestone 4: smooth 1-3 m bathymetry, predominantly 1.8-2.3 m, with narrow
  corridors blended toward about 2.1 m and depth-aware vegetation placement.
- Milestone 5: swimming birds (`BIRD`, `DYNAMIC_WILDLIFE`) with separate physical
  and disturbance radii, valid waypoint motion, and debug routes.
- Milestone 6: an approximately 3 m target motor vessel (`TARGET_VESSEL`,
  `DYNAMIC_VESSEL`) with simple geometry and smooth waypoint traffic scenarios.
- Milestone 7: `mvp`, `easy`, `medium`, and `hard` presets plus targeted visual
  and usability refinement.

Later debug output should grow into geometry, scenario, navigation, and depth
views. Generate only information used by developers or consumers.
