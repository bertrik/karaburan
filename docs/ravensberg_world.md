# Ravensberg simulation world

The Ravensberg MVP is a deterministic, metre-scale Gazebo Sim world for early
autonomous navigation development. It intentionally uses inexpensive geometry
and a debug-first workflow. Later milestones can add semantic obstacles and
bathymetry without replacing the source geometry or the existing boat.

## Quick start

Build and source the ROS workspace, then generate and inspect the scenario
without starting Gazebo:

```bash
cd ros_ws
colcon build --packages-select karaburan_simulation
source install/setup.bash
ros2 run karaburan_simulation generate_ravensberg_scenario \
  --debug-only \
  --output-dir /tmp/ravensberg-mvp
```

The command writes deterministic metadata and an SVG geometry map. Start the
complete scenario with:

```bash
ros2 launch karaburan_simulation ravensberg.launch.py
```

For a simulation server without Gazebo or RViz windows:

```bash
ros2 launch karaburan_simulation ravensberg.launch.py \
  headless:=true with_rviz:=false
```

Generated launch artifacts default to
`~/.ros/karaburan/ravensberg_mvp`. Use `output_dir:=...` to select another
location. The launch file generates the world first and passes the configured
start pose to the existing standard simulator and boat spawn flow.

## Data flow

`config/ravensberg_mvp.json` is the single source for fixed geometry, geographic
origin, seed, ownship start, and goal. The generator validates that start and
goal are in water and outside all islands, then produces:

- an SDF world for Gazebo;
- compact JSON scenario metadata for tools and later navigation validation; and
- an SVG geometry view for fast inspection without Gazebo.

Do not edit files in `karaburan_simulation/generated` by hand. Regenerate them
from the configuration. The committed output is an inspectable baseline; a
normal launch writes a fresh copy below `~/.ros`.

## Geometry and coordinate system

The world uses a local ENU frame in metres:

- positive x points east;
- positive y points north;
- positive z points up;
- the waterline is `z = 0`; and
- the flat MVP bottom is at approximately `z = -2.0 m`.

The world origin is `52.0436549 N, 4.7416484 E`. The source is OpenStreetMap
water relation
[`14671863`](https://www.openstreetmap.org/relation/14671863), retrieved on
2026-08-21 under the [ODbL 1.0](https://www.openstreetmap.org/copyright).
Coordinates were projected locally to metres. The outer water ring was reduced
with a 30 m simplification tolerance, and the twelve largest inner land rings
were retained with an 8 m tolerance. This is navigation-test geometry, not a
survey product.

The debug map treats the water polygon minus the retained islands as navigable
water. Gazebo uses the same points for the water visual, flat bottom, island
collisions, and segmented shoreline collisions. This avoids independent map
and simulator outlines drifting apart.

## Scenario semantics

Land and islands use:

```text
semantic_type: LAND
navigation_class: HARD_OBSTACLE
```

The metadata separates scenario semantics and navigation classification from
Gazebo collision and visual geometry. The seed is present from the MVP onward,
although Milestone 1 contains no random placement.

## Existing boat integration

The MVP reuses the existing `karaburan_boat.sdf` without changing its buoyancy,
hydrodynamics, sensors, thrusters, or `/cmd_vel` control path. The Ravensberg
launch file delegates to `sim.launch.py` after generation. Consequently,
existing manual-control and Nav2 commands continue to apply.

## Performance choices

The generated fixed world contains three static models: water, flat bottom, and
land. The current source geometry produces 37 collision shapes and 38 visuals.
The outer shoreline uses a small set of box segments; islands use one extruded
polygon visual and one conservative box collision each. There are no per-vertex
entities, terrain meshes, waves, or dynamic scenery. Box collisions are used
because Gazebo Harmonic's buoyancy system does not support polyline collision
geometry.

Generation time and the deterministic complexity counts are printed by the
command. Runtime real-time factor must be observed on a machine with Gazebo
Harmonic installed; the standard ROS-only CI image intentionally does not
contain Gazebo.

The Milestone 1 runtime check used Gazebo Sim 8.11.0 in a headless container
with software rendering. With the existing boat and sensors spawned, 50 samples
produced a mean real-time factor of 0.801 for Ravensberg and 0.831 for the old
empty-water baseline. The approximately 3.6% difference is acceptable for the
MVP and is small relative to the observed per-sample scheduling variation.
Publishing forward `/cmd_vel` for ten seconds moved the boat from x = -120.00 m
to x = -117.45 m.

## Known limitations

- The shoreline and twelve islands are deliberately simplified. Numerous small
  legakkers and narrow channels in the source relation are omitted.
- The bottom is flat and collision detail is optimized for test performance,
  not visual realism.
- The water is a static translucent visual; it has no waves or current.
- Route existence is not validated yet. That belongs to Milestone 3.
- The optional Leaflet UI still has the legacy WGS84 origin. Keep `with_map`
  disabled for Ravensberg until that UI becomes scenario-origin aware.
- SVG island labels can overlap in the dense southern part of the lake; the
  polygons and IDs remain machine-readable in metadata.
- The existing boat model emits SDF parser warnings for its Gazebo-specific
  `gz_frame_id` sensor elements. These warnings predate the Ravensberg world and
  do not prevent its IMU, NavSat, or LiDAR systems from loading.
