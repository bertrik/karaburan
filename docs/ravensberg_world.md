# Ravensberg simulation world

The Ravensberg scenario is a deterministic, metre-scale Gazebo Sim world for
autonomous navigation development. Its first static scenario adds reeds,
lily-pad fields, peat chunks, and buoys to inexpensive fixed world geometry and
a debug-first workflow. Later milestones can add procedural placement and
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

The command writes deterministic metadata plus separate SVG geometry and
scenario maps. Start the committed Ravensberg baseline directly with:

```bash
ros2 launch karaburan_simulation sim.launch.py
```

This uses `generated/ravensberg_mvp.sdf`, the `ravensberg` world name, and the
configured MVP ownship start as its defaults. Explicit `world_sdf`,
`world_name`, and pose arguments can still select another world.

Generate a fresh copy from the scenario configuration and start it with:

```bash
ros2 launch karaburan_simulation ravensberg.launch.py
```

Both launch paths start the Gazebo server with the selected world first and attach
the optional GUI as a separate client. This avoids Gazebo's combined-process
`/gazebo/starting_world` handshake and its misleading ten-second GUI wait
warning. `headless:=true` uses the same server path and only omits the client.

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
origin, seed, ownship start, goal, and explicitly positioned static scenario
objects. The generator validates that start, goal, object centres, and complete
navigation extents are in navigable water, then produces:

- an SDF world for Gazebo;
- compact JSON scenario metadata for tools and later navigation validation; and
- an SVG geometry view for fast inspection without Gazebo; and
- an SVG scenario view with object IDs and exact navigation extents.

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
to 77 points with a 10 m visual and navigation tolerance. Forty of the 69 inner
land rings were retained by source area and simplified with a 3 m tolerance;
the smallest retained ring is approximately 609 m2. This is navigation-test
geometry, not a survey product. The repository import tool at
`ros_ws/src/karaburan_simulation/tools/import_ravensberg_osm.py` reproduces the
selection from the official OSM relation.

The debug map treats the water polygon minus the retained islands as navigable
water. Gazebo uses those points for the water visual, flat bottom, island
visuals, and navigation geometry. The physical outer shoreline deliberately
uses a separately generated 24-point contour with a 30 m tolerance. This keeps
the detailed map while avoiding 53 unnecessary collision and visual segments.

## Scenario semantics

Land and islands use:

```text
semantic_type: LAND
navigation_class: HARD_OBSTACLE
```

The metadata separates scenario semantics and navigation classification from
Gazebo collision and visual geometry. The static scenario uses:

| Object | Semantic type | Navigation class | Physical representation |
| --- | --- | --- | --- |
| Reed patch | `REED_ZONE` | `AVOID_ZONE` | One box collision per cluster |
| Lily-pad field | `LILY_PAD_FIELD` | `STATIC_SOFT_OBSTACLE` | No collision; navigation extent only |
| Peat chunk | `PEAT_CHUNK` | `STATIC_HARD_OBSTACLE` | One cylinder collision |
| Buoy | `BUOY` | `STATIC_HARD_OBSTACLE` | One cylinder collision |

Visual, collision, and navigation extents remain independently configurable.
The scenario SVG draws navigation extents with their safety margins, not merely
the smaller Gazebo visuals. The seed is present from the MVP onward, although
Milestone 2 deliberately uses explicit positions rather than random placement.

## Existing boat integration

The MVP reuses the existing `karaburan_boat.sdf` without changing its buoyancy,
hydrodynamics, sensors, thrusters, or `/cmd_vel` control path. The Ravensberg
launch file delegates to `sim.launch.py` after generation. Consequently,
existing manual-control and Nav2 commands continue to apply.

## Performance choices

The generated world contains four static models: water, flat bottom, land, and
one aggregate model for all static scenario objects. The current source
geometry and scenario produce 46 collision shapes and 77 visuals. The outer
shoreline uses a small set of box segments; islands use one extruded polygon
visual and one conservative box collision each. Reed and lily clusters do not
use per-stem or per-leaf entities. Lily fields are navigation-only obstacles,
while the three reeds, three peat chunks, and three buoys add only nine simple
collisions. Box collisions are used because Gazebo Harmonic's buoyancy system
does not support polyline collision geometry.

Generation time and the deterministic complexity counts are printed by the
command. Runtime real-time factor must be observed on a machine with Gazebo
Harmonic installed; the standard ROS-only CI image intentionally does not
contain Gazebo.

The refined Milestone 1 runtime check used Gazebo Sim 8.11.0 in a headless
container with software rendering. With the existing boat and sensors spawned,
50 samples produced a mean real-time factor of 0.932. An otherwise identical
12-island visual proxy produced 0.964, so the additional 28 island visuals
reduced the mean by approximately 3.3%, within the accepted approximately 10%
limit. Publishing forward `/cmd_vel` for ten seconds in the final world moved
the boat from x = -120.00 m to x = -116.60 m.

The Milestone 2 check used the same Gazebo Sim 8.11.0 software-rendered runtime,
world settings, boat, and sensors. Fifty samples measured mean real-time factors
of 0.957 for the Milestone 1 baseline and 0.896 for the static cluster scenario,
a decrease of approximately 6.4%. This remains within the previously accepted
approximately 10% change budget while adding all four static object types.

## Known limitations

- The shoreline and forty islands are deliberately simplified. Twenty-nine
  smaller OSM inner rings and some narrow source details remain omitted.
- All forty retained islands are visual and hard navigation obstacles. To stay
  within the MVP performance budget, only the twelve largest have physical
  Gazebo box collisions; the remaining 28 are explicitly marked with
  `collision.type: none` in metadata.
- The bottom is flat and collision detail is optimized for test performance,
  not visual realism.
- The water is a static translucent visual; it has no waves or current.
- Static scenario objects use explicit positions and abstract box/cylinder
  cluster visuals. They are intended for navigation tests, not botanical or
  photorealistic rendering.
- Lily-pad fields have no physical collision in the performance-oriented
  cluster representation. Consumers must respect their semantic navigation
  extents.
- Route existence is not validated yet. That belongs to Milestone 3.
- The optional Leaflet UI still has the legacy WGS84 origin. Keep `with_map`
  disabled for Ravensberg until that UI becomes scenario-origin aware.
- SVG labels are limited to the twelve largest candidates and labels less than
  40 pixels apart are suppressed. All polygons and IDs remain machine-readable
  in metadata.
- The existing boat model emits SDF parser warnings for its Gazebo-specific
  `gz_frame_id` sensor elements. These warnings predate the Ravensberg world and
  do not prevent its IMU, NavSat, or LiDAR systems from loading.
