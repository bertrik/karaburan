"""Generate the deterministic Ravensberg MVP world and debug artifacts."""

import argparse
from dataclasses import dataclass
from html import escape
import json
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET


SEMANTIC_LAND = 'LAND'
NAVIGATION_HARD_OBSTACLE = 'HARD_OBSTACLE'


@dataclass(frozen=True)
class GenerationResult:
    """Paths and deterministic complexity metrics for one generation run."""

    metadata_path: Path
    debug_path: Path
    world_path: Path | None
    static_entity_count: int
    collision_count: int
    visual_count: int


def default_config_path():
    """Return the installed default MVP configuration path."""
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory('karaburan_simulation'))
    return share / 'config' / 'ravensberg_mvp.json'


def load_config(path):
    """Load and validate a Ravensberg scenario configuration."""
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    validate_config(config)
    return config


def _polygon_area(points):
    return 0.5 * sum(
        point[0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * point[1]
        for index, point in enumerate(points)
    )


def _orientation(first, second, third):
    value = (
        (second[1] - first[1]) * (third[0] - second[0])
        - (second[0] - first[0]) * (third[1] - second[1])
    )
    if math.isclose(value, 0.0, abs_tol=1e-9):
        return 0
    return 1 if value > 0 else 2


def _on_segment(first, point, second):
    return (
        min(first[0], second[0]) <= point[0] <= max(first[0], second[0])
        and min(first[1], second[1]) <= point[1] <= max(first[1], second[1])
    )


def _segments_intersect(first, second, third, fourth):
    orientations = (
        _orientation(first, second, third),
        _orientation(first, second, fourth),
        _orientation(third, fourth, first),
        _orientation(third, fourth, second),
    )
    if orientations[0] != orientations[1] and orientations[2] != orientations[3]:
        return True
    checks = (
        orientations[0] == 0 and _on_segment(first, third, second),
        orientations[1] == 0 and _on_segment(first, fourth, second),
        orientations[2] == 0 and _on_segment(third, first, fourth),
        orientations[3] == 0 and _on_segment(third, second, fourth),
    )
    return any(checks)


def _validate_polygon(name, points):
    if len(points) < 3:
        raise ValueError(f'{name} must contain at least three points')
    if any(len(point) != 2 for point in points):
        raise ValueError(f'{name} points must contain x and y')
    if len({tuple(point) for point in points}) != len(points):
        raise ValueError(f'{name} contains duplicate points')
    if math.isclose(_polygon_area(points), 0.0, abs_tol=1e-6):
        raise ValueError(f'{name} has zero area')
    count = len(points)
    for first_index in range(count):
        first = points[first_index]
        second = points[(first_index + 1) % count]
        for third_index in range(first_index + 1, count):
            if third_index in (
                first_index,
                (first_index + 1) % count,
                (first_index - 1) % count,
            ):
                continue
            if first_index == 0 and third_index == count - 1:
                continue
            third = points[third_index]
            fourth = points[(third_index + 1) % count]
            if _segments_intersect(first, second, third, fourth):
                raise ValueError(f'{name} intersects itself')


def point_in_polygon(point, polygon):
    """Return whether a point is inside or on the edge of a polygon."""
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _orientation(previous, point, current) == 0 and _on_segment(
                previous, point, current):
            return True
        crosses = (current[1] > point[1]) != (previous[1] > point[1])
        if crosses:
            edge_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < edge_x:
                inside = not inside
        previous = current
    return inside


def is_navigable_water(config, point):
    """Return whether a map point is water and not one of the MVP islands."""
    if not point_in_polygon(point, config['water_outline']):
        return False
    return not any(
        point_in_polygon(point, island['points'])
        for island in config['islands']
    )


def validate_config(config):
    """Validate geometry, identifiers, scale, start, and goal."""
    if config.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    if not isinstance(config.get('seed'), int):
        raise ValueError('seed must be an integer')
    if not re.fullmatch(r'[a-z][a-z0-9_]*', config.get('scenario', '')):
        raise ValueError('scenario must be a lowercase identifier')
    coordinates = config['coordinates']
    if coordinates.get('orientation') != 'ENU' or coordinates.get('unit') != 'm':
        raise ValueError('coordinates must use metre-scale ENU')
    _validate_polygon('water_outline', config['water_outline'])
    identifiers = [island['id'] for island in config['islands']]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('island IDs must be unique')
    for island in config['islands']:
        _validate_polygon(island['id'], island['points'])
        if not all(
                point_in_polygon(point, config['water_outline'])
                for point in island['points']):
            raise ValueError(f"{island['id']} is not inside the water outline")
    start = config['ownship']['start']
    goal = config['goal']
    if not is_navigable_water(config, [start['x'], start['y']]):
        raise ValueError('ownship start must be in navigable water')
    if not is_navigable_water(config, [goal['x'], goal['y']]):
        raise ValueError('goal must be in navigable water')
    world = config['world']
    if world['bottom_depth_m'] <= 0.0:
        raise ValueError('bottom_depth_m must be positive')
    if world['shoreline_width_m'] <= 0.0:
        raise ValueError('shoreline_width_m must be positive')


def _subelement(parent, tag, text=None, attributes=None):
    element = ET.SubElement(parent, tag, attributes or {})
    if text is not None:
        element.text = str(text)
    return element


def _add_plugin(world, filename, name):
    _subelement(world, 'plugin', attributes={
        'filename': filename,
        'name': name,
    })


def _add_polyline(parent, points, height):
    geometry = _subelement(parent, 'geometry')
    polyline = _subelement(geometry, 'polyline')
    _subelement(polyline, 'height', _number(height))
    for point in points:
        _subelement(polyline, 'point', f'{_number(point[0])} {_number(point[1])}')


def _add_material(visual, ambient, diffuse):
    material = _subelement(visual, 'material')
    _subelement(material, 'ambient', ambient)
    _subelement(material, 'diffuse', diffuse)


def _number(value):
    return f'{value:.6f}'.rstrip('0').rstrip('.') if isinstance(value, float) else str(value)


def _world_tree(config):
    sdf = ET.Element('sdf', {'version': '1.10'})
    world_config = config['world']
    world = _subelement(sdf, 'world', attributes={'name': world_config['name']})
    physics = _subelement(world, 'physics', attributes={
        'name': 'default',
        'type': 'dart',
    })
    _subelement(physics, 'max_step_size', '0.001')
    _subelement(physics, 'real_time_update_rate', '1000')
    _add_plugin(world, 'gz-sim-physics-system', 'gz::sim::systems::Physics')
    _add_plugin(
        world,
        'gz-sim-user-commands-system',
        'gz::sim::systems::UserCommands',
    )
    _add_plugin(
        world,
        'gz-sim-scene-broadcaster-system',
        'gz::sim::systems::SceneBroadcaster',
    )
    buoyancy = _subelement(world, 'plugin', attributes={
        'filename': 'gz-sim-buoyancy-system',
        'name': 'gz::sim::systems::Buoyancy',
    })
    graded = _subelement(buoyancy, 'graded_buoyancy')
    _subelement(graded, 'default_density', '1000')
    density = _subelement(graded, 'density_change')
    _subelement(density, 'above_depth', '0')
    _subelement(density, 'density', '1')
    sensors = _subelement(world, 'plugin', attributes={
        'filename': 'gz-sim-sensors-system',
        'name': 'gz::sim::systems::Sensors',
    })
    _subelement(sensors, 'render_engine', 'ogre2')
    _add_plugin(world, 'gz-sim-imu-system', 'gz::sim::systems::Imu')
    _add_plugin(world, 'gz-sim-navsat-system', 'gz::sim::systems::NavSat')
    origin = config['coordinates']['origin']
    spherical = _subelement(world, 'spherical_coordinates')
    _subelement(spherical, 'surface_model', 'EARTH_WGS84')
    _subelement(spherical, 'world_frame_orientation', 'ENU')
    _subelement(spherical, 'latitude_deg', origin['latitude_deg'])
    _subelement(spherical, 'longitude_deg', origin['longitude_deg'])
    _subelement(spherical, 'elevation', origin['elevation_m'])
    _subelement(spherical, 'heading_deg', '0')
    _add_light(world)
    _add_water(world, config)
    _add_bottom(world, config)
    _add_land(world, config)
    ET.indent(sdf, space='  ')
    return ET.ElementTree(sdf)


def _add_light(world):
    light = _subelement(world, 'light', attributes={
        'name': 'sun',
        'type': 'directional',
    })
    _subelement(light, 'cast_shadows', 'true')
    _subelement(light, 'pose', '0 0 1000 0 0 0')
    _subelement(light, 'diffuse', '0.8 0.8 0.8 1')
    _subelement(light, 'specular', '0.2 0.2 0.2 1')
    _subelement(light, 'direction', '-0.5 0.3 -1')


def _add_water(world, config):
    model = _subelement(world, 'model', attributes={'name': 'water_surface'})
    _subelement(model, 'static', 'true')
    link = _subelement(model, 'link', attributes={'name': 'water_visual'})
    _subelement(link, 'gravity', 'false')
    visual = _subelement(link, 'visual', attributes={'name': 'water'})
    _subelement(visual, 'pose', '0 0 -0.02 0 0 0')
    _add_polyline(visual, config['water_outline'], 0.02)
    _add_material(visual, '0.08 0.32 0.52 0.75', '0.10 0.45 0.70 0.75')
    _subelement(visual, 'transparency', '0.25')


def _add_bottom(world, config):
    depth = config['world']['bottom_depth_m']
    outline = config['water_outline']
    minimum_x = min(point[0] for point in outline)
    maximum_x = max(point[0] for point in outline)
    minimum_y = min(point[1] for point in outline)
    maximum_y = max(point[1] for point in outline)
    model = _subelement(world, 'model', attributes={'name': 'flat_bottom'})
    _subelement(model, 'static', 'true')
    link = _subelement(model, 'link', attributes={'name': 'bottom'})
    centre_x = (minimum_x + maximum_x) / 2.0
    centre_y = (minimum_y + maximum_y) / 2.0
    pose = '{} {} {} 0 0 0'.format(
        _number(centre_x),
        _number(centre_y),
        _number(-depth - 0.1),
    )
    collision = _subelement(link, 'collision', attributes={'name': 'bottom_collision'})
    _subelement(collision, 'pose', pose)
    geometry = _subelement(collision, 'geometry')
    box = _subelement(geometry, 'box')
    _subelement(
        box,
        'size',
        '{} {} 0.2'.format(
            _number(maximum_x - minimum_x + 100.0),
            _number(maximum_y - minimum_y + 100.0),
        ),
    )
    visual = _subelement(link, 'visual', attributes={'name': 'bottom_visual'})
    _subelement(visual, 'pose', f'0 0 {_number(-depth - 0.2)} 0 0 0')
    _add_polyline(visual, outline, 0.2)
    _add_material(visual, '0.18 0.14 0.08 1', '0.25 0.20 0.10 1')


def _add_land(world, config):
    world_config = config['world']
    depth = world_config['bottom_depth_m']
    above = world_config['land_height_above_water_m']
    height = depth + above
    width = world_config['shoreline_width_m']
    centre_z = (-depth + above) / 2.0
    model = _subelement(world, 'model', attributes={'name': 'land'})
    _subelement(model, 'static', 'true')
    link = _subelement(model, 'link', attributes={'name': 'land_geometry'})
    outline = config['water_outline']
    for index, first in enumerate(outline):
        second = outline[(index + 1) % len(outline)]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy) + width
        pose = '{} {} {} 0 0 {}'.format(
            _number((first[0] + second[0]) / 2.0),
            _number((first[1] + second[1]) / 2.0),
            _number(centre_z),
            _number(math.atan2(dy, dx)),
        )
        size = f'{_number(length)} {_number(width)} {_number(height)}'
        name = f'shore_{index + 1:02d}'
        collision = _subelement(link, 'collision', attributes={'name': name})
        _subelement(collision, 'pose', pose)
        geometry = _subelement(collision, 'geometry')
        box = _subelement(geometry, 'box')
        _subelement(box, 'size', size)
        visual = _subelement(link, 'visual', attributes={'name': name})
        _subelement(visual, 'pose', pose)
        geometry = _subelement(visual, 'geometry')
        box = _subelement(geometry, 'box')
        _subelement(box, 'size', size)
        _add_material(visual, '0.18 0.28 0.10 1', '0.28 0.42 0.15 1')
    for island in config['islands']:
        minimum_x = min(point[0] for point in island['points'])
        maximum_x = max(point[0] for point in island['points'])
        minimum_y = min(point[1] for point in island['points'])
        maximum_y = max(point[1] for point in island['points'])
        collision_pose = '{} {} {} 0 0 0'.format(
            _number((minimum_x + maximum_x) / 2.0),
            _number((minimum_y + maximum_y) / 2.0),
            _number(centre_z),
        )
        collision = _subelement(
            link,
            'collision',
            attributes={'name': f"{island['id']}_collision"},
        )
        _subelement(collision, 'pose', collision_pose)
        geometry = _subelement(collision, 'geometry')
        box = _subelement(geometry, 'box')
        _subelement(
            box,
            'size',
            '{} {} {}'.format(
                _number(maximum_x - minimum_x),
                _number(maximum_y - minimum_y),
                _number(height),
            ),
        )
        visual = _subelement(
            link,
            'visual',
            attributes={'name': f"{island['id']}_visual"},
        )
        _subelement(visual, 'pose', f'0 0 {_number(-depth)} 0 0 0')
        _add_polyline(visual, island['points'], height)
        _add_material(visual, '0.16 0.30 0.09 1', '0.25 0.45 0.12 1')


def _complexity(config):
    collision_count = 1 + len(config['water_outline']) + len(config['islands'])
    visual_count = 2 + len(config['water_outline']) + len(config['islands'])
    return {
        'static_entity_count': 3,
        'collision_count': collision_count,
        'visual_count': visual_count,
    }


def _metadata(config):
    complexity = _complexity(config)
    objects = [{
        'id': 'shoreline',
        'semantic_type': SEMANTIC_LAND,
        'navigation_class': NAVIGATION_HARD_OBSTACLE,
        'visual': {
            'type': 'boundary',
            'points': config['water_outline'],
        },
        'collision': {
            'type': 'box_segments',
            'width_m': config['world']['shoreline_width_m'],
        },
        'navigation': {
            'type': 'outside_water_outline',
        },
    }]
    for island in config['islands']:
        minimum_x = min(point[0] for point in island['points'])
        maximum_x = max(point[0] for point in island['points'])
        minimum_y = min(point[1] for point in island['points'])
        maximum_y = max(point[1] for point in island['points'])
        objects.append({
            'id': island['id'],
            'semantic_type': SEMANTIC_LAND,
            'navigation_class': NAVIGATION_HARD_OBSTACLE,
            'visual': {
                'type': 'polygon',
                'points': island['points'],
            },
            'collision': {
                'type': 'axis_aligned_box',
                'minimum': [minimum_x, minimum_y],
                'maximum': [maximum_x, maximum_y],
            },
            'navigation': {
                'type': 'polygon',
                'points': island['points'],
            },
        })
    return {
        'schema_version': 1,
        'scenario': config['scenario'],
        'seed': config['seed'],
        'coordinates': config['coordinates'],
        'source': config['source'],
        'world': config['world'],
        'ownship': config['ownship'],
        'goal': config['goal'],
        'navigable_area': {
            'water_outline': config['water_outline'],
            'excluded_island_ids': [item['id'] for item in config['islands']],
        },
        'objects': objects,
        'complexity': complexity,
    }


def _svg_points(points, transform):
    return ' '.join(
        f'{transform(point)[0]:.1f},{transform(point)[1]:.1f}'
        for point in points
    )


def _debug_svg(config):
    width = 1000
    height = 1200
    padding = 70
    points = config['water_outline']
    minimum_x = min(point[0] for point in points)
    maximum_x = max(point[0] for point in points)
    minimum_y = min(point[1] for point in points)
    maximum_y = max(point[1] for point in points)
    scale = min(
        (width - 2 * padding) / (maximum_x - minimum_x),
        (height - 2 * padding) / (maximum_y - minimum_y),
    )
    map_width = (maximum_x - minimum_x) * scale
    map_height = (maximum_y - minimum_y) * scale
    offset_x = (width - map_width) / 2.0
    offset_y = (height - map_height) / 2.0

    def transform(point):
        return (
            offset_x + (point[0] - minimum_x) * scale,
            height - offset_y - (point[1] - minimum_y) * scale,
        )

    water = _svg_points(points, transform)
    start = config['ownship']['start']
    start_xy = transform([start['x'], start['y']])
    goal = config['goal']
    goal_xy = transform([goal['x'], goal['y']])
    origin_xy = transform([0.0, 0.0])
    scale_length = 200.0 * scale
    source_url = escape(config['source']['url'])
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#d8d2a8"/>',
        f'<polygon points="{water}" fill="#70add8" stroke="#174b65" '
        'stroke-width="3"/>',
    ]
    for island in config['islands']:
        polygon = _svg_points(island['points'], transform)
        centre_x = sum(point[0] for point in island['points']) / len(island['points'])
        centre_y = sum(point[1] for point in island['points']) / len(island['points'])
        label = transform([centre_x, centre_y])
        identifier = escape(island['id'])
        lines.extend([
            f'<polygon points="{polygon}" fill="#6f8f3b" '
            'stroke="#344d20" stroke-width="2"/>',
            f'<text x="{label[0]:.1f}" y="{label[1]:.1f}" '
            'class="island">{}</text>'.format(identifier),
        ])
    lines.extend([
        '<style>text{font-family:Arial,sans-serif;fill:#12222b}'
        '.label{font-size:18px;font-weight:bold}'
        '.island{font-size:11px;text-anchor:middle}</style>',
        f'<circle cx="{start_xy[0]:.1f}" cy="{start_xy[1]:.1f}" r="9" '
        'fill="#20a65a" stroke="white" stroke-width="3"/>',
        f'<text x="{start_xy[0] + 14:.1f}" y="{start_xy[1] + 6:.1f}" '
        'class="label">START</text>',
        f'<circle cx="{goal_xy[0]:.1f}" cy="{goal_xy[1]:.1f}" r="9" '
        'fill="#d63b32" stroke="white" stroke-width="3"/>',
        f'<text x="{goal_xy[0] + 14:.1f}" y="{goal_xy[1] + 6:.1f}" '
        'class="label">GOAL</text>',
        f'<path d="M {origin_xy[0] - 10:.1f} {origin_xy[1]:.1f} '
        f'H {origin_xy[0] + 10:.1f} M {origin_xy[0]:.1f} '
        f'{origin_xy[1] - 10:.1f} V {origin_xy[1] + 10:.1f}" '
        'stroke="#111" stroke-width="2"/>',
        f'<text x="{origin_xy[0] + 12:.1f}" y="{origin_xy[1] - 12:.1f}">'
        'WORLD ORIGIN (0, 0)</text>',
        f'<path d="M 80 1100 H {80 + scale_length:.1f}" stroke="#111" '
        'stroke-width="6"/>',
        '<text x="80" y="1085" class="label">200 m</text>',
        '<path d="M 900 110 V 55 M 900 55 L 888 78 M 900 55 L 912 78" '
        'stroke="#111" stroke-width="4" fill="none"/>',
        '<text x="893" y="45" class="label">N</text>',
        '<text x="45" y="38" class="label">Ravensberg MVP - geometry view</text>',
        '<text x="45" y="62">Blue: navigable water | Green/tan: land | '
        'Scale: 1 unit = 1 metre</text>',
        f'<text x="45" y="1180">Seed {config["seed"]} | Source: '
        f'<a href="{source_url}">OpenStreetMap relation '
        f'{config["source"]["osm_id"]}</a></text>',
        '</svg>',
        '',
    ])
    return '\n'.join(lines)


def generate_scenario(config_path, output_dir, debug_only=False):
    """Generate deterministic metadata, SVG debug map, and optional SDF."""
    config = load_config(config_path)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    prefix = f"seed_{config['seed']:04d}"
    metadata_path = output / f'{prefix}_metadata.json'
    debug_path = output / f'{prefix}_geometry.svg'
    world_path = None if debug_only else output / f"{config['scenario']}.sdf"
    metadata_path.write_text(
        json.dumps(_metadata(config), indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    debug_path.write_text(_debug_svg(config), encoding='utf-8')
    if world_path is not None:
        tree = _world_tree(config)
        tree.write(world_path, encoding='unicode', xml_declaration=True)
    complexity = _complexity(config)
    return GenerationResult(
        metadata_path=metadata_path,
        debug_path=debug_path,
        world_path=world_path,
        **complexity,
    )


def main(arguments=None):
    """Run the scenario generator command line interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('generated') / 'ravensberg_mvp',
    )
    parser.add_argument(
        '--debug-only',
        action='store_true',
        help='Generate metadata and the debug map, but no Gazebo SDF.',
    )
    options = parser.parse_args(arguments)
    config_path = options.config or default_config_path()
    started = time.perf_counter()
    result = generate_scenario(
        config_path,
        options.output_dir,
        debug_only=options.debug_only,
    )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    print(f'Generated metadata: {result.metadata_path}')
    print(f'Generated debug map: {result.debug_path}')
    if result.world_path is not None:
        print(f'Generated Gazebo world: {result.world_path}')
    print(
        'Complexity: '
        f'{result.static_entity_count} static entities, '
        f'{result.collision_count} collisions, '
        f'{result.visual_count} visuals'
    )
    print(f'Generation duration: {elapsed_ms:.1f} ms')


if __name__ == '__main__':
    main()
