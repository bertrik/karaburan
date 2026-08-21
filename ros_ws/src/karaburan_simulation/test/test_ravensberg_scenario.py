"""Tests for the deterministic Ravensberg MVP generator."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from karaburan_simulation.ravensberg_scenario import (
    generate_scenario,
    is_navigable_water,
    load_config,
)
import pytest


PACKAGE_DIR = Path(__file__).parents[1]
CONFIG_PATH = PACKAGE_DIR / 'config' / 'ravensberg_mvp.json'


def test_source_geometry_has_real_scale_and_valid_start_goal():
    config = load_config(CONFIG_PATH)
    outline = config['water_outline']
    width = max(point[0] for point in outline) - min(point[0] for point in outline)
    height = max(point[1] for point in outline) - min(point[1] for point in outline)

    assert config['source']['osm_id'] == 14671863
    assert config['coordinates']['unit'] == 'm'
    assert width > 1300.0
    assert height > 1700.0
    assert len(config['water_outline']) == 77
    assert len(config['shoreline_collision_outline']) < 40
    assert len(config['islands']) == 40
    assert config['world']['island_collision_count'] == 12
    assert min(item['source_area_m2'] for item in config['islands']) >= 250.0
    assert len({item['osm_way_id'] for item in config['islands']}) == 40
    start = config['ownship']['start']
    assert is_navigable_water(config, [start['x'], start['y']])
    assert is_navigable_water(config, [config['goal']['x'], config['goal']['y']])


def test_generation_is_byte_stable_and_world_is_valid_xml(tmp_path):
    first = generate_scenario(CONFIG_PATH, tmp_path / 'first')
    second = generate_scenario(CONFIG_PATH, tmp_path / 'second')

    assert first.metadata_path.read_bytes() == second.metadata_path.read_bytes()
    assert first.debug_path.read_bytes() == second.debug_path.read_bytes()
    assert (
        first.scenario_debug_path.read_bytes()
        == second.scenario_debug_path.read_bytes()
    )
    assert first.world_path.read_bytes() == second.world_path.read_bytes()

    root = ET.parse(first.world_path).getroot()
    world = root.find("world[@name='ravensberg']")
    assert world is not None
    assert world.findtext('spherical_coordinates/latitude_deg') == '52.0436549'
    assert world.findtext('spherical_coordinates/longitude_deg') == '4.7416484'
    assert world.find("model[@name='flat_bottom']") is not None
    assert world.find("model[@name='static_scenario_objects']") is not None
    land = world.find("model[@name='land']/link[@name='land_geometry']")
    assert land is not None
    assert len(world.findall('.//collision')) == first.collision_count
    assert len(world.findall('.//visual')) == first.visual_count


def test_metadata_separates_semantics_and_reports_complexity(tmp_path):
    result = generate_scenario(CONFIG_PATH, tmp_path)
    metadata = json.loads(result.metadata_path.read_text(encoding='utf-8'))

    assert metadata['seed'] == 1
    assert metadata['scenario'] == 'ravensberg_mvp'
    assert metadata['objects'][0]['semantic_type'] == 'LAND'
    assert metadata['objects'][0]['navigation_class'] == 'HARD_OBSTACLE'
    assert metadata['objects'][12]['collision']['type'] == 'axis_aligned_box'
    assert metadata['objects'][13]['collision']['type'] == 'none'
    assert len({item['id'] for item in metadata['objects']}) == 52
    assert metadata['complexity'] == {
        'collision_count': 46,
        'static_entity_count': 4,
        'visual_count': 77,
    }
    assert result.collision_count == 46


def test_static_scenario_objects_have_required_semantics_and_geometry(tmp_path):
    config = load_config(CONFIG_PATH)
    result = generate_scenario(CONFIG_PATH, tmp_path)
    metadata = json.loads(result.metadata_path.read_text(encoding='utf-8'))
    objects = metadata['objects'][41:]

    assert len(config['scenario_objects']) == 11
    assert [item['semantic_type'] for item in objects].count('REED_ZONE') == 3
    assert [item['semantic_type'] for item in objects].count(
        'LILY_PAD_FIELD') == 2
    assert [item['semantic_type'] for item in objects].count('PEAT_CHUNK') == 3
    assert [item['semantic_type'] for item in objects].count('BUOY') == 3
    assert all(item['navigation_class'] == 'AVOID_ZONE'
               for item in objects if item['semantic_type'] == 'REED_ZONE')
    assert all(item['navigation_class'] == 'STATIC_HARD_OBSTACLE'
               for item in objects
               if item['semantic_type'] in ('PEAT_CHUNK', 'BUOY'))
    assert all(item['collision']['shape'] == 'none'
               for item in objects
               if item['semantic_type'] == 'LILY_PAD_FIELD')
    assert all(item['collision']['shape'] != 'none'
               for item in objects
               if item['semantic_type'] != 'LILY_PAD_FIELD')


def test_debug_only_does_not_write_a_gazebo_world(tmp_path):
    result = generate_scenario(CONFIG_PATH, tmp_path, debug_only=True)

    assert result.world_path is None
    assert result.metadata_path.is_file()
    assert result.debug_path.is_file()
    assert result.scenario_debug_path.is_file()
    debug = result.debug_path.read_text(encoding='utf-8')
    scenario_debug = result.scenario_debug_path.read_text(encoding='utf-8')
    assert '<svg ' in debug
    assert 1 <= debug.count('class="island"') <= 12
    assert 'reed_01' in scenario_debug
    assert 'lily_02' in scenario_debug
    assert 'peat_03' in scenario_debug
    assert 'buoy_03' in scenario_debug
    assert scenario_debug.count('class="navigation-zone"') == 11
    assert not list(tmp_path.glob('*.sdf'))


def test_invalid_goal_is_rejected(tmp_path):
    config = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    config['goal'] = {'x': 2000.0, 'y': 2000.0}
    invalid = tmp_path / 'invalid.json'
    invalid.write_text(json.dumps(config), encoding='utf-8')

    with pytest.raises(ValueError, match='goal must be in navigable water'):
        load_config(invalid)


def test_scenario_object_extent_outside_water_is_rejected(tmp_path):
    config = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    config['scenario_objects'][0]['pose']['x'] = 2000.0
    invalid = tmp_path / 'invalid_object.json'
    invalid.write_text(json.dumps(config), encoding='utf-8')

    with pytest.raises(ValueError, match='extent must be in navigable water'):
        load_config(invalid)
