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
    assert len(config['islands']) == 12
    start = config['ownship']['start']
    assert is_navigable_water(config, [start['x'], start['y']])
    assert is_navigable_water(config, [config['goal']['x'], config['goal']['y']])


def test_generation_is_byte_stable_and_world_is_valid_xml(tmp_path):
    first = generate_scenario(CONFIG_PATH, tmp_path / 'first')
    second = generate_scenario(CONFIG_PATH, tmp_path / 'second')

    assert first.metadata_path.read_bytes() == second.metadata_path.read_bytes()
    assert first.debug_path.read_bytes() == second.debug_path.read_bytes()
    assert first.world_path.read_bytes() == second.world_path.read_bytes()

    root = ET.parse(first.world_path).getroot()
    world = root.find("world[@name='ravensberg']")
    assert world is not None
    assert world.findtext('spherical_coordinates/latitude_deg') == '52.0436549'
    assert world.findtext('spherical_coordinates/longitude_deg') == '4.7416484'
    assert world.find("model[@name='flat_bottom']") is not None
    assert world.find("model[@name='land']") is not None


def test_metadata_separates_semantics_and_reports_complexity(tmp_path):
    result = generate_scenario(CONFIG_PATH, tmp_path)
    metadata = json.loads(result.metadata_path.read_text(encoding='utf-8'))

    assert metadata['seed'] == 1
    assert metadata['scenario'] == 'ravensberg_mvp'
    assert metadata['objects'][0]['semantic_type'] == 'LAND'
    assert metadata['objects'][0]['navigation_class'] == 'HARD_OBSTACLE'
    assert len({item['id'] for item in metadata['objects']}) == 13
    assert metadata['complexity'] == {
        'collision_count': 37,
        'static_entity_count': 3,
        'visual_count': 38,
    }
    assert result.collision_count == 37


def test_debug_only_does_not_write_a_gazebo_world(tmp_path):
    result = generate_scenario(CONFIG_PATH, tmp_path, debug_only=True)

    assert result.world_path is None
    assert result.metadata_path.is_file()
    assert result.debug_path.is_file()
    assert '<svg ' in result.debug_path.read_text(encoding='utf-8')
    assert not list(tmp_path.glob('*.sdf'))


def test_invalid_goal_is_rejected(tmp_path):
    config = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
    config['goal'] = {'x': 2000.0, 'y': 2000.0}
    invalid = tmp_path / 'invalid.json'
    invalid.write_text(json.dumps(config), encoding='utf-8')

    with pytest.raises(ValueError, match='goal must be in navigable water'):
        load_config(invalid)
