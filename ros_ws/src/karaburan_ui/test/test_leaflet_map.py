import math
from pathlib import Path

from karaburan_ui.leaflet_map import map_point_to_wgs84


PACKAGE_DIR = Path(__file__).parents[1]


def test_leaflet_map_owns_web_asset_and_navigation_topics():
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    map_node = (PACKAGE_DIR / 'karaburan_ui' / 'leaflet_map.py').read_text()
    map_launch = (PACKAGE_DIR / 'launch' / 'leaflet_map.launch.py').read_text()
    map_html = (PACKAGE_DIR / 'web' / 'leaflet_map.html').read_text()

    assert "glob('web/*')" in setup
    assert 'leaflet_map = karaburan_ui.leaflet_map:main' in setup
    assert 'ORIGIN_LATITUDE = 52.018599' in map_launch
    assert 'ORIGIN_LONGITUDE = 4.708720' in map_launch
    assert "package='karaburan_ui'" in map_launch
    assert "get_package_share_directory('karaburan_ui')" in map_node
    assert "NavSatFix, '/fix/valid'" in map_node
    assert "NavPath, '/plan'" in map_node
    assert 'tile.openstreetmap.org' in map_html
    assert 'OpenStreetMap contributors' in map_html


def test_map_point_conversion_preserves_origin_and_moves_east():
    origin = (52.018599, 4.708720)
    assert map_point_to_wgs84(0.0, 0.0, *origin) == [*origin]

    latitude, longitude = map_point_to_wgs84(10.0, 0.0, *origin)
    assert math.isclose(latitude, origin[0])
    assert longitude > origin[1]
