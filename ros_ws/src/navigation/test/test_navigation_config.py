from pathlib import Path


PACKAGE_DIR = Path(__file__).parents[1]
CONFIG_DIR = PACKAGE_DIR / 'config'
LAUNCH_DIR = PACKAGE_DIR / 'launch'


def test_runtime_rates_and_ranges_match_boat_hardware():
    ekf = (CONFIG_DIR / 'ekf.yaml').read_text()
    slam = (CONFIG_DIR / 'slam_params.yaml').read_text()

    assert 'frequency: 10.0' in ekf
    assert 'scan_queue_size: 1' in slam
    assert 'max_laser_range: 10.0' in slam


def test_navigation_uses_external_slam_lifecycle_and_navfn():
    planner = (CONFIG_DIR / 'planner_server.yaml').read_text()
    launch = (LAUNCH_DIR / 'nav2_stack.launch.py').read_text()
    managed_nodes_start = launch.index("'node_names': [")
    managed_nodes_end = launch.index(']', managed_nodes_start)
    managed_nodes = launch[managed_nodes_start:managed_nodes_end]

    assert 'nav2_navfn_planner::NavfnPlanner' in planner
    assert "'use_lifecycle_manager': 'true'" in launch
    assert managed_nodes.index("'slam_toolbox',") < managed_nodes.index("'planner_server',")


def test_global_costmap_fits_raspberry_pi_memory_budget():
    planner = (CONFIG_DIR / 'planner_server.yaml').read_text()

    assert 'width: 100' in planner
    assert 'height: 100' in planner
    assert 'resolution: 0.5' in planner


def test_lidar_obstacles_feed_both_costmaps_and_controller():
    controller = (CONFIG_DIR / 'controller_server.yaml').read_text()
    planner = (CONFIG_DIR / 'planner_server.yaml').read_text()

    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in controller
    assert 'observation_sources: scan' in controller
    assert 'topic: /scan' in controller
    assert 'marking: true' in controller
    assert 'clearing: true' in controller
    assert 'max_obstacle_height: 2.0' in controller
    assert 'use_collision_detection: true' in controller
    assert 'enable_collision_checking' not in controller
    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in planner
    assert 'observation_sources: scan' in planner
    assert 'topic: /scan' in planner
    assert 'marking: true' in planner
    assert 'clearing: true' in planner
    assert 'max_obstacle_height: 2.0' in planner


def test_leaflet_map_matches_simulation_origin_and_topics():
    map_node = (PACKAGE_DIR / 'navigation' / 'leaflet_map.py').read_text()
    map_launch = (LAUNCH_DIR / 'leaflet_map.launch.py').read_text()
    map_html = (PACKAGE_DIR / 'web' / 'leaflet_map.html').read_text()
    sim_launch = (LAUNCH_DIR / 'sim.launch.py').read_text()
    world = (CONFIG_DIR / 'world.sdf').read_text()

    assert 'ORIGIN_LATITUDE = 52.018599' in map_launch
    assert 'ORIGIN_LONGITUDE = 4.708720' in map_launch
    assert '<latitude_deg>52.018599</latitude_deg>' in world
    assert '<longitude_deg>4.708720</longitude_deg>' in world
    assert "NavSatFix, '/fix/valid'" in map_node
    assert "NavPath, '/plan'" in map_node
    assert 'tile.openstreetmap.org' in map_html
    assert 'OpenStreetMap contributors' in map_html
    assert "'with_map'" in sim_launch
