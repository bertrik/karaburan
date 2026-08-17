from pathlib import Path


CONFIG_DIR = Path(__file__).parents[1] / 'config'
LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def test_runtime_rates_and_ranges_match_boat_hardware():
    ekf = (CONFIG_DIR / 'ekf.yaml').read_text()
    slam = (CONFIG_DIR / 'slam_params.yaml').read_text()

    assert 'frequency: 20.0' in ekf
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
