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


def test_navigation_uses_external_slam_lifecycle_and_hybrid_planner():
    planner = (CONFIG_DIR / 'planner_server.yaml').read_text()
    boat_bt = (CONFIG_DIR / 'navigate_to_pose_boat.xml').read_text()
    launch = (LAUNCH_DIR / 'nav2_stack.launch.py').read_text()
    managed_nodes_start = launch.index("'node_names': [")
    managed_nodes_end = launch.index(']', managed_nodes_start)
    managed_nodes = launch[managed_nodes_start:managed_nodes_end]

    assert 'nav2_smac_planner::SmacPlannerHybrid' in planner
    assert 'motion_model_for_search: "REEDS_SHEPP"' in planner
    assert 'minimum_turning_radius: 5.0' in planner
    assert 'reverse_penalty: 1.05' in planner
    assert 'change_penalty: 0.5' in planner
    assert 'retrospective_penalty: 0.0' in planner
    assert 'non_straight_penalty: 1.05' in planner
    assert 'cost_penalty: 4.0' in planner
    assert 'analytic_expansion_max_length: 50.0' in planner
    assert "'use_lifecycle_manager': 'true'" in launch
    assert managed_nodes.index("'slam_toolbox',") < managed_nodes.index("'planner_server',")
    assert "'default_nav_to_pose_bt_xml': navigate_to_pose_bt" in launch
    assert boat_bt.index('<BackUp ') < boat_bt.index('<Spin ')
    assert 'backup_dist="1.5"' in boat_bt
    assert 'backup_speed="0.20"' in boat_bt
    assert '<RateController hz="0.5">' in boat_bt


def test_global_costmap_fits_raspberry_pi_memory_budget():
    planner = (CONFIG_DIR / 'planner_server.yaml').read_text()

    assert 'width: 100' in planner
    assert 'height: 100' in planner
    assert 'resolution: 0.2' in planner


def test_lidar_obstacles_feed_both_costmaps_and_controller():
    controller = (CONFIG_DIR / 'controller_server.yaml').read_text()
    collision_monitor = (CONFIG_DIR / 'collision_monitor.yaml').read_text()
    planner = (CONFIG_DIR / 'planner_server.yaml').read_text()
    launch = (LAUNCH_DIR / 'nav2_stack.launch.py').read_text()

    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in controller
    assert 'observation_sources: scan' in controller
    assert 'topic: /scan' in controller
    assert 'marking: true' in controller
    assert 'clearing: true' in controller
    assert 'max_obstacle_height: 2.0' in controller
    assert 'RegulatedPurePursuitController' in controller
    assert 'nav2_controller::SimpleProgressChecker' in controller
    assert 'use_rotate_to_heading: false' in controller
    assert 'allow_reversing: true' in controller
    assert 'lookahead_dist: 2.0' in controller
    assert 'min_lookahead_dist: 1.0' in controller
    assert 'max_lookahead_dist: 3.0' in controller
    assert 'use_collision_detection: false' in controller
    assert controller.count('footprint: "[[0.28, 0.15]') == 1
    assert 'inflation_radius: 1.5' in controller
    assert 'plugins: ["obstacle_layer", "inflation_layer"]' in planner
    assert 'observation_sources: scan' in planner
    assert 'topic: /scan' in planner
    assert 'marking: true' in planner
    assert 'clearing: true' in planner
    assert 'max_obstacle_height: 2.0' in planner
    assert planner.count('footprint: "[[0.28, 0.15]') == 1
    assert 'inflation_radius: 3.0' in planner
    assert 'cost_scaling_factor: 1.0' in planner
    assert 'cmd_vel_in_topic: /cmd_vel_nav' in collision_monitor
    assert 'cmd_vel_out_topic: /cmd_vel' in collision_monitor
    assert 'action_type: approach' in collision_monitor
    assert 'time_before_collision: 3.0' in collision_monitor
    assert 'polygons: ["PolygonStop"]' in collision_monitor
    assert 'points: "[[0.20, 0.10]' in collision_monitor
    assert launch.count("remappings=[('/cmd_vel', '/cmd_vel_planner')]") == 2
    assert "executable='reverse_arc_controller'" in launch
    assert "'collision_monitor'," in launch


def test_reverse_arc_is_fast_stateful_and_geometry_driven():
    config = (CONFIG_DIR / 'reverse_arc_controller.yaml').read_text()
    setup = (PACKAGE_DIR / 'setup.py').read_text()

    assert 'radius: 2.0' in config
    assert 'heading_change: 1.5707963267948966' in config
    assert 'reverse_speed: 1.0' in config
    assert 'forward_speed: 0.25' in config
    assert 'minimum_trigger_angular: 0.05' in config
    assert 'heading_tolerance: 0.01' in config
    assert 'maneuver_timeout: 25.0' in config
    assert 'arc_sample_step: 0.20' in config
    assert 'footprint_half_length: 0.30' in config
    assert 'footprint_half_width: 0.17' in config
    controller = (
        PACKAGE_DIR / 'navigation' / 'reverse_arc_controller.py'
    ).read_text()
    assert 'OccupancyGrid, Odometry, Path' in controller
    assert "'/local_costmap/costmap_raw'" in controller
    assert "Path, '/plan'" in controller
    assert 'port_score = self.arc_cost(-1.0)' in controller
    assert 'starboard_score = self.arc_cost(1.0)' in controller
    assert 'self.required_plan_generation = self.plan_generation + 1' in controller
    assert 'reverse_arc_controller = navigation.reverse_arc_controller:main' in setup
