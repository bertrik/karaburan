from pathlib import Path


PACKAGE_DIR = Path(__file__).parents[1]


def test_simulation_owns_model_world_and_control_node():
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    launch = (PACKAGE_DIR / 'launch' / 'sim.launch.py').read_text()
    model = (PACKAGE_DIR / 'models' / 'karaburan_boat.sdf').read_text()
    world = (PACKAGE_DIR / 'worlds' / 'world.sdf').read_text()

    assert "glob('models/*.sdf')" in setup
    assert "glob('worlds/*.sdf')" in setup
    assert "glob('config/*.json')" in setup
    assert 'generate_ravensberg_scenario' in setup
    assert 'simcontrol_node = karaburan_simulation.simcontrolnode:main' in setup
    assert "get_package_share_directory('karaburan_simulation')" in launch
    assert "'headless'" in launch
    assert "'with_rviz'" in launch
    assert "'with_map'" in launch
    assert '<pose>0 0 0.155 0 0 0</pose>' in model
    assert '<gz_frame_id>lidar_link</gz_frame_id>' in model
    assert '<latitude_deg>52.018599</latitude_deg>' in world
    assert '<longitude_deg>4.708720</longitude_deg>' in world


def test_ravensberg_launch_uses_generated_world_and_configured_start():
    launch = (
        PACKAGE_DIR / 'launch' / 'ravensberg.launch.py'
    ).read_text()

    assert "'scenario_config'" in launch
    assert "'output_dir'" in launch
    assert 'generate_scenario(config_path, output_dir)' in launch
    assert "'world_sdf': str(result.world_path)" in launch
    assert "'x': str(start['x'])" in launch


def test_gazebo_server_owns_world_before_optional_gui_connects():
    launch = (PACKAGE_DIR / 'launch' / 'sim.launch.py').read_text()

    assert "'gz_args': [world_sdf, ' -r -s']" in launch
    assert "'gz_args': '-g'" in launch
    assert 'actions=[gazebo_gui]' in launch
    assert 'condition=UnlessCondition(headless)' in launch
    assert "'gz_args': [world_sdf, ' -r']" not in launch
