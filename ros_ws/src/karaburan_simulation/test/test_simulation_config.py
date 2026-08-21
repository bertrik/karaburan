from pathlib import Path


PACKAGE_DIR = Path(__file__).parents[1]


def test_simulation_owns_model_world_and_control_node():
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    launch = (PACKAGE_DIR / 'launch' / 'sim.launch.py').read_text()
    model = (PACKAGE_DIR / 'models' / 'karaburan_boat.sdf').read_text()
    world = (PACKAGE_DIR / 'worlds' / 'world.sdf').read_text()

    assert "glob('models/*.sdf')" in setup
    assert "glob('worlds/*.sdf')" in setup
    assert 'simcontrol_node = karaburan_simulation.simcontrolnode:main' in setup
    assert "get_package_share_directory('karaburan_simulation')" in launch
    assert "'headless'" in launch
    assert "'with_rviz'" in launch
    assert "'with_map'" in launch
    assert '<pose>0 0 0.155 0 0 0</pose>' in model
    assert '<gz_frame_id>lidar_link</gz_frame_id>' in model
    assert '<latitude_deg>52.018599</latitude_deg>' in world
    assert '<longitude_deg>4.708720</longitude_deg>' in world
