from pathlib import Path


PACKAGE_DIR = Path(__file__).parents[1]


def test_bringup_owns_production_orchestration_and_calibration():
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    package = (PACKAGE_DIR / 'package.xml').read_text()
    boat = (PACKAGE_DIR / 'launch' / 'boat.launch.py').read_text()
    instruments = (
        PACKAGE_DIR / 'launch' / 'measurement_instruments.launch.py'
    ).read_text()
    storage = (PACKAGE_DIR / 'launch' / 'storage.launch.py').read_text()
    imu = (PACKAGE_DIR / 'config' / 'mpu9250.yaml').read_text()

    assert "glob('config/*.yaml')" in setup
    assert "glob('launch/*.launch.py')" in setup
    assert "get_package_share_directory('karaburan_bringup')" in boat
    assert "_include('navigation', 'nav2_stack.launch.py'" in boat
    assert '<exec_depend>navigation</exec_depend>' in package
    assert "package='lidar'" in instruments
    assert "package='tempreader'" in instruments
    assert "'navigation': [" in storage
    assert 'frame_id: imu_link' in imu
