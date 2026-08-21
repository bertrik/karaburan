from pathlib import Path


PACKAGE_DIR = Path(__file__).parents[1]


def test_repeatable_maneuver_suite_is_installed_and_headless():
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    package = (PACKAGE_DIR / 'package.xml').read_text()
    test_launch = (PACKAGE_DIR / 'launch' / 'maneuver_test.launch.py').read_text()
    runner = (
        PACKAGE_DIR
        / 'karaburan_navigation_tests'
        / 'maneuver_test_runner.py'
    ).read_text()
    script = (PACKAGE_DIR / 'scripts' / 'run_maneuver_tests.sh').read_text()

    assert '<depend>action_msgs</depend>' in package
    assert '<depend>ament_index_python</depend>' in package
    assert '<exec_depend>karaburan_simulation</exec_depend>' in package
    assert "'headless': 'true'" in test_launch
    assert "'with_rviz': 'false'" in test_launch
    assert "'record_enabled': 'false'" in test_launch
    assert "glob('scenarios/*.sdf')" in setup
    assert "glob('scripts/*.sh')" in setup
    assert 'karaburan_navigation_tests.maneuver_test_runner:main' in setup
    assert "'follow_straight'" in runner
    assert "'obstacle_port'" in runner
    assert "'obstacle_starboard'" in runner
    assert (
        'setsid ros2 launch karaburan_navigation_tests '
        'maneuver_test.launch.py'
    ) in script
