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
    simulation_script = (
        PACKAGE_DIR / 'scripts' / 'run_simulation_tests.sh').read_text()

    assert '<depend>action_msgs</depend>' in package
    assert '<depend>ament_index_python</depend>' in package
    assert '<exec_depend>karaburan_simulation</exec_depend>' in package
    assert "'headless': 'true'" in test_launch
    assert "'with_rviz': 'false'" in test_launch
    assert "'record_enabled': 'false'" in test_launch
    assert "'world_name': 'ocean'" in test_launch
    assert "'x': '0.0'" in test_launch
    assert "glob('scenarios/*.sdf')" in setup
    assert "glob('scripts/*.sh')" in setup
    assert 'karaburan_navigation_tests.maneuver_test_runner:main' in setup
    assert 'karaburan_navigation_tests.maneuver_report:main' in setup
    assert 'karaburan_navigation_tests.junit_html_report:main' in setup
    assert '<exec_depend>python3-junit2html</exec_depend>' in package
    assert "'follow_straight'" in runner
    assert "'planner_direct'" in runner
    assert "'obstacle_port'" in runner
    assert "'obstacle_starboard'" in runner
    assert (
        'setsid ros2 launch karaburan_navigation_tests '
        'maneuver_test.launch.py'
    ) in script
    assert 'RCUTILS_COLORIZED_OUTPUT=0' in script
    assert 'maneuver_test_report' in script
    assert 'timestamp="$(date +%Y%m%d%H%M%S)"' in simulation_script
    assert 'test-results-$timestamp' in simulation_script
    assert 'bash "$script_dir/run_maneuver_tests.sh"' in simulation_script
    assert '--packages-select karaburan_navigation_tests' in simulation_script
    assert (
        'karaburan_simulation karaburan_navigation_tests'
        not in simulation_script
    )
    assert 'colcon test' in simulation_script
    assert 'junit_html_report' in simulation_script
    test_world = (PACKAGE_DIR / 'scenarios' / 'maneuver_test_world.sdf').read_text()
    assert '<real_time_update_rate>1000</real_time_update_rate>' in test_world
    assert '<collision_detector>bullet</collision_detector>' in test_world
