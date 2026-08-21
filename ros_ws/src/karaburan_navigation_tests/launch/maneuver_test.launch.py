"""Launch the deterministic, accelerated manoeuvre test environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    test_share = get_package_share_directory('karaburan_navigation_tests')
    simulation_share = get_package_share_directory('karaburan_simulation')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_share, 'launch', 'sim.launch.py')
            ),
            launch_arguments={
                'world_sdf': os.path.join(
                    test_share, 'scenarios', 'maneuver_test_world.sdf'),
                'model_sdf': os.path.join(
                    simulation_share, 'models', 'karaburan_boat.sdf'),
                'headless': 'true',
                'with_rviz': 'false',
                'with_map': 'false',
                'record_enabled': 'false',
            }.items(),
        ),
    ])
