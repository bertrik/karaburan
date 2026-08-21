"""Generate and launch the Ravensberg MVP scenario."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from karaburan_simulation.ravensberg_scenario import generate_scenario, load_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_scenario(context):
    """Generate the selected scenario and include the standard simulator."""
    config_path = Path(LaunchConfiguration('scenario_config').perform(context))
    output_dir = Path(LaunchConfiguration('output_dir').perform(context))
    config = load_config(config_path)
    result = generate_scenario(config_path, output_dir)
    start = config['ownship']['start']
    simulation_dir = get_package_share_directory('karaburan_simulation')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world_sdf': str(result.world_path),
            'world_name': config['world']['name'],
            'x': str(start['x']),
            'y': str(start['y']),
            'z': str(start['z']),
            'Y': str(start['yaw']),
            'headless': LaunchConfiguration('headless'),
            'with_rviz': LaunchConfiguration('with_rviz'),
            'with_map': LaunchConfiguration('with_map'),
        }.items(),
    )]


def generate_launch_description():
    """Return the Ravensberg generation and simulation launch description."""
    simulation_dir = get_package_share_directory('karaburan_simulation')
    default_config = os.path.join(
        simulation_dir,
        'config',
        'ravensberg_mvp.json',
    )
    default_output = str(
        Path.home() / '.ros' / 'karaburan' / 'ravensberg_mvp'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_config',
            default_value=default_config,
            description='Ravensberg scenario JSON configuration.',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=default_output,
            description='Directory for generated world and debug artifacts.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Gazebo server.',
        ),
        DeclareLaunchArgument(
            'with_rviz',
            default_value='true',
            description='Start RViz after the simulator is ready.',
        ),
        DeclareLaunchArgument(
            'with_map',
            default_value='false',
            description='Start the Leaflet view (uses the legacy origin).',
        ),
        OpaqueFunction(function=_launch_scenario),
    ])
