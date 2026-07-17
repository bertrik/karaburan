"""Launch the physical Karaburan boat stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from navigation.launch_arguments import (
    instrument_argument_declarations,
    instrument_launch_arguments,
    recording_argument_declarations,
    recording_launch_arguments,
)


def _include(package_name, launch_file, launch_arguments=None):
    package_dir = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    record_arguments = recording_launch_arguments(
        bag_prefix='karaburan', use_sim_time='false'
    )
    instrument_arguments = instrument_launch_arguments()
    declarations = recording_argument_declarations('/data/karaburan/bags')
    declarations += instrument_argument_declarations()

    return LaunchDescription(declarations + [
        _include('gpsd_client', 'gpsd_client-launch.py'),
        Node(
            package='navigation',
            executable='fix_status_override_node',
            name='fix_status_override_node',
            output='screen',
        ),
        _include('boatcontrol', 'boatcontrol.launch.py'),
        _include('mpu9250', 'mpu9250.launch.py'),
        _include('navigation', 'nav2_stack.launch.py', {'use_sim_time': 'false'}),
        _include('navigation', 'measurement_instruments.launch.py', instrument_arguments),
        _include('navigation', 'storage.launch.py', record_arguments),
    ])
