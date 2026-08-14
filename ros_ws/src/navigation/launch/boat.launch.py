"""Launch the physical Karaburan boat stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    declarations += instrument_argument_declarations(lidar_default='true')
    declarations += [
        DeclareLaunchArgument(
            'fix_status_override_enabled',
            default_value='true',
            description=(
                'Change invalid GPS status values to STATUS_FIX while relaying '
                '/fix to /fix/valid'
            ),
        ),
        DeclareLaunchArgument(
            'motor_serial_port',
            default_value='/dev/ttyS0',
            description='Serial device connected to the motor controller',
        ),
        DeclareLaunchArgument(
            'motor_baud_rate',
            default_value='115200',
            description='Motor controller serial baud rate',
        ),
    ]

    mpu9250_config = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'mpu9250.yaml',
    )

    return LaunchDescription(declarations + [
        _include('gpsd_client', 'gpsd_client-launch.py'),
        Node(
            package='navigation',
            executable='fix_status_override_node',
            name='fix_status_override_node',
            output='screen',
            parameters=[{
                'override_invalid_status': LaunchConfiguration(
                    'fix_status_override_enabled'
                ),
            }],
        ),
        _include('boatcontrol', 'boatcontrol.launch.py', {
            'serial_port': LaunchConfiguration('motor_serial_port'),
            'baud_rate': LaunchConfiguration('motor_baud_rate'),
        }),
        Node(
            package='mpu9250',
            executable='mpu9250',
            name='mpu9250',
            output='screen',
            parameters=[mpu9250_config],
            remappings=[('/imu', '/imu/data')],
        ),
        _include('navigation', 'nav2_stack.launch.py', {'use_sim_time': 'false'}),
        _include('navigation', 'measurement_instruments.launch.py', instrument_arguments),
        _include('navigation', 'storage.launch.py', record_arguments),
    ])
