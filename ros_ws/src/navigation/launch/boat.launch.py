"""Launch the physical Karaburan boat stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package_name, launch_file, launch_arguments=None):
    package_dir = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    record_arguments = {
        'enabled': LaunchConfiguration('record_enabled'),
        'profile': LaunchConfiguration('record_profile'),
        'include_scan': LaunchConfiguration('record_include_scan'),
        'extra_topics': LaunchConfiguration('record_extra_topics'),
        'output_dir': LaunchConfiguration('record_output_dir'),
        'bag_prefix': 'karaburan',
        'max_bag_duration': LaunchConfiguration('record_max_bag_duration'),
        'max_bag_size': LaunchConfiguration('record_max_bag_size'),
        'start_delay': LaunchConfiguration('record_start_delay'),
        'use_sim_time': 'false',
    }
    instrument_arguments = {
        'temperature_enabled': LaunchConfiguration('with_temperature'),
        'temperature_sensor_id': LaunchConfiguration('temperature_sensor_id'),
        'sonar_enabled': LaunchConfiguration('with_sonar'),
        'sonar_device': LaunchConfiguration('sonar_device'),
        'lidar_enabled': LaunchConfiguration('with_lidar'),
        'lidar_device': LaunchConfiguration('lidar_device'),
        'tof_enabled': LaunchConfiguration('with_tof'),
        'tof_rate_hz': LaunchConfiguration('tof_rate_hz'),
        'bt785_enabled': LaunchConfiguration('with_bt785'),
        'bt785_device': LaunchConfiguration('bt785_device'),
    }

    declarations = [
        DeclareLaunchArgument('record_enabled', default_value='false'),
        DeclareLaunchArgument('record_profile', default_value='navigation'),
        DeclareLaunchArgument('record_include_scan', default_value='false'),
        DeclareLaunchArgument('record_extra_topics', default_value=''),
        DeclareLaunchArgument('record_output_dir', default_value='/data/karaburan/bags'),
        DeclareLaunchArgument('record_max_bag_duration', default_value='900'),
        DeclareLaunchArgument('record_max_bag_size', default_value='2147483648'),
        DeclareLaunchArgument('record_start_delay', default_value='5.0'),
        DeclareLaunchArgument('with_temperature', default_value='false'),
        DeclareLaunchArgument('temperature_sensor_id', default_value='28.7AAB46D42000'),
        DeclareLaunchArgument('with_sonar', default_value='false'),
        DeclareLaunchArgument('sonar_device', default_value='D3:01:01:02:2F:C6'),
        DeclareLaunchArgument('with_lidar', default_value='false'),
        DeclareLaunchArgument('lidar_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('with_tof', default_value='false'),
        DeclareLaunchArgument('tof_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('with_bt785', default_value='false'),
        DeclareLaunchArgument('bt785_device', default_value='D3:01:01:02:2F:C6'),
    ]

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
