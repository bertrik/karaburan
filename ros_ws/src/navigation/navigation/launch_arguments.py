"""Shared launch arguments for optional instruments and MCAP recording."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def recording_argument_declarations(output_dir):
    """Declare the recording arguments exposed by a top-level launch file."""
    return [
        DeclareLaunchArgument('record_enabled', default_value='false'),
        DeclareLaunchArgument('record_profile', default_value='navigation'),
        DeclareLaunchArgument('record_include_scan', default_value='false'),
        DeclareLaunchArgument('record_extra_topics', default_value=''),
        DeclareLaunchArgument('record_output_dir', default_value=output_dir),
        DeclareLaunchArgument('record_max_bag_duration', default_value='900'),
        DeclareLaunchArgument('record_max_bag_size', default_value='2147483648'),
        DeclareLaunchArgument('record_start_delay', default_value='5.0'),
    ]


def recording_launch_arguments(*, bag_prefix, use_sim_time):
    """Map top-level recording arguments to storage.launch.py arguments."""
    return {
        'enabled': LaunchConfiguration('record_enabled'),
        'profile': LaunchConfiguration('record_profile'),
        'include_scan': LaunchConfiguration('record_include_scan'),
        'extra_topics': LaunchConfiguration('record_extra_topics'),
        'output_dir': LaunchConfiguration('record_output_dir'),
        'bag_prefix': bag_prefix,
        'max_bag_duration': LaunchConfiguration('record_max_bag_duration'),
        'max_bag_size': LaunchConfiguration('record_max_bag_size'),
        'start_delay': LaunchConfiguration('record_start_delay'),
        'use_sim_time': use_sim_time,
    }


def instrument_argument_declarations():
    """Declare optional physical-instrument arguments."""
    return [
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


def instrument_launch_arguments():
    """Map top-level instrument arguments to measurement_instruments.launch.py."""
    return {
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
