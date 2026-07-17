"""Configurable rosbag2/MCAP recording for Karaburan."""

from datetime import datetime, timezone
from pathlib import Path
import re

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

PROFILES = {
    'minimal': [
        '/fix',
        '/fix/valid',
        '/temperature',
        '/sonar',
        '/tof/distance',
        '/bt785',
    ],
    'navigation': [
        '/fix',
        '/fix/valid',
        '/imu/data',
        '/odometry/filtered',
        '/odometry/gps',
        '/cmd_vel',
        '/tf',
        '/tf_static',
        '/temperature',
        '/sonar',
        '/tof/distance',
        '/bt785',
    ],
}


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_recorder(context):
    if not _as_bool(LaunchConfiguration('enabled').perform(context)):
        return [LogInfo(msg='MCAP recording is disabled')]

    profile = LaunchConfiguration('profile').perform(context).strip().lower()
    if profile not in (*PROFILES, 'full'):
        raise ValueError(
            f"Unknown recording profile '{profile}'; choose minimal, navigation or full"
        )

    topics = list(PROFILES['navigation' if profile == 'full' else profile])
    include_scan = profile == 'full' or _as_bool(
        LaunchConfiguration('include_scan').perform(context)
    )
    if include_scan:
        topics.append('/scan')

    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    if use_sim_time:
        topics.append('/clock')

    extra_topics = LaunchConfiguration('extra_topics').perform(context)
    topics.extend(topic.strip() for topic in extra_topics.split(',') if topic.strip())
    topics = list(dict.fromkeys(topics))

    output_dir = Path(LaunchConfiguration('output_dir').perform(context)).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    prefix = LaunchConfiguration('bag_prefix').perform(context).strip()
    safe_prefix = re.sub(r'[^A-Za-z0-9_.-]+', '-', prefix).strip('-') or 'karaburan'
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    bag_path = (output_dir / f'{safe_prefix}_{timestamp}').resolve()

    command = [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--storage-preset-profile',
        LaunchConfiguration('storage_preset').perform(context),
        '--output', str(bag_path),
    ]

    max_duration = int(LaunchConfiguration('max_bag_duration').perform(context))
    max_size = int(LaunchConfiguration('max_bag_size').perform(context))
    if max_duration > 0:
        command.extend(['--max-bag-duration', str(max_duration)])
    if max_size > 0:
        command.extend(['--max-bag-size', str(max_size)])
    if use_sim_time:
        command.append('--use-sim-time')
    command.extend(topics)

    delay = float(LaunchConfiguration('start_delay').perform(context))
    return [
        TimerAction(
            period=delay,
            actions=[
                LogInfo(msg=f"Recording MCAP profile '{profile}' to {bag_path}"),
                LogInfo(msg=f"Recording topics: {', '.join(topics)}"),
                ExecuteProcess(cmd=command, output='screen'),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument(
            'profile',
            default_value='navigation',
            description='minimal, navigation or full; full includes /scan',
        ),
        DeclareLaunchArgument('include_scan', default_value='false'),
        DeclareLaunchArgument(
            'extra_topics',
            default_value='',
            description='Comma-separated additional topic names',
        ),
        DeclareLaunchArgument('output_dir', default_value='./bags'),
        DeclareLaunchArgument('bag_prefix', default_value='karaburan'),
        DeclareLaunchArgument('storage_preset', default_value='zstd_fast'),
        DeclareLaunchArgument(
            'max_bag_duration',
            default_value='900',
            description='Rotate after this many seconds; 0 disables duration rotation',
        ),
        DeclareLaunchArgument(
            'max_bag_size',
            default_value='2147483648',
            description='Rotate after this many bytes; 0 disables size rotation',
        ),
        DeclareLaunchArgument('start_delay', default_value='5.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_recorder),
    ])
