"""Optional physical measurement instruments for Karaburan.

All instruments default to disabled so this launch file is safe on systems without
attached hardware, including the simulator and CI containers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('temperature_enabled', default_value='false'),
        DeclareLaunchArgument('temperature_sensor_id', default_value='28.7AAB46D42000'),
        DeclareLaunchArgument('sonar_enabled', default_value='false'),
        DeclareLaunchArgument('sonar_device', default_value='D3:01:01:02:2F:C6'),
        DeclareLaunchArgument('lidar_enabled', default_value='false'),
        DeclareLaunchArgument('lidar_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('tof_enabled', default_value='false'),
        DeclareLaunchArgument('tof_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('bt785_enabled', default_value='false'),
        DeclareLaunchArgument('bt785_device', default_value='D3:01:01:02:2F:C6'),
        Node(
            package='tempreader',
            executable='tempreaderNode',
            name='tempreader',
            output='screen',
            condition=IfCondition(LaunchConfiguration('temperature_enabled')),
            parameters=[{
                'sensorId': LaunchConfiguration('temperature_sensor_id'),
                'topic': '/temperature',
            }],
        ),
        Node(
            package='sonar',
            executable='sonar_node',
            name='sonar',
            output='screen',
            condition=IfCondition(LaunchConfiguration('sonar_enabled')),
            parameters=[{
                'device': LaunchConfiguration('sonar_device'),
                'topic': '/sonar',
                'frame_id': 'sonar',
            }],
        ),
        Node(
            package='lidar',
            executable='lidar_node',
            name='lidar',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lidar_enabled')),
            parameters=[{
                'device': LaunchConfiguration('lidar_device'),
                'topic': '/scan',
                'frame_id': 'laser',
            }],
        ),
        Node(
            package='vl53l0x',
            executable='vl53l0x_node',
            name='vl53l0x',
            output='screen',
            condition=IfCondition(LaunchConfiguration('tof_enabled')),
            parameters=[{
                'topic': '/tof/distance',
                'frame_id': 'tof_link',
                'rate_hz': ParameterValue(LaunchConfiguration('tof_rate_hz'), value_type=float),
            }],
        ),
        Node(
            package='bt785',
            executable='bt785_node',
            name='bt785',
            output='screen',
            condition=IfCondition(LaunchConfiguration('bt785_enabled')),
            parameters=[{
                'device': LaunchConfiguration('bt785_device'),
                'topic': '/bt785',
                'frame_id': 'bt785',
            }],
        ),
    ])
