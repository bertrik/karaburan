from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ORIGIN_LATITUDE = 52.018599
ORIGIN_LONGITUDE = 4.708720


def generate_launch_description():
    open_browser = LaunchConfiguration('open_browser')
    port = LaunchConfiguration('port')
    url = ['http://127.0.0.1:', port]
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock'
        ),
        DeclareLaunchArgument(
            'open_browser', default_value='true',
            description='Open the Leaflet map in the default browser'
        ),
        DeclareLaunchArgument('port', default_value='8088'),
        Node(
            package='navigation',
            executable='leaflet_map',
            name='leaflet_map',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port': ParameterValue(port, value_type=int),
                'origin_latitude': ORIGIN_LATITUDE,
                'origin_longitude': ORIGIN_LONGITUDE,
            }],
            output='screen',
        ),
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=['python3', '-m', 'webbrowser', '-t', url],
                output='screen',
            )],
            condition=IfCondition(open_browser),
        ),
    ])
