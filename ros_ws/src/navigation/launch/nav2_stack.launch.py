from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions

def generate_launch_description():
    nodes = [
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': '../maps/empty.yml',
                'use_sim_time': False
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='navigation',         # Je eigen package
            executable='navigation_node', # De entrypoint (zoals in setup.py -> console_scripts)
            name='navigation_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
