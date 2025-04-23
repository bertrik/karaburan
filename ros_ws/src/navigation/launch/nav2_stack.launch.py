from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    controller_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'controller_server.yaml'
    )
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
            parameters=[controller_yaml],
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
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
