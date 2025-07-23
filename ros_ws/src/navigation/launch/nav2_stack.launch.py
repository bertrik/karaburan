from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    controller_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'controller_server.yaml'
    )
    behavior_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'behavior_server.yaml'
    )
    map_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'maps',
        'empty.yml'
    )
    ekf_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'ekf.yaml'
    )
    navsat_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'navsat.yaml'
    )
    nodes = [
        Node(
            package='navigation',
            executable='fix_status_override_node',
            name='fix_status_override_node',
            output='screen'
        ),
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='',
            parameters=[{'use_sim_time': False}],
        ),
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            parameters=[{
                'use_sim_time': False,
                'planner_frequency': 0.5,
                'expected_planner_frequency': 0.5
            }],
        ),
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            parameters=[controller_yaml],
        ),
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            parameters=[{
                'yaml_filename': map_yaml,
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
            executable='my_navigation_node', # De entrypoint (zoals in setup.py -> console_scripts)
            name='my_navigation_node1',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace='',
            output='screen',
            parameters=[behavior_yaml],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ ekf_yaml ],
            remappings=[('/odometry/filtered', '/odometry/filtered')]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[ navsat_yaml ],
            remappings=[('/fix', '/fix/valid')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }]
        ),
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
