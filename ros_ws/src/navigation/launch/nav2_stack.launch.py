from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    controller_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'controller_server.yaml'
    )
    planner_server_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'planner_server.yaml'
    )
    behavior_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'behavior_server.yaml'
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
    bt_navigator_yaml = os.path.join(
        get_package_share_directory('navigation'),
        'config',
        'bt_navigator.yaml'
    )
    nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link'
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_gps',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'gps'
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base',
            output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link'
            ]
        ),
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
            parameters=[ bt_navigator_yaml ],
            output='screen'
        ),
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            parameters=[
                controller_yaml
            ],
            output='log'
        ),
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            parameters=[
                planner_server_yaml
            ],
            output='log'
        ),
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace='',
            output='screen',
            parameters=[behavior_yaml],
        ),
        LifecycleNode(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='',
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace='',
            output='screen',
            parameters=[],
            arguments=[
              '--ros-args',
              '--params-file', ekf_yaml,
            ],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[ navsat_yaml ],
            remappings=[
                ('/gps/fix', '/fix/valid'),
                ('/imu', '/imu/data'),
                ('/odometry/filtered', '/odometry/filtered'),
                ('/odometry/gps', '/odometry/gps')
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
