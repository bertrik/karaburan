from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    boat_xacro = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config',
        'karaburan.xacro'
    ])
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        boat_xacro
    ])
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'publish_frequency': 1.0,
                         'use_sim_time': use_sim_time }],
            output='screen',
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
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='',
            parameters=[ bt_navigator_yaml, { 'use_sim_time': use_sim_time } ],
            output='screen'
        ),
        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            parameters=[
                controller_yaml, { 'use_sim_time': use_sim_time }
            ],
            arguments=['--ros-args', '--log-level', 'controller_server:=debug', '--log-level', 'regulated_pure_pursuit_controller:=debug'],
            output='log'
        ),
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            parameters=[
                planner_server_yaml, { 'use_sim_time': use_sim_time }
            ],
            output='log'
        ),
        LifecycleNode(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace='',
            output='screen',
            parameters=[behavior_yaml, { 'use_sim_time': use_sim_time }],
        ),
        LifecycleNode(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='',
            parameters=[{ 'use_sim_time': use_sim_time }],
            output='screen',
        ),
        LifecycleNode(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace='',
            output='screen',
            parameters=[{
                "use_sim_time": True,
                "smoother_plugins": ["simple_smoother"],
                "simple_smoother": {
                    "plugin": "nav2_smoother::SimpleSmoother",
                    "tolerance": 0.1
                }
            }]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace='',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time }],
            arguments=[
              '--ros-args',
              '--params-file', ekf_yaml,
            ],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[ navsat_yaml, { 'use_sim_time': use_sim_time } ],
            remappings=[
                ('/gps/fix', '/fix/valid'),
                ('/imu', '/imu/data')
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
                    'smoother_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),
    ]

    return LaunchDescription(
            [ DeclareLaunchArgument('use_sim_time', default_value='true', description='Gebruik /clock (true) of systeemklok (false)') ] +
            nodes + [
        launch_testing.actions.ReadyToTest()
    ])
