# Start the Karaburan Gazebo world, navigation stack, and simulated boat.
#
# Usage:
#   ros2 launch karaburan_simulation sim.launch.py
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from navigation.launch_arguments import (
    instrument_argument_declarations,
    instrument_launch_arguments,
    recording_argument_declarations,
    recording_launch_arguments,
)


def generate_launch_description():
    simulation_dir = get_package_share_directory('karaburan_simulation')
    simcontrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, 'launch', 'simcontrol.launch.py')
        )
    )

    # --- Launch arguments ---
    world_sdf = LaunchConfiguration('world_sdf')
    model_sdf = LaunchConfiguration('model_sdf')
    world_name = LaunchConfiguration('world_name')
    entity_name = LaunchConfiguration('entity_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')

    # Build parameter_bridge arguments (GZ -> ROS for sensors)
    ns = LaunchConfiguration('ns')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    with_map = LaunchConfiguration('with_map')
    headless = LaunchConfiguration('headless')
    with_rviz = LaunchConfiguration('with_rviz')
    nav_dir = get_package_share_directory('navigation')
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'nav2_stack.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_enabled': 'false',
        }.items()
    )
    measurement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'measurement_instruments.launch.py')
        ),
        launch_arguments=instrument_launch_arguments().items()
    )
    storage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'storage.launch.py')
        ),
        launch_arguments=recording_launch_arguments(
            bag_prefix='karaburan-sim', use_sim_time='true'
        ).items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        ])),
        launch_arguments={
            'gz_args': [world_sdf, ' -r']
        }.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        ])),
        launch_arguments={
            'gz_args': [world_sdf, ' -r -s']
        }.items(),
        condition=IfCondition(headless),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz],
        condition=IfCondition(with_rviz),
    )
    delayed_map = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_dir, 'launch', 'leaflet_map.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ],
        condition=IfCondition(with_map),
    )

    return LaunchDescription([
        # Paths / args
        DeclareLaunchArgument(
            'world_sdf',
            default_value=os.path.join(simulation_dir, 'worlds', 'world.sdf'),
            description="Path to world (SDF). 'empty.sdf' also works."
        ),
        DeclareLaunchArgument('world_name', default_value='ocean'),
        DeclareLaunchArgument(
            'model_sdf',
            default_value=os.path.join(
                simulation_dir, 'models', 'karaburan_boat.sdf'),
            description='Path to the SDF model with hydrodynamics plugin'
        ),
        DeclareLaunchArgument('entity_name', default_value='karaburan'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='-0.015'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument(
            'with_map',
            default_value='false',
            description='Start the Leaflet OpenStreetMap navigation view',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Gazebo server without its GUI',
        ),
        DeclareLaunchArgument(
            'with_rviz',
            default_value='true',
            description='Start RViz after the simulator is ready',
        ),
        # Launch args for the GZ - ROS2 bridge
        DeclareLaunchArgument('ns', default_value='', description='ROS namespace for the bridge'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='GZ/ROS topic for IMU'),
        DeclareLaunchArgument('gps_topic', default_value='/fix/valid',
                              description='GZ/ROS topic for GPS/NavSat'),
        DeclareLaunchArgument('lidar_topic', default_value='/scan',
                              description='LiDAR LaserScan topic'),
        DeclareLaunchArgument(
            'left_topic',
            default_value='/model/karaburan/joint/left_prop_joint/cmd_thrust',
            description='Gazebo left propeller thrust topic'
        ),
        DeclareLaunchArgument(
            'right_topic',
            default_value='/model/karaburan/joint/right_prop_joint/cmd_thrust',
            description='Gazebo right propeller thrust topic'
        ),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Base frame id'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link',
                              description='IMU frame id (must match gz_frame_id in SDF)'),
        DeclareLaunchArgument('gps_frame', default_value='gps_link',
                              description='GPS frame id (must match gz_frame_id in SDF)'),
        # Optional MCAP recording and physical instruments (HIL only in sim)
        *recording_argument_declarations('./bags'),
        *instrument_argument_declarations(),

        gazebo,
        gazebo_headless,
        nav_launch,
        simcontrol,
        measurement_launch,
        storage_launch,

        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('ros_gz_sim'), 'launch', 'gz_spawn_model.launch.py'
                    ])),
                    launch_arguments={
                        'world': world_name,
                        'entity_name': entity_name,
                        'file': model_sdf,
                        'x': x, 'y': y, 'z': z,
                        'R': R, 'P': P, 'Y': Y
                    }.items()
                ),
                delayed_rviz,
                delayed_map,
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_parameter_bridge',
                    namespace=ns,
                    output='screen',
                    arguments=[
                        PythonExpression(
                            ['"', imu_topic, '" + \'@sensor_msgs/msg/Imu[gz.msgs.IMU\'']),
                        PythonExpression(
                            ['"', gps_topic, '" + \'@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat\'']),
                        PythonExpression(['"', lidar_topic,
                                         '" + \'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan\'']),
                        PythonExpression(['"', left_topic,
                                         '" + \'@std_msgs/msg/Float64]gz.msgs.Double\'']),
                        PythonExpression(['"', right_topic,
                                         '" + \'@std_msgs/msg/Float64]gz.msgs.Double\'']),
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                    ],
                    remappings=[
                        (left_topic, '/left'),
                        (right_topic, '/right'),
                    ],
                )
            ]
        ),
    ])
