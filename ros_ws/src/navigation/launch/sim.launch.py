# Start Gazebo world, load xacro -> /robot_description & spawn in sim.
#
# Usage:
#   ros2 launch navigation sim.launch.py \
#       xacro_file:=/path/karaburan.xacro \
#       world_sdf:=/path/world.sdf \
#       entity_name:=karaburan x:=0 y:=0 z:=0.15 with_gazebo:=true \
#       xacro_args:="arg1:=value1 arg2:=value2"
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    boatcontrol_dir = get_package_share_directory('boatcontrol')
    boatcontrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boatcontrol_dir, 'launch', 'simcontrol.launch.py')
        )
    )

    # --- Launch arguments ---
    xacro_file = LaunchConfiguration("xacro_file")
    world_sdf = LaunchConfiguration("world_sdf")
    model_sdf = LaunchConfiguration("model_sdf")
    world_name = LaunchConfiguration("world_name")
    entity_name = LaunchConfiguration("entity_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    R = LaunchConfiguration("R")
    P = LaunchConfiguration("P")
    Y = LaunchConfiguration("Y")
    with_gazebo = LaunchConfiguration("with_gazebo")
    xacro_args = LaunchConfiguration("xacro_args")

    # Build parameter_bridge arguments (GZ -> ROS for sensors)
    ns = LaunchConfiguration('ns')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    publish_static_tf = LaunchConfiguration('publish_static_tf')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    gps_frame = LaunchConfiguration('gps_frame') 
    nav_dir = get_package_share_directory('navigation')
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'nav2_stack.launch.py')
        )
    )

    return LaunchDescription([
        # Paths / args
        DeclareLaunchArgument(
            "xacro_file",
            default_value="navigation/config/karaburan.xacro",
            description="Path to .urdf.xacro file"
        ),
        DeclareLaunchArgument(
            "world_sdf",
            default_value="navigation/config/world.sdf",
            description="Path to world (SDF). 'empty.sdf' also works."
        ),
        DeclareLaunchArgument("world_name", default_value="ocean"),
        DeclareLaunchArgument(
            "model_sdf",
            default_value="navigation/config/karaburan_boat.sdf",
            description="Pad naar SDF-model met hydrodynamics plugin"
        ),
        DeclareLaunchArgument("entity_name", default_value="karaburan"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="-0.015"),
        DeclareLaunchArgument("R", default_value="0.0"),
        DeclareLaunchArgument("P", default_value="0.0"),
        DeclareLaunchArgument("Y", default_value="0.0"),
        DeclareLaunchArgument(
            "xacro_args", default_value="",
        ),
        # Launch args for the GZ - ROS2 bridge
        DeclareLaunchArgument('ns', default_value='', description='ROS namespace for the bridge'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data', description='GZ/ROS topic for IMU'),
        DeclareLaunchArgument('gps_topic', default_value='/fix/valid', description='GZ/ROS topic for GPS/NavSat'),
        DeclareLaunchArgument('lidar_topic', default_value='/scan', description='LiDAR LaserScan topic'),
        DeclareLaunchArgument('left_topic', default_value='/left', description='Left propellor topic'),
        DeclareLaunchArgument('right_topic', default_value='/right', description='Right propellor topic'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame id'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link', description='IMU frame id (must match gz_frame_id in SDF)'),
        DeclareLaunchArgument('gps_frame', default_value='gps_link', description='GPS frame id (must match gz_frame_id in SDF)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])),
            launch_arguments={
                "gz_args": world_sdf
            }.items()
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command([
                    FindExecutable(name="xacro"), " ",
                    xacro_file, " ",
                    xacro_args
                ])
            }]
        ),
        nav_launch,
        boatcontrol,

        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare("ros_gz_sim"), "launch", "gz_spawn_model.launch.py"
                    ])),
                    launch_arguments={
                        "world": world_name,
                        "entity_name": entity_name,
                        "file": model_sdf,
                        "x": x, "y": y, "z": z,
                        "R": R, "P": P, "Y": Y
                    }.items()
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_parameter_bridge',
                    namespace=ns,
                    output='screen',
                    arguments=[
                        PythonExpression(['"', imu_topic, '" + \'@sensor_msgs/msg/Imu[gz.msgs.IMU\'']),
                        PythonExpression(['"', gps_topic, '" + \'@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat\'']),
                        PythonExpression(['"', lidar_topic, '" + \'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan\'']),
                        PythonExpression(['"', left_topic, '" + \'@std_msgs/msg/Float64[gz.msgs.Float\'']),
                        PythonExpression(['"', right_topic, '" + \'@std_msgs/msg/Float64[gz.msgs.Float\'']),
                        '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                    ],
                )
            ]
        ),
    ])

