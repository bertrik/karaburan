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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Launch arguments ---
    xacro_file = LaunchConfiguration("xacro_file")
    world_sdf = LaunchConfiguration("world_sdf")
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
        DeclareLaunchArgument("entity_name", default_value="karaburan"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.15"),
        DeclareLaunchArgument("R", default_value="0.0"),
        DeclareLaunchArgument("P", default_value="0.0"),
        DeclareLaunchArgument("Y", default_value="0.0"),
        DeclareLaunchArgument(
            "with_gazebo", default_value="true",
        ),
        DeclareLaunchArgument(
            "xacro_args", default_value="",
        ),

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
                    "with_gazebo:=", with_gazebo, " ",
                    xacro_args
                ])
            }]
        ),

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
                        "topic": "/robot_description",
                        "x": x, "y": y, "z": z,
                        "R": R, "P": P, "Y": Y
                    }.items()
                )
            ]
        ),
    ])

