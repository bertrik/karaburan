from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions

def generate_launch_description():
    nodes = [
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