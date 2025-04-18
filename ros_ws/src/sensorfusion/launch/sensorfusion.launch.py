from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions

def generate_launch_description():
    nodes = [
        Node(
            package='sensorfusion',
            executable='sensorfusion_node',
            name='sensorfusion_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
