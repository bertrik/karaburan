from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions

def generate_launch_description():
    nodes = [
        Node(
            package='boatcontrol',
            executable='simcontrol_node',
            name='simcontrol_node',
            output='screen',
        )
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
