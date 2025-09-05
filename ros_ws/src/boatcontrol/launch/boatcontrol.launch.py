from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions

def generate_launch_description():
    nodes = [
        Node(
            package='boatcontrol',
            executable='boatcontrol_node',
            name='boatcontrol_node',
            output='screen',
        )
    ]

    return LaunchDescription(nodes + [
        launch_testing.actions.ReadyToTest()
    ])
