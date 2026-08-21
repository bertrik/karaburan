from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = [
        Node(
            package='karaburan_simulation',
            executable='simcontrol_node',
            name='simcontrol_node',
            output='screen',
        )
    ]

    return LaunchDescription(nodes)
