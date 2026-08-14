from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing.actions


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    nodes = [
        Node(
            package='boatcontrol',
            executable='boatcontrol_node',
            name='boatcontrol_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': ParameterValue(baud_rate, value_type=int),
            }],
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyS0',
            description='Serial device connected to the motor controller',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Motor controller serial baud rate',
        ),
    ] + nodes + [
        launch_testing.actions.ReadyToTest()
    ])
