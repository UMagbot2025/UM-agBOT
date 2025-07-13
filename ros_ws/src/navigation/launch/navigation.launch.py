from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_controller',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 9600,
                'use_arduino': True
            }],
            output='screen'
        )
    ])