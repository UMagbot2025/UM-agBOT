from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Use the correct model path that actually exists in your system
    model_path = '/workspace/models/agbot_yolo_nano.onnx'
    
    return LaunchDescription([
        # Vision node with mock camera
        Node(
            package='vision',
            executable='egg_detector',
            name='egg_detection',
            parameters=[{
                'model_path': model_path,
                'confidence_threshold': 0.4,
                'min_box_area': 500.0,
                'input_size': 640,
                'use_basler': False,
                'use_mock': True,
                'test_images_dir': '/workspace/test_images'
            }],
            output='screen'
        ),
        
        # Navigation node with mock Arduino
        Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_controller',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 9600,
                'use_arduino': True,
                'use_mock': True
            }],
            output='screen'
        )
    ])