from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Use the correct model path that actually exists in your system
    model_path = '/workspace/models/agbot_yolo_nano.onnx'
    
    return LaunchDescription([
        Node(
            package='vision',
            executable='egg_detector',  # This should match the name in setup.py entry_points
            name='egg_detection',
            parameters=[{
                'model_path': model_path,  # Using absolute path
                'confidence_threshold': 0.4,
                'min_box_area': 500.0,
                'input_size': 640,
                'use_basler': True
            }],
            output='screen'
        )
    ])