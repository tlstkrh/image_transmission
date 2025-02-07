from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(__file__), '../config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='image_transmission',      
            executable='stream_publish', 
            name='stream_publish',
            output='screen',
            parameters=[config_file]
        )
    ])
