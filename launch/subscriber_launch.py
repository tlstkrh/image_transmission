from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    subscriber_node = Node(
        package='image_transmission',
        executable='subscribe',
        name='subscribe',
        output='screen'
    )
    
    return LaunchDescription([subscriber_node])

