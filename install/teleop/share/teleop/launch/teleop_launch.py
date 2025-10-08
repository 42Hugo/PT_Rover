from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop',
            executable='rover', 
            name='rover_node',
            namespace='rover',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_joy_node',
        )
    ])