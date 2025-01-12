from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='switch_mode_launch',
            executable='remapping',
            name='remapping_mode',
            output='screen'
        )
    ])
