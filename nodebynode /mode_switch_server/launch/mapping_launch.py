from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='slam_toolbox_node'
        )
    ])