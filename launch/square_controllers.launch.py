from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = ['robot1','robot2','robot3','robot4']
    nodes = []
    for ns in robots:
        nodes.append(Node(
            package='square_formation',
            executable='square_controller.py',
            namespace=ns,
            name='square_controller',
            output='screen'
        ))
    return LaunchDescription(nodes)
