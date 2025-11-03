import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot3_gazebo = FindPackageShare(package='turtlebot3_gazebo')
    
    # Spawn 4 robots at different positions
    robots_config = [
        {'name': 'robot1', 'x_pos': '0.0', 'y_pos': '0.0', 'z_pos': '0.0'},
        {'name': 'robot2', 'x_pos': '0.5', 'y_pos': '0.0', 'z_pos': '0.0'},
        {'name': 'robot3', 'x_pos': '0.0', 'y_pos': '0.5', 'z_pos': '0.0'},
        {'name': 'robot4', 'x_pos': '0.5', 'y_pos': '0.5', 'z_pos': '0.0'},
    ]
    
    entities = []
    
    for robot in robots_config:
        entities.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot['name'],
                    '-topic', f'/{robot["name"]}/robot_description',
                    '-x', robot['x_pos'],
                    '-y', robot['y_pos'],
                    '-z', robot['z_pos'],
                ],
                output='screen'
            )
        )
    
    return LaunchDescription(entities)

