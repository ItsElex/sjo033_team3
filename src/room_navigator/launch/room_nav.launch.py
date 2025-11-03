import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_room_nav = PathJoinSubstitution([FindPackageShare('room_navigator'), 'launch', 'sim_room_nav.launch.py'])
    spawn_tb3 = PathJoinSubstitution([FindPackageShare('multi_turtlebot_sim'), 'launch', 'spawn_turtlebot3.launch.py'])

    spawn_positions = {
        'robot1': {'x_pose': '0.35', 'y_pose': '-0.3'},
        'robot2': {'x_pose': '-0.7',  'y_pose': '-0.5'},
        'robot3': {'x_pose': '-0.4',  'y_pose': '0.7'},
        'robot4': {'x_pose': '0.75', 'y_pose': '0.7'},
    }

    ld = LaunchDescription()
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_room_nav)))

    for robot_prefix, pos in spawn_positions.items():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_tb3),
                launch_arguments={
                    'robot_prefix': robot_prefix,
                    'x_pose': pos['x_pose'],
                    'y_pose': pos['y_pose'],
                }.items()
            )
        )

    return ld

