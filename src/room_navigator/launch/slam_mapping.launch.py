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

    ld = LaunchDescription()
    
    # Launch Gazebo with custom world
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_room_nav)))
    
    # Spawn only ONE robot for mapping
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_tb3),
            launch_arguments={
                'robot_prefix': 'robot1',
                'x_pose': '0.35',
                'y_pose': '-0.3',
            }.items()
        )
    )

    return ld

