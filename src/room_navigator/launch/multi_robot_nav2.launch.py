import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup = FindPackageShare(package='nav2_bringup')
    room_navigator = FindPackageShare(package='room_navigator')
    
    map_yaml_file = os.path.join(room_navigator, 'config', 'map.yaml')
    nav2_params_file = os.path.join(room_navigator, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml_file),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Nav2 Bringup (we'll configure this properly after mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])

