#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    multi_room_dir = get_package_share_directory('multi_room')
    
    # Path to your Cartographer map files
    map_yaml = os.path.join(multi_room_dir, 'maps', 'my_map.yaml')
    
    use_sim_time = SetEnvironmentVariable(
        name='USE_SIM_TIME',
        value='True'
    )
    
    # Map server (publishes map as /map topic) - without lifecycle manager
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    ld = LaunchDescription([
        use_sim_time,
        map_server,
    ])
    
    return ld

