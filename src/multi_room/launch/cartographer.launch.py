#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    multi_room_dir = get_package_share_directory('multi_room')
    config_dir = os.path.join(multi_room_dir, 'config')
    
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_scan_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'base_scan'],
        output='screen'
    )
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ('/scan', '/robot1/scan'),
            ('/odom', '/robot1/odom'),
        ],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_robot1.lua',
        ]
    )
    
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
    )
    
    delayed_cartographer = TimerAction(
        period=2.0,
        actions=[cartographer_node]
    )
    
    delayed_occupancy = TimerAction(
        period=5.0,
        actions=[occupancy_grid]
    )

    return LaunchDescription([
        static_tf_pub,
        delayed_cartographer,
        delayed_occupancy,
    ])

