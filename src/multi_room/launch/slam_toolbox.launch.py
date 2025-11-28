#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction


def generate_launch_description():
    # Publish static transform: base_footprint -> base_scan
    # Arguments: x y z roll pitch yaw parent_frame child_frame
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_scan_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'base_scan'],
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'solver_plugin': 'solver_plugins::CeresSolver',
                'ceres_linear_solver_type': 'SPARSE_NORMAL_CHOLESKY',
                'ceres_linear_solver_threads': 1,
                'ceres_threads': 4,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': 'scan',
                'mode': 'mapping',
                'throttle_scans': 5,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_travel_distance': 0.5,
                'minimum_rotation_radians': 0.5,
                'use_map_saver': True,
                'map_file_name': 'multi_room_map',
                'load_state_filename': '',
                'tf_buffer_duration': 30.0,
            }
        ],
        remappings=[
            ('scan', '/robot1/scan'),
            ('odom', '/robot1/odom'),
        ]
    )

    # Delay SLAM by 2 seconds to let static TF publish first
    delayed_slam = TimerAction(
        period=2.0,
        actions=[slam_toolbox_node]
    )

    return LaunchDescription([
        static_tf_pub,
        delayed_slam,
    ])

