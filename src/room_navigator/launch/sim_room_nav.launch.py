import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_path = PathJoinSubstitution([FindPackageShare('room_navigator'), 'config', 'room_world.sdf'])

    gzserver_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
    gzclient_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': LaunchConfiguration('verbose'),
                'pause': LaunchConfiguration('paused'),
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gzclient_launch)),
    ])

