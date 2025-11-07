#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_all_robots(context, *args, **kwargs):
    """Spawn all 4 robots"""
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    spawn_positions = [
        ('robot1', '0.35', '-0.3', '0.01'),
        ('robot2', '-0.7', '-0.5', '0.01'),
        ('robot3', '-0.4', '0.7', '0.01'),
        ('robot4', '0.75', '0.7', '0.01'),
    ]
    
    spawn_nodes = []
    for robot_name, x, y, z in spawn_positions:
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf'),
                '-x', x,
                '-y', y,
                '-z', z,
                '-robot_namespace', robot_name,
            ],
            output='screen'
        )
        spawn_nodes.append(spawn_node)
    
    return spawn_nodes


def generate_launch_description():
    multi_room_dir = get_package_share_directory('multi_room')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    world_file = os.path.join(multi_room_dir, 'worlds', 'room_world.sdf')
    
    gazebo_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(turtlebot3_gazebo_dir, 'models') + ':$GAZEBO_MODEL_PATH'
    )
    
    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(turtlebot3_gazebo_dir, 'lib') + ':$GAZEBO_PLUGIN_PATH'
    )
    
    turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    print(f"\n[multi_room] Launching all 4 robots in your world\n")
    
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )
    
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    spawn_robots = OpaqueFunction(function=spawn_all_robots)
    
    ld = LaunchDescription([
        gazebo_model_db,
        gazebo_model_path,
        gazebo_plugin_path,
        turtlebot3_model,
        gzserver,
        gzclient,
        spawn_robots,
    ])
    
    return ld

