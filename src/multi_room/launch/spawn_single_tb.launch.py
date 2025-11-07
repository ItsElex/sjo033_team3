#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_robot_with_state_pub(context, *args, **kwargs):
    """Spawn robot1 and start robot state publisher"""
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    nodes = []
    
    # Spawn robot1
    nodes.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf'),
            '-x', '0.35',
            '-y', '-0.3',
            '-z', '0.01',
            '-robot_namespace', 'robot1',
        ],
        output='screen'
    ))
    
    # Robot state publisher (publishes TF tree from URDF)
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        remappings=[
            ('robot_description', '/robot1/robot_description'),
            ('joint_states', '/robot1/joint_states'),
        ]
    ))
    
    return nodes


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
    
    print("\n[multi_room] Launching robot1 with state publisher\n")
    
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
    
    spawn_robot = OpaqueFunction(function=spawn_robot_with_state_pub)
    
    ld = LaunchDescription([
        gazebo_model_db,
        gazebo_model_path,
        gazebo_plugin_path,
        turtlebot3_model,
        gzserver,
        gzclient,
        spawn_robot,
    ])
    
    return ld

