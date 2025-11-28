#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_robots_with_state_pub(context, *args, **kwargs):
    """Spawn all 4 robots and start robot state publishers"""
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Robot spawn configurations
    robots_config = [
        {
            'name': 'robot1',
            'x': '0.35',
            'y': '-0.3',
        },
        {
            'name': 'robot2',
            'x': '-0.7',
            'y': '-0.5',
        },
        {
            'name': 'robot3',
            'x': '-0.4',
            'y': '0.7',
        },
        {
            'name': 'robot4',
            'x': '0.75',
            'y': '0.7',
        },
    ]
    
    nodes = []
    
    # Spawn each robot and create state publisher
    for robot_config in robots_config:
        robot_name = robot_config['name']
        
        # Get URDF file from turtlebot3_gazebo
        urdf_file = os.path.join(
            turtlebot3_gazebo_dir, 
            'urdf', 
            'turtlebot3_burger.urdf'
        )
        
        # Read URDF file and publish as robot_description
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
        
        # Spawn robot
        nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf'),
                '-x', robot_config['x'],
                '-y', robot_config['y'],
                '-z', '0.01',
                '-robot_namespace', robot_name,
            ],
            output='screen'
        ))
        
        # Robot state publisher (publishes URDF to robot_description topic)
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{robot_name}',
            namespace=robot_name,
            output='screen',
            parameters=[
                {'robot_description': urdf_content},
                {'use_sim_time': True}
            ],
            remappings=[
                ('joint_states', f'/{robot_name}/joint_states'),
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
    
    use_sim_time = SetEnvironmentVariable(
        name='USE_SIM_TIME',
        value='True'
    )
    
    print("\n[multi_room] Launching 4 robots with state publishers\n")
    
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
    
    spawn_robots = OpaqueFunction(function=spawn_robots_with_state_pub)
    
    ld = LaunchDescription([
        gazebo_model_db,
        gazebo_model_path,
        gazebo_plugin_path,
        turtlebot3_model,
        use_sim_time,
        gzserver,
        gzclient,
        spawn_robots,
    ])
    
    return ld

