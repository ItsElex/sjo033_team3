#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('multi_turtlebot_sim')

    # world in your package (copy your multi_room.sdf into share/multi_turtlebot_sim/worlds/)
    world_file = os.path.join(pkg, 'worlds', 'multi_room.sdf')

    # turtlebot3 URDF (plain urdf shipped with package)
    tb3_urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )

    # spawn positions (user-provided)
    spawn_positions = [
        ('robot1', 0.35, -0.3, 0.01),
        ('robot2', -0.7, -0.5, 0.01),
        ('robot3', -0.4, 0.7, 0.01),
        ('robot4', 0.75, 0.7, 0.01),
    ]

    actions = []

    # 1) Start Ignition Gazebo (Fortress). If your system uses `ign gazebo` replace accordingly.
    ign_cmd = ['ign', 'gazebo', world_file, '-v', '4']
    actions.append(
        ExecuteProcess(cmd=ign_cmd, output='screen')
    )

    # 2) Spawn robots one-by-one with a small delay
    #    We call the ros_ign_gazebo 'create' helper via ros2 run to inject the model.
    #    If your installation exposes a different CLI, change the command accordingly.
    spawn_delay = 2.5  # seconds between spawns
    for idx, (name, x, y, z) in enumerate(spawn_positions):
        spawn_cmd = [
            'ros2', 'run', 'ros_ign_gazebo', 'create',
            '-name', name,
            '-file', tb3_urdf,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ]
        # delay each spawn so ign has time to bring world up and previous spawns complete
        actions.append(TimerAction(period=spawn_delay*(idx+1),
                                   actions=[ExecuteProcess(cmd=spawn_cmd, output='screen')]))

    return LaunchDescription(actions)

