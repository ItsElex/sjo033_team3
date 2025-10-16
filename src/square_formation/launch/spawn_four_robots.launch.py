import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_multi_turtlebot_sim = get_package_share_directory('multi_turtlebot_sim')

    # Include Gazebo launch (empty world by default)
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
    )

    spawn_positions = {
        'robot1': {'x_pose': 0.0, 'y_pose': 0.0},
        'robot2': {'x_pose': 1.0, 'y_pose': 0.0},
        'robot3': {'x_pose': 1.0, 'y_pose': 1.0},
        'robot4': {'x_pose': 0.0, 'y_pose': 1.0},
    }

    spawn_robots = []
    for robot_prefix, pos in spawn_positions.items():
        spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_multi_turtlebot_sim, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'robot_prefix': robot_prefix,
                'x_pose': str(pos['x_pose']),
                'y_pose': str(pos['y_pose']),
            }.items()
        )
        spawn_robots.append(spawn)

    ld = LaunchDescription()
    ld.add_action(include_gazebo)
    for spawn in spawn_robots:
        ld.add_action(spawn)

    return ld

