import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_multi_room = get_package_share_directory('multi_room_turtlebots')

    map_path = os.path.join(pkg_multi_room, 'maps', 'multi_room_map.yaml')
    params_path = os.path.join(pkg_multi_room, 'params', 'nav2_params.yaml')

    robots = ['robot1', 'robot2', 'robot3', 'robot4']
    nav2_launches = []

    for robot in robots:
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': robot,
                'use_namespace': 'True',
                'map': map_path,
                'params_file': params_path,
                'use_sim_time': 'True'
            }.items()
        )
        nav2_launches.append(nav2)

    ld = LaunchDescription()
    for nav2 in nav2_launches:
        ld.add_action(nav2)

    return ld

