from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Start Gazebo (Classic) with server+client
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Path to TurtleBot3 Burger SDF
    tb3_gz_share = get_package_share_directory('turtlebot3_gazebo')
    model_path = os.path.join(tb3_gz_share, 'models', 'turtlebot3_burger', 'model.sdf')

    # Define robot namespaces and spawn poses
    robots = [
        ('robot1', 0.0, 0.0, 0.0),
        ('robot2', 2.0, 0.0, 0.0),
        ('robot3', 2.0, 2.0, 0.0),
        ('robot4', 0.0, 2.0, 0.0),
    ]

    spawners = []
    controllers = []

    for ns, x, y, yaw in robots:
        spawners.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{ns}',
                arguments=[
                    '-entity', ns,
                    '-file', model_path,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.01',
                    '-Y', str(yaw),
                    '-robot_namespace', f'/{ns}',
                ],
                output='screen'
            )
        )

        controllers.append(
            Node(
                package='square_formation',
                executable='square_controller.py',
                namespace=ns,
                name='square_controller',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        )

    return LaunchDescription([gazebo_launch] + spawners + controllers)

