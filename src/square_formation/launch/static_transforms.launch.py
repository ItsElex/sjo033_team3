from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '1.0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot2/odom']
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '1.0', '1.0', '0', '0', '0', '0', 'robot1/odom', 'robot3/odom']
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '1.0', '0', '0', '0', '0', 'robot1/odom', 'robot4/odom']
        ),
    ])

