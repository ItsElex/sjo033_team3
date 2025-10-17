from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # robot1 at (0, 0, yaw 0): q = (0,0,0,1)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'map', 'robot1/odom']),
        # robot2 at (1, 0, yaw 0): q = (0,0,0,1)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['1.0', '0.0', '0.0', '0', '0', '0', '1', 'map', 'robot2/odom']),
        # robot3 at (1, 1, yaw +pi/2): qz≈0.7071, qw≈0.7071
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['1.0', '1.0', '0.0', '0', '0', '0.7071068', '0.7071068', 'map', 'robot3/odom']),
        # robot4 at (0, 1, yaw +pi): qz≈1.0, qw≈0.0
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.0', '1.0', '0.0', '0', '0', '1.0', '0.0', 'map', 'robot4/odom']),
    ])

