from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Assuming you physically place them in a square formation:
    # robot1 at origin, robot2 1m in +x, robot3 at (1,1), robot4 1m in +y
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'map', '--child-frame-id', 'robot1/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '1.0', '--y', '0.0', '--z', '0.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'map', '--child-frame-id', 'robot2/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '1.0', '--y', '1.0', '--z', '0.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'map', '--child-frame-id', 'robot3/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.0', '--y', '1.0', '--z', '0.0',
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'map', '--child-frame-id', 'robot4/odom']
        ),
    ])

