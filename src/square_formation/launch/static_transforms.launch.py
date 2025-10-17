from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static TFs: map -> robotN/odom (translation only, identity rotation)
    static_tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x','0.0','--y','0.0','--z','0.0',
                '--qx','0','--qy','0','--qz','0','--qw','1',
                '--frame-id','map','--child-frame-id','robot1/odom'
            ],
            name='map_to_robot1_odom'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x','1.0','--y','0.0','--z','0.0',
                '--qx','0','--qy','0','--qz','0','--qw','1',
                '--frame-id','map','--child-frame-id','robot2/odom'
            ],
            name='map_to_robot2_odom'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x','1.0','--y','1.0','--z','0.0',
                '--qx','0','--qy','0','--qz','0','--qw','1',
                '--frame-id','map','--child-frame-id','robot3/odom'
            ],
            name='map_to_robot3_odom'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x','0.0','--y','1.0','--z','0.0',
                '--qx','0','--qy','0','--qz','0','--qw','1',
                '--frame-id','map','--child-frame-id','robot4/odom'
            ],
            name='map_to_robot4_odom'
        ),
    ]

    # Formation controllers (one per robot)
    formation_nodes = [
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot1',
            parameters=[{'robot_name': 'robot1'}],
        ),
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot2',
            parameters=[{'robot_name': 'robot2'}],
        ),
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot3',
            parameters=[{'robot_name': 'robot3'}],
        ),
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot4',
            parameters=[{'robot_name': 'robot4'}],
        ),
    ]

    return LaunchDescription(static_tfs + formation_nodes)

