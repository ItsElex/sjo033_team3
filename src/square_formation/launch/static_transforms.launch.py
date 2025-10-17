from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tfs = [
        # robot1 at (0, 0), yaw 0  -> q = (0,0,0,1)
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
        # robot2 at (1, 0), yaw 0 -> q = (0,0,0,1)
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
        # robot3 at (1, 1), yaw +pi/2 -> qz=sin(pi/4)=0.7071068, qw=cos(pi/4)=0.7071068
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x','1.0','--y','1.0','--z','0.0',
                '--qx','0','--qy','0','--qz','0.7071068','--qw','0.7071068',
                '--frame-id','map','--child-frame-id','robot3/odom'
            ],
            name='map_to_robot3_odom'
        ),
        # robot4 at (0, 1), yaw +pi -> qz=sin(pi/2)=1.0, qw=cos(pi/2)=0.0
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x','0.0','--y','1.0','--z','0.0',
                '--qx','0','--qy','0','--qz','1.0','--qw','0.0',
                '--frame-id','map','--child-frame-id','robot4/odom'
            ],
            name='map_to_robot4_odom'
        ),
    ]

    # One formation node per robot; parameter robot_name selects the topics
    formation_nodes = [
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot1',
            parameters=[{'robot_name':'robot1'}],
            # If your node uses relative topic names, you can also namespace it:
            # namespace='robot1',
        ),
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot2',
            parameters=[{'robot_name':'robot2'}],
            # namespace='robot2',
        ),
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot3',
            parameters=[{'robot_name':'robot3'}],
            # namespace='robot3',
        ),
        Node(
            package='square_formation',
            executable='square_formation_node',
            name='square_formation_robot4',
            parameters=[{'robot_name':'robot4'}],
            # namespace='robot4',
        ),
    ]

    return LaunchDescription(static_tfs + formation_nodes)

