#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def spawn_robot(context, *args, **kwargs):

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_description = get_package_share_directory('turtlebot3_description')

    xacro_file = os.path.join(tb3_description, "urdf", "turtlebot3_burger.urdf")
    robot_description = xacro.process_file(xacro_file).toxml()

    nodes = []

    nodes.append(
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "turtlebot3",
                "-file", os.path.join(tb3_gazebo, "models", "turtlebot3_burger", "model.sdf"),
                "-x", "-0.8", "-y", "0.7", "-z", "0.01"

                # по высоте от меня и ширина это у
            ],
            output="screen",
        )
    )

    # qustion mark here can be different
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True
            }]
        )
    )

    return nodes


def generate_launch_description():

    multi_room_dir = get_package_share_directory("multi_room")
    world_file = os.path.join(multi_room_dir, "worlds", "room_world.sdf")

    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")

    return LaunchDescription([

        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.path.join(tb3_gazebo, "models")
        ),

        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),

        # supposed to work like this 
        ExecuteProcess(
            cmd=[
                "gzserver",
                world_file,
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so"
            ],
            output="screen"
        ),

        ExecuteProcess(
            cmd=["gzclient"],
            output="screen"
        ),

        # call service basically
        OpaqueFunction(function=spawn_robot),
    ])
