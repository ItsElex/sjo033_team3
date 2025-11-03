from setuptools import setup

package_name = 'room_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/sim_room_nav.launch.py',
             'launch/room_nav.launch.py',
             'launch/slam_mapping.launch.py',
             'launch/slam_toolbox.launch.py',
             'launch/teleop.launch.py',
             'launch/multi_robot_nav2.launch.py']),
        ('share/' + package_name + '/config',
            ['config/room_world.sdf',
             'config/waypoints.yaml',
             'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    entry_points={
        'console_scripts': [
            'coordinator_node = room_navigator.coordinator_node:main',
        ],
    },
)

