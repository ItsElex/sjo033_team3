from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'multi_room'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package.xml
        ('share/ament_cmake_core/markers', ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        # Install map files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.pgm'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Multi-robot room navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'room_coordinate_node = multi_room.room_coordinate_node:main',
        ],
    },
)

