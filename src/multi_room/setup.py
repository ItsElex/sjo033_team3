import os
from glob import glob
from setuptools import setup

package_name = 'multi_room'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elex',
    maintainer_email='elex@example.com',
    description='Multi-room robot navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)

