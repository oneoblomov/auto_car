from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaplan',
    maintainer_email='222802078@ogr.cbu.edu.tr',
    description='Tank robot autonomous driving system with obstacle detection and path planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_node = detect.obstacle_node:main',
            'path_planner = detect.path_planner_node:main',
            'autonomous_driver = detect.autonomous_driver_node:main',
            'behavior_manager = detect.behavior_manager_node:main',
            'control_interface = detect.control_interface_node:main',
            'map_server = detect.map_server_node:main',
            'slam_node = detect.slam_node:main',
            'tf_publisher = detect.tf_publisher_node:main',
        ],
    },
)
