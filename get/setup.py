from setuptools import find_packages, setup

package_name = 'get'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaplan',
    maintainer_email='222802078@ogr.cbu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = get.imu_node:main',
            'lidar_node = get.lidar_node:main',
            'encoder_node = get.encoder_node:main',
            'rgbd_camera_node = get.rgbd_camera_node:main',
            'targeting_camera_node = get.targeting_camera_node:main',
            'back_camera_node = get.back_camera_node:main',
        ],
    },
)
