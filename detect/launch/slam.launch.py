#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='20.0',
        description='Width of the SLAM map in meters'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height', 
        default_value='20.0',
        description='Height of the SLAM map in meters'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Map resolution in meters per pixel'
    )
    
    num_particles_arg = DeclareLaunchArgument(
        'num_particles',
        default_value='100',
        description='Number of particles for localization'
    )

    # SLAM node
    slam_node = Node(
        package='detect',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'map_width': LaunchConfiguration('map_width'),
            'map_height': LaunchConfiguration('map_height'),
            'resolution': LaunchConfiguration('resolution'),
            'num_particles': LaunchConfiguration('num_particles'),
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom'
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('slam_map', '/slam_map'),
            ('slam_pose', '/slam_pose'),
        ]
    )

    return LaunchDescription([
        map_width_arg,
        map_height_arg,
        resolution_arg,
        num_particles_arg,
        slam_node,
    ])
