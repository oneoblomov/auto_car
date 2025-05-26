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
        description='Width of the map in meters'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height', 
        default_value='20.0',
        description='Height of the map in meters'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Map resolution in meters per pixel'
    )

    # Map server node
    map_server_node = Node(
        package='detect',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'map_width': LaunchConfiguration('map_width'),
            'map_height': LaunchConfiguration('map_height'),
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': 'map'
        }],
        remappings=[
            ('map', '/map'),
        ]
    )

    return LaunchDescription([
        map_width_arg,
        map_height_arg,
        resolution_arg,
        map_server_node,
    ])
