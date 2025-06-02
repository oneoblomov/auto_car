#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Enable SLAM for real-time mapping'
        ),
        
        DeclareLaunchArgument(
            'enable_map_server',
            default_value='false',
            description='Enable static map server'
        ),
        
        LogInfo(msg="🚀 Tank Robot Otonom Sürüş Sistemi Başlatılıyor..."),
        
        # Motor control node
        Node(
            package='control',
            executable='motor_node',
            name='motor_control',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # TF Publisher node for coordinate transforms
        Node(
            package='detect',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Map server node (conditional)
        Node(
            package='detect',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_map_server')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_width': 20.0,
                'map_height': 20.0,
                'resolution': 0.1,
                'frame_id': 'map'
            }]
        ),
        
        # SLAM node (conditional)
        Node(
            package='detect',
            executable='slam_node',
            name='slam_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_slam')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_width': 20.0,
                'map_height': 20.0,
                'resolution': 0.1,
                'num_particles': 100,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom'
            }]
        ),
        
        # Obstacle detection node
        Node(
            package='detect',
            executable='obstacle_node',
            name='obstacle_detection',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Behavior manager node
        Node(
            package='detect',
            executable='behavior_manager',
            name='behavior_manager',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Path planner node
        Node(
            package='detect',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Autonomous driver node
        Node(
            package='detect',
            executable='autonomous_driver',
            name='autonomous_driver',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Control interface node
        Node(
            package='detect',
            executable='control_interface',
            name='control_interface',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        LogInfo(msg="✅ Tüm node'lar başlatıldı. Kontrol arayüzünü kullanabilirsiniz."),
    ])
