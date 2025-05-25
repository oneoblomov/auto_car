#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
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
