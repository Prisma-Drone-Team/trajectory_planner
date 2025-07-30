#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    # Trova il package
    pkg_share = FindPackageShare('trajectory_planner')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'controller_type',
            default_value='xbox',
            choices=['xbox', 'ps4', 'custom'],
            description='Tipo di controller (xbox, ps4, custom)'
        ),
        
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Device del joystick'
        ),
        
        DeclareLaunchArgument(
            'max_velocity',
            default_value='2.0',
            description='Velocità massima in m/s'
        ),
        
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Abilita log di debug'
        ),
        
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),
        
        # Teleop con configurazione Xbox
        Node(
            package='trajectory_planner',
            executable='teleop_node',
            name='teleop_node',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'teleop_params.yaml']),
                {
                    # Override per Xbox controller
                    'axis_x': 0,      # Left stick X
                    'axis_y': 1,      # Left stick Y  
                    'axis_z': 4,      # Right trigger/left trigger
                    'axis_yaw': 3,    # Right stick X
                    'button_arm': 0,      # A
                    'button_takeoff': 1,  # B
                    'button_land': 2,     # X
                    'button_emergency': 3, # Y
                    'max_velocity': LaunchConfiguration('max_velocity')
                }
            ],
            output='screen',
            condition=LaunchConfigurationEquals('controller_type', 'xbox')
        ),
        
        # Teleop con configurazione PS4
        Node(
            package='trajectory_planner',
            executable='teleop_node',
            name='teleop_node',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'teleop_params.yaml']),
                {
                    # Override per PS4 controller  
                    'axis_x': 0,      # Left stick X
                    'axis_y': 1,      # Left stick Y
                    'axis_z': 5,      # Right stick Y 
                    'axis_yaw': 2,    # Right stick X
                    'button_arm': 1,      # Circle
                    'button_takeoff': 0,  # X
                    'button_land': 3,     # Square
                    'button_emergency': 2, # Triangle
                    'max_velocity': LaunchConfiguration('max_velocity')
                }
            ],
            output='screen',
            condition=LaunchConfigurationEquals('controller_type', 'ps4')
        ),
        
        # Teleop con file di configurazione custom
        Node(
            package='trajectory_planner', 
            executable='teleop_node',
            name='teleop_node',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'teleop_params.yaml']),
                {
                    'max_velocity': LaunchConfiguration('max_velocity')
                }
            ],
            output='screen',
            condition=LaunchConfigurationEquals('controller_type', 'custom')
        )
    ])
