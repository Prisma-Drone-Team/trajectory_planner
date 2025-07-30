#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments per configurare il nodo
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Topic del joystick'
        ),
        
        DeclareLaunchArgument(
            'odom_topic', 
            default_value='/px4/odometry/out',
            description='Topic odometry del drone'
        ),
        
        DeclareLaunchArgument(
            'max_velocity',
            default_value='2.0',
            description='Velocità massima in m/s'
        ),
        
        DeclareLaunchArgument(
            'max_yaw_rate',
            default_value='1.0', 
            description='Velocità massima di yaw in rad/s'
        ),
        
        DeclareLaunchArgument(
            'velocity_scale',
            default_value='1.0',
            description='Scala per le velocità lineari'
        ),
        
        DeclareLaunchArgument(
            'yaw_scale',
            default_value='1.0',
            description='Scala per la velocità di yaw'
        ),
        
        # Joy node - pubblica i messaggi del joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # Modifica se necessario
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Teleop node
        Node(
            package='trajectory_planner',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'trajectory_setpoint_topic': '/px4/trajectory_setpoint_enu',
                'offboard_control_mode_topic': 'fmu/in/offboard_control_mode',
                'vehicle_command_topic': 'fmu/in/vehicle_command',
                'max_velocity': LaunchConfiguration('max_velocity'),
                'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                'velocity_scale': LaunchConfiguration('velocity_scale'),
                'yaw_scale': LaunchConfiguration('yaw_scale'),
                # Mapping assi per controller standard (Xbox/PS4)
                'axis_x': 0,      # Stick sinistro orizzontale
                'axis_y': 1,      # Stick sinistro verticale  
                'axis_z': 3,      # Stick destro verticale (throttle)
                'axis_yaw': 2,    # Stick destro orizzontale (yaw)
                # Mapping pulsanti
                'button_arm': 0,      # A/X
                'button_takeoff': 1,  # B/Circle
                'button_land': 2,     # X/Square
                'button_emergency': 3 # Y/Triangle
            }],
            output='screen'
        )
    ])
