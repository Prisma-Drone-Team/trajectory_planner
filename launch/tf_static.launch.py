import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    return LaunchDescription([
        # ATTENZIONE grandissimo pezzotto! Odometria ruotato di 90 gradi
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            # arguments=['0', '0', '0', '1.57', '0', '0', 'base_footprint_enu', 'base_footprint']),
            arguments=['0', '0', '0', '3.14', '0', '0', 'base_footprint_enu', 'base_footprint']),
        
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.12', '0', '0.14', '0', '0', '0', 'base_footprint', 'camera_link']),

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0.28', '0', '0', '0', 'base_footprint', 'base_link']),

        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '-1.57', '0', '0', 'map', 'map_drone']),
    ])

