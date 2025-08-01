import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    return LaunchDescription([
        # OAk-D Lite
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['2', '5', '2', '0','0', '0', 'map', 'goal1']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['1', '4', ' 2',  '0', '0', '0', 'map', 'goal2']),
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['8', '4', ' 2',  '0', '0', '0', 'map', 'goal3']), 
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['7', '8', ' 2',  '0', '0', '0', 'map', 'goal4']), 
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['12', '7', ' 2',  '0', '0', '0', 'map', 'goal5']), 
        # Realsense camera
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0.15', '0.03', ' 0.202', '0', '0', '0', 'base_link', '/x500_depth_0/camera_link/depth']),
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0.15', '0.03', ' 0.202', '0', '0', '0', 'base_link', '/x500_depth_0/camera_link/color']),
    ])

