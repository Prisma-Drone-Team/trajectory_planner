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
            arguments=['0.15', '0.03', ' 0.202', '-1.5707','0', '-1.5707', 'base_link', 'x500_depth_0/OakD-Lite/base_link/StereoOV7251']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.15', '0.03', ' 0.202',  '-1.5707', '0', '-1.5707', 'base_link', 'x500_depth_0/OakD-Lite/base_link/IMX214']),
        
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', ' 0', '0', '0', '3.14159265359', 'base_link_FRD', 'base_link']),  # ZYX

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['2', '5', ' 1.5', '0','0', '0', 'map', 'goal1']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['1', '4', ' 1.5',  '0', '0', '0', 'map', 'goal2']),
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['8', '4', ' 1.5',  '0', '0', '0', 'map', 'goal3']), 
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['7', '8', ' 1.5',  '0', '0', '0', 'map', 'goal4']), 
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['12', '7', ' 1.5',  '0', '0', '0', 'map', 'goal5']), 
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['15', '6', ' 1.5',  '0', '0', '0', 'map', 'goal6']), 
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['15', '1', ' 1.5',  '0', '0', '0', 'map', 'goal7']), 
    ])

