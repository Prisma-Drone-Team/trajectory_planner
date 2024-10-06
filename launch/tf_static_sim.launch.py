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
            arguments=['0', '0', ' 0', '-1.5707963', '-3.1415927',  '0', 'fake_odom', 'odomNED']),  # ZYX
        # Realsense camera
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0.15', '0.03', ' 0.202', '0', '0', '0', 'base_link', '/x500_depth_0/camera_link/depth']),
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0.15', '0.03', ' 0.202', '0', '0', '0', 'base_link', '/x500_depth_0/camera_link/color']),
    ])
