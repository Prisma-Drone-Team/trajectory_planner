# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (make sure you have this patch: https://github.com/IntelRealSense/realsense-ros/issues/2564#issuecomment-1336288238)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d400.launch.py
#          'qos_image': 2,
#          'qos': 2,
#

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'map_frame_id':'map',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'visual_odometry': False,
          'approx_sync':True}]

    remappings=[
          ('rgb/image', '/camera'),
          ('/odom', '/odom'),
          ('rgb/camera_info', '/camera_info'),
          ('depth/image','/depth_camera')]

    return LaunchDescription([

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('realsense2_camera'), 'launch'),
        #         '/rs_launch.py']),
        #         launch_arguments={'align_depth.enable': 'true',
        #                           'rgb_camera.profile': '640x360x30'}.items(),
        # ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),


        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),
    ])
