import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        #'database_path': '/root/ros2_ws/src/pkg/trajectory_planner/config/rtabmap.db',  # rtabmap database
        'subscribe_depth': True,
        'subscribe_odom_info': False,  
        'approx_sync': True,
        'qos_image': 2,
        'qos': 2,
        'odom_topic':'/model/baby_k_0/odometry', 
        'rgbd_cameras': 1,
        'rgbd_camera_frame_id': 'baby_k_0/OakD-Lite/base_link/IMX214', 
        'wait_for_transform': 10.5,
        'Mem/IncrementalMemory': 'true',  # Set to false for localization mode
        'Mem/InitWMWithAllNodes': 'true',  
        #'sync_queue_size': 10,  # Adjusted for simulation
        #'topic_queue_size': 10,  # Adjusted for simulation
        #'publish_tf': True,  # CRITICAL: Force rtabmap to publish tf
        'publish_tf_odom': False,  # Don't conflict with move_manager odometry
        #'publish_null_when_lost': True,  # Keep publishing tf even when lost
        # 'Reg/Force3DoF': 'true',  # Enable 2D SLAM for better initial mapping
        # 'Grid/FromDepth': 'false',  # Use 2D occupancy grid
        #'Mem/NotLinkedNodesKept': 'false',  # Don't keep unlinked nodes
        'use_sim_time': True,
    }]

    remappings=[
          ('rgb/image', '/camera'),
          ('rgb/camera_info', '/camera_info'),
          ('depth/image', '/depth_camera')]

    return LaunchDescription([


        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings),

    ])
