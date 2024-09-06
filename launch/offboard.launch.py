from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    offboard_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('trajectory_planner'), '/launch/offboard.launch.py'])
    )
    # Get the config path
    config = os.path.join(
        get_package_share_directory('trajectory_planner'),
        'config',
        'params.yaml'
        )

    # Define the mocap_px4_bridge node
    offboard_node = Node(
        package='trajectory_planner',
        executable='offboard_node',
        name='offboard_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        offboard_launch_file,
        offboard_node,
    ])
