# 4D_trajectory_planning
ROS2 node to plan cartesian trajectory for a standard px4-based UAV  

## How to run

Launch PX4 gazebo simulation and run DDS agent in two different terminal

       $ MicroXRCEAgent udp4 -p 8888

In a third terminal run the trajectory planner node

       $ ros2 run 4D_trajectory_planning offboard_control 

## Features

Takeoff routine 

The desired yaw is computed to have the drone navigating toward the approaching direction

