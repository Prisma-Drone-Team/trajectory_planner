# FEATURES to add

1. checker thread ( IDEA: add  _plan_is_valid flag to evluate)
2. add a way to control also yaw (no fixed heading)
3. octomap integration (subscription etc.)
4. state machine for current navigation state (publish it?)

#NB: 
ROS2 PX4 only works with NED frames. It is necessary to properly think the map bounds and frame transformation to avoid problem with ompl. (e.g. x-y are swapped between enu-ned and the arena is rectangular -> the planner gets stuck thinking the drone is outside the map)

- Drone Manager node:
  1. Implement keyboard command input and msg publish

# 4D_trajectory_planning
ROS2 node to plan cartesian trajectory for a standard px4-based UAV  

## How to run

Launch PX4 gazebo simulation and run DDS agent in two different terminal

       $ MicroXRCEAgent udp4 -p 8888

In a third terminal run the trajectory planner node

       $ ros2 run trajectory_planner offboard_control 

To include params

       $ ros2 run trajectory_planner offboard_control --ros-args --params-file <path_to_param_file> 
       $ ros2 run trajectory_planner offboard_control --ros-args --params-file /root/ros2_ws/src/pkg/trajectory_planner/config/params.yaml


## Features

Arming routine

Takeoff routine 

Point-to-point trajectory planning

The desired yaw is computed to have the drone navigating toward the approaching direction

Landing routine
