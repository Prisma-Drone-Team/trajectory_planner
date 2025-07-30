# Trajectory Planner

A comprehensive ROS2 trajectory planning and control system for autonomous drones with integrated teleop capabilities, SLAM integration, and multi-modal navigation.

## Overview

This package provides a complete autonomous flight control system featuring:
- **Trajectory Planning**: Spline-based trajectory generation with obstacle avoidance
- **Teleop Control**: Joystick-based manual control with safety mechanisms
- **SLAM Integration**: Real-time mapping and localization with RTABMap
- **Multi-Modal Navigation**: Seamless switching between autonomous and manual modes
- **Safety Systems**: Comprehensive safety checks and failsafes

## Features

### 🚁 Flight Modes
- **Autonomous Navigation**: Point-to-point and multi-waypoint navigation
- **Teleop Mode**: Real-time joystick control with safety mechanisms
- **Takeoff/Landing**: Automated takeoff and landing procedures
- **Emergency Stop**: Immediate trajectory termination

### 🗺️ Navigation Capabilities
- **SLAM Integration**: Real-time mapping with RTABMap
- **Obstacle Avoidance**: Dynamic path replanning around obstacles
- **Multi-Waypoint Navigation**: Sequential waypoint following with smooth transitions
- **Frame Transformations**: Automatic coordinate frame conversions (map ↔ odom)

### 🎮 Control Interfaces
- **Joystick Control**: Xbox/PS4 controller support
- **Keyboard Input**: Manual command interface
- **ROS2 Topics**: External command integration
- **Service Calls**: Programmatic control interface

### 🛡️ Safety Features
- **Joy Node Detection**: Automatic joystick availability checking
- **Smooth Transitions**: Continuous setpoint tracking between modes
- **Emergency Stop**: Immediate trajectory termination
- **Bounded Controls**: Velocity and acceleration limits

## Installation

### Prerequisites
- ROS2 Humble
- PX4 Autopilot
- Gazebo Garden
- RTABMap
- Boost Libraries

### Dependencies
```bash
sudo apt install ros-humble-joy ros-humble-tf2-ros ros-humble-geometry-msgs
sudo apt install ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-std-msgs
sudo apt install libboost-thread-dev libboost-system-dev
```

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_planner
source install/setup.bash
```

## Usage

### 1. Simulation Setup

#### Option A: Automated Setup with tmux (Recommended)
Use tmuxp to launch all nodes automatically:
```bash
# Install tmuxp if not already installed
pip install tmuxp

# Launch complete simulation environment
tmuxp load path/to/folder/simulation.yml
```

#### Option B: Manual Setup
Launch the complete simulation environment in separate terminals:
```bash
# Start simulation
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth

# Start MicroXRCE Agent
MicroXRCEAgent udp4 -p 8888

# Start ROS2 bridge
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo /camera@sensor_msgs/msg/Image@gz.msgs.Image /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image /model/x500_depth_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry

# Start RTABMap SLAM
ros2 launch trajectory_planner rtabmap_sim.launch.py

# Start joystick node
ros2 run joy joy_node

# Start visualization
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/pkg/trajectory_planner/rviz/leo.rviz

# Start TF publishers
ros2 launch trajectory_planner tf_static_sim.launch.py
```

### 2. Start the Trajectory Planner
```bash
# Main control node
ros2 run trajectory_planner offboard_control --ros-args --params-file /path/to/sim_params.yaml

# Command manager (optional)
ros2 run trajectory_planner move_manager_node --ros-args --params-file /path/to/sim_params.yaml
```

### 3. Basic Flight Operations

#### Keyboard Control
The system accepts these keyboard commands:
- `arm` - Arm the vehicle
- `takeoff` - Automated takeoff to specified altitude
- `go` - Fly to specific coordinates (X, Y, Z)
- `nav` - Multi-waypoint navigation with obstacle avoidance
- `teleop` - Enter joystick control mode
- `land` - Automated landing
- `stop` - Emergency stop
- `term` - Terminate system

#### Topic Commands
```bash
# Basic commands
ros2 topic pub /seed_pdt_drone/command std_msgs/msg/String "{data: 'takeoff'}"
ros2 topic pub /seed_pdt_drone/command std_msgs/msg/String "{data: 'flyto(goalX)'}" #X=1-7
ros2 topic pub /seed_pdt_drone/command std_msgs/msg/String "{data: 'teleop'}"
ros2 topic pub /seed_pdt_drone/command std_msgs/msg/String "{data: 'land'}"
```

### 4. Teleop Control

#### Joystick Mapping
- **Left Stick X** → Yaw rotation
- **Left Stick Y** → Vertical movement (Z)
- **Right Stick X** → Lateral movement (Y)
- **Right Stick Y** → Forward/backward (X)

#### Safety Features
- Teleop only activates if joystick node is running
- Automatic velocity limiting (50% of max velocity)
- Smooth transitions when exiting teleop mode
- Continuous position tracking for seamless mode switching

## Configuration

### Parameters (sim_params.yaml)
```yaml
# Basic parameters
use_key_input: 1.0          # Enable keyboard input
use_mocap: 0.0              # Use motion capture system
do_transform: 1.0           # Enable coordinate transformations
replan: 1.0                 # Enable dynamic replanning

# Velocity limits
max_vel: 2.0                # Maximum velocity (m/s)
max_acc: 1.0                # Maximum acceleration (m/s²)

# Navigation parameters
waypoint_tolerance: 0.2     # Waypoint arrival tolerance (m)
planning_timeout: 5.0       # Path planning timeout (s)

# Safety parameters
emergency_stop_decel: 3.0   # Emergency stop deceleration (m/s²)
min_flight_height: 0.5      # Minimum flight altitude (m)
```

## Architecture

### Core Components

#### OffboardControl
Main control node that handles:
- Trajectory execution and monitoring
- Mode switching (autonomous ↔ teleop)
- Safety checks and emergency stops
- Continuous setpoint publishing

#### MoveManagerNode
Command interface that:
- Processes external commands
- Validates command parameters
- Forwards commands to OffboardControl
- Maintains command history

### Control Flow
```
External Command → MoveManagerNode → OffboardControl → PX4
     ↑                                      ↓
Joystick Input ←→ Teleop Thread ←→ Unified Callback → Vehicle
```

### Threading Model
- **Main Thread**: ROS2 node spinning and trajectory execution
- **Teleop Thread**: Real-time joystick input processing
- **Key Input Thread**: Keyboard command interface
- **Path Checking Thread**: Dynamic obstacle detection

## Safety Mechanisms

### 1. Joystick Safety
- Joy node availability detection via `_joy_available` flag
- Automatic teleop deactivation if joystick disconnects
- Smooth position handoff when exiting teleop

### 2. Trajectory Continuity
- Continuous `_prev_sp` tracking across all modes
- Smooth transitions between waypoints in nav mode
- Position synchronization during mode switches

### 3. Emergency Systems
- Immediate trajectory termination on `stop` command
- Velocity bounds enforcement
- Altitude limit checking
- Communication timeout detection


## License

This project is licensed under the MIT License.

### Authors
- **Simone D'Angelo** - simone.dangelo@unina.it
- **Francesca Pagano** - francesca.pagano@unina.it  
- **Vincenzo Scognamiglio** - vincenzo.scognamiglio2@unina.it

**PRISMA LAB** - University of Naples Federico II

## Acknowledgments

- PX4 Development Team
- ROS2 Community
- RTABMap Developers
- PRISMA LAB - University of Naples Federico II

---

**Note**: This system is designed for research and development purposes. Always follow local aviation regulations and safety guidelines when operating autonomous vehicles.
