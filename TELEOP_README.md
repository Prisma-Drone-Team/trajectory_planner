# Teleop Control System

Integrated joystick teleoperation system for the Trajectory Planner, providing real-time manual control with safety mechanisms and seamless mode switching.

## Overview

The teleop system is fully integrated into the main `offboard_control` node, running as a separate thread to provide:
- **Real-time Control**: Direct joystick input processing without blocking trajectory execution
- **Safety Mechanisms**: Automatic joystick detection and availability checking
- **Seamless Transitions**: Smooth switching between autonomous and manual modes
- **Continuous Tracking**: Position and orientation continuity across mode changes

## Features

### 🎮 Control Capabilities
- **4-Axis Control**: Full position (X, Y, Z) and yaw control
- **Velocity Limiting**: Automatic 50% velocity reduction for safety
- **Dead Zone Handling**: Configurable stick dead zones
- **Smooth Transitions**: Continuous setpoint tracking

### 🛡️ Safety Features
- **Joy Node Detection**: Automatic joystick availability via `_joy_available` flag
- **Connection Monitoring**: Real-time joystick connection status
- **Safe Deactivation**: Smooth position handoff when exiting teleop
- **Emergency Stop**: Immediate trajectory termination capability

### 🔄 Integration Features
- **Threaded Operation**: Non-blocking parallel execution with main control loop
- **Unified Publishing**: Single trajectory setpoint publisher for consistency
- **Mode Awareness**: Intelligent switching between autonomous and manual control
- **State Preservation**: Position tracking for seamless mode transitions

## Joystick Mapping

### Control Axes
- **Left Stick X** (axis 0) → **Yaw Rotation**
- **Left Stick Y** (axis 1) → **Vertical Movement (Z)** *(inverted)*
- **Right Stick X** (axis 2) → **Lateral Movement (Y)**
- **Right Stick Y** (axis 3) → **Forward/Backward (X)**

### Control Characteristics
- **Velocity Scale**: 50% of maximum configured velocity
- **Update Rate**: Real-time joystick input processing
- **Dead Zone**: Configurable stick dead zone (default: 0.1)
- **Smooth Control**: Continuous velocity integration

## Usage

### 1. System Requirements
- Xbox/PlayStation compatible controller
- `joy_node` running and publishing to `/joy`
- Main trajectory planner (`offboard_control`) active

### 2. Activation Methods

#### Via Keyboard Interface
```bash
# In the offboard_control terminal
Enter command: teleop
```

#### Via Topic Command
```bash
ros2 topic pub /seed_pdt_drone/command std_msgs/msg/String "{data: 'teleop'}"
```

#### Via Move Manager
```bash
# If using move_manager_node
ros2 topic pub /move_cmd trajectory_planner/msg/MoveCmd "{command: 'teleop'}"
```

### 3. Operation Sequence
1. **Ensure Joystick**: Connect controller and verify `joy_node` is running
2. **Activate Teleop**: Use any activation method above
3. **Manual Control**: Use joystick sticks for 4-axis control
4. **Exit Teleop**: Send any other command or use emergency stop

### 4. Safety Checks
The system automatically verifies:
- Joy node is active and publishing
- Joystick data is recent and valid
- Controller connection is stable

## Integration Architecture

### Threading Model
```
Main Thread (offboard_control)
├── ROS2 Spinning & Trajectory Execution
├── Keyboard Input Thread
└── Teleop Thread (when active)
    ├── Joy Input Processing
    ├── Velocity Calculation
    └── Position Integration
```

### Data Flow
```
Joystick → joy_node → /joy topic → teleop_thread → unified_callback → PX4
                                        ↓
                               Position Tracking → _prev_sp → Smooth Transitions
```

### Safety Integration
- **_joy_available Flag**: Set only when joy_node is detected
- **Connection Monitoring**: Continuous joystick data validation
- **Smooth Handoff**: Position synchronization on mode exit
- **Emergency Systems**: Immediate stop capability

## Configuration

### System Parameters
The teleop system uses the main trajectory planner parameters:
```yaml
# In sim_params.yaml or params.yaml
max_vel: 2.0                # Maximum velocity (teleop uses 50%)
max_acc: 1.0                # Maximum acceleration
use_key_input: 1.0          # Enable keyboard interface
```

### Joystick Configuration
```bash
# Verify joystick device
ls /dev/input/js*

# Test joystick functionality
jstest /dev/input/js0

# Check joy_node output
ros2 topic echo /joy
```

## Troubleshooting

### Common Issues

#### 1. Teleop Won't Activate
```bash
# Check joy_node status
ros2 topic list | grep joy
ros2 topic hz /joy

# Verify _joy_available flag is set
# Look for "Joy node detected" in offboard_control logs
```

#### 2. Erratic Movement
- Check joystick calibration with `jstest`
- Verify stick dead zones are appropriate
- Ensure stable joystick connection

#### 3. Mode Switching Issues
```bash
# Check current mode status
ros2 topic echo /leo/drone/plan_status

# Verify position continuity
ros2 topic echo /fmu/in/trajectory_setpoint
```

### Debug Information
```bash
# Monitor teleop activation
# Look for these log messages in offboard_control:
# - "Joy node detected, enabling teleop capability"
# - "Entering teleop mode"
# - "Exiting teleop mode due to new command"

# Check joystick data
ros2 topic echo /joy --no-arr

# Monitor trajectory setpoints
ros2 topic echo /fmu/in/trajectory_setpoint
```

## Advanced Features

### Smooth Transitions
The system ensures continuity when switching modes:
```cpp
// On teleop exit, position is preserved
_prev_sp = _teleop_position;
_prev_yaw_sp = _teleop_yaw;
_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, _teleop_yaw));
```

### Thread Safety
- Atomic flag operations for mode switching
- Thread-safe position updates
- Synchronized trajectory publishing

### Integration with Autonomous Modes
- Seamless entry from any autonomous mode
- Preserved trajectory state on exit
- Intelligent fallback mechanisms

## Development Notes

### Code Location
- **Main Integration**: `src/offboard/offboard_control.cpp`
- **Joy Callback**: `joy_callback()` function
- **Teleop Thread**: `teleop_mode()` function
- **Safety Checks**: `_joy_available` flag management

### Key Functions
```cpp
void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);    // Joy input processing
void teleop_mode();                                               // Main teleop thread
void offboard_callback();                                         // Unified trajectory publishing
```

### Extension Points
- Modify joystick mapping in `joy_callback()`
- Adjust velocity scaling in `teleop_mode()`
- Add new safety checks in teleop activation logic

## Safety Guidelines

⚠️ **Important Safety Notes**:
- Always test in simulation first
- Maintain visual contact with drone during teleop
- Keep emergency stop readily available
- Verify joystick connection stability before flight
- Ensure adequate flight space for manual control

## License

This teleop control system is part of the Trajectory Planner package.

### Authors
- **Simone D'Angelo** - simone.dangelo@unina.it
- **Francesca Pagano** - francesca.pagano@unina.it  
- **Vincenzo Scognamiglio** - vincenzo.scognamiglio2@unina.it

**PRISMA LAB** - University of Naples Federico II

---

**Note**: This system is designed for research and development purposes. Always follow local aviation regulations and safety guidelines when operating autonomous vehicles.
