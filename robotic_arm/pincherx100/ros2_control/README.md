# PincherX100 ROS2 Control

ROS2 Humble control system for the PincherX100 robotic arm using Dynamixel XL430-W250 servos. This package provides a complete solution for controlling the arm via ROS2 topics, services, and actions, all running in a Docker container on Raspberry Pi 5.

## Features

-  **Topic-based control**: Position and velocity commands
-  **Joint state publishing**: Real-time feedback at 50Hz
-  **Trajectory execution**: Follow joint trajectories via action server
-  **Preset poses**: Home, Sleep, Ready positions via services
-  **Gripper control**: Open/close services
-  **Safety features**: Joint limit enforcement
-  **Docker containerized**: Isolated ROS2 environment
-  **Calibration tools**: Interactive calibration script
-  **Test utilities**: Joint testing and verification

## System Architecture

```
┌─────────────────────────────────────────┐
│         ROS2 Topics/Services            │
│  /joint_states (sensor_msgs/JointState) │
│  /joint_position_commands               │
│  /joint_velocity_commands               │
│  /joint_trajectory                      │
│  /follow_joint_trajectory (action)      │
│  /go_home, /go_sleep, /go_ready (svc)  │
│  /gripper_open, /gripper_close (svc)   │
└─────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────┐
│      ArmControllerNode (Python)         │
│  - Multi-threaded executor              │
│  - Joint limit enforcement              │
│  - Trajectory interpolation             │
└─────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────┐
│     DynamixelInterface (Python)         │
│  - Dynamixel SDK wrapper                │
│  - Sync read/write operations           │
└─────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────┐
│    Hardware: XL430-W250 Servos          │
│  /dev/ttyUSB0 @ 1000000 baud            │
└─────────────────────────────────────────┘
```

## Quick Start

### 1. Prerequisites

- Raspberry Pi 5 with Raspberry Pi OS
- Docker and Docker Compose installed
- PincherX100 arm connected via USB (`/dev/ttyUSB0`)
- User added to `dialout` group

### 2. Build and Start

```bash
cd /home/jx/Dev/Robotics/pincherx100/ros2_control
docker-compose build
docker-compose up -d
```

### 3. Launch the Controller

```bash
# Enter the container
docker exec -it pincherx100_ros2 bash

# Launch the arm controller
ros2 launch pincherx100_control arm_control.launch.py
```

### 4. Test the Arm

In another terminal:
```bash
# Enter the container
docker exec -it pincherx100_ros2 bash

# Test each joint
ros2 run pincherx100_control test_joints

# Or use preset poses
ros2 service call /go_home std_srvs/srv/Trigger
```

## Documentation

- [**SETUP_GUIDE.md**](./SETUP_GUIDE.md) - Detailed setup instructions for Docker, permissions, and first-time setup
- [**USAGE.md**](./USAGE.md) - How to control the arm using ROS2 topics, services, and actions
- [**CALIBRATION.md**](./CALIBRATION.md) - Calibration procedures and joint limit configuration

## Directory Structure

```
ros2_control/
├── Dockerfile                      # Docker image definition
├── docker-compose.yml              # Docker Compose configuration
├── entrypoint.sh                   # Container entrypoint script
├── README.md                       # This file
├── SETUP_GUIDE.md                  # Setup instructions
├── USAGE.md                        # Usage guide
└── pincherx100_control/            # ROS2 package
    ├── package.xml                 # Package manifest
    ├── setup.py                    # Python package setup
    ├── setup.cfg                   # Setup configuration
    ├── resource/                   # Package resources
    ├── config/                     # Configuration files
    │   └── arm_config.yaml         # Arm configuration
    ├── launch/                     # Launch files
    │   ├── arm_control.launch.py   # Main launch file
    │   └── test_arm.launch.py      # Test launch file
    └── pincherx100_control/        # Python package
        ├── __init__.py
        ├── arm_controller_node.py  # Main controller node
        ├── dynamixel_interface.py  # Hardware interface
        ├── calibrate_arm.py        # Calibration script
        └── test_joints.py          # Test script
```

## ROS2 Interface

### Published Topics

- `/joint_states` (`sensor_msgs/JointState`) - Current joint positions, velocities, and efforts at 50Hz

### Subscribed Topics

- `/joint_position_commands` (`std_msgs/Float64MultiArray`) - Position commands in radians
- `/joint_velocity_commands` (`std_msgs/Float64MultiArray`) - Velocity commands in rad/s
- `/joint_trajectory` (`trajectory_msgs/JointTrajectory`) - Trajectory commands

### Action Servers

- `/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) - Execute joint trajectories with feedback

### Services

- `/go_home` (`std_srvs/srv/Trigger`) - Move to home position
- `/go_sleep` (`std_srvs/srv/Trigger`) - Move to sleep position
- `/go_ready` (`std_srvs/srv/Trigger`) - Move to ready position
- `/gripper_open` (`std_srvs/srv/Trigger`) - Open gripper
- `/gripper_close` (`std_srvs/srv/Trigger`) - Close gripper

## Configuration

Edit `config/arm_config.yaml` to customize:
- Serial port and baudrate
- Joint names and servo IDs
- Movement profile (velocity, acceleration)
- Joint limits
- Preset poses

## Troubleshooting

### Cannot connect to servos
```bash
# Check if device exists
ls -l /dev/ttyUSB0

# Check permissions
groups
# Should include 'dialout'

# If not, add user to dialout group (outside Docker)
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Docker container can't access USB device
```bash
# Check if device is available to Docker
docker exec pincherx100_ros2 ls -l /dev/ttyUSB0

# If not found, restart container
docker-compose down
docker-compose up -d
```

### Servo not responding
```bash
# Run calibration tool to scan servos
docker exec -it pincherx100_ros2 bash
ros2 run pincherx100_control calibrate
```

## Development

### Rebuilding After Code Changes

Since the package is mounted as a volume, most Python changes are reflected immediately. However, if you modify `setup.py` or add new files:

```bash
# Inside container
cd /ros2_ws
colcon build --symlink-install --packages-select pincherx100_control
source install/setup.bash
```

### Viewing Logs

```bash
# Follow container logs
docker-compose logs -f

# Inside container, view ROS2 logs
ros2 node list
ros2 node info /arm_controller
ros2 topic echo /joint_states
```

## Safety Notes

 **Important Safety Information:**

1. **Joint Limits**: Always configure appropriate joint limits in `arm_config.yaml` to prevent mechanical damage
2. **Emergency Stop**: Press Ctrl+C to stop the controller and disable torque
3. **Power**: Ensure adequate power supply (11.1V recommended for XL430-W250)
4. **First Use**: Run calibration script before first use to verify servo directions and limits
5. **Workspace**: Keep workspace clear of obstacles when arm is moving

## License

MIT License - See LICENSE file for details

## Support

For issues and questions:
- Check the troubleshooting section
- Review the setup guide
- Examine ROS2 logs for error messages
- Verify hardware connections and power supply



