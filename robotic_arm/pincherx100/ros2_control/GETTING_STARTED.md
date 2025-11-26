# Getting Started with PincherX100 ROS2 Control

**Welcome!** This guide will get you up and running with the PincherX100 ROS2 control system in Docker.

## ⚡ Quick Start (5 Minutes)

### Prerequisites Check

```bash
# 1. Check Docker is installed
docker --version

# 2. Check USB device
ls -l /dev/ttyUSB0

# 3. Check user permissions
groups | grep dialout
```

If any checks fail, see [SETUP_GUIDE.md](./SETUP_GUIDE.md) for detailed setup instructions.

### Launch the System

```bash
# Navigate to ros2_control directory
cd /home/jx/Dev/Robotics/pincherx100/ros2_control

# Use the quick start script
./quick_start.sh

# Or manually:
# 1. Build Docker image (first time only, ~10-15 minutes)
docker-compose build

# 2. Start container
docker-compose up -d

# 3. Launch arm controller
docker exec -it pincherx100_ros2 bash
ros2 launch pincherx100_control arm_control.launch.py
```

### Test the Arm

In another terminal:

```bash
# Enter container
docker exec -it pincherx100_ros2 bash

# Move to home position
ros2 service call /go_home std_srvs/srv/Trigger

# Open gripper
ros2 service call /gripper_open std_srvs/srv/Trigger

# Close gripper
ros2 service call /gripper_close std_srvs/srv/Trigger
```

**Success!** If the arm moves, you're ready to go! 🎉

---

## 📚 Documentation Overview

This project includes comprehensive documentation:

### Essential Guides

1. **[README.md](./README.md)** - Project overview, features, and architecture
2. **[SETUP_GUIDE.md](./SETUP_GUIDE.md)** - Complete setup instructions (READ THIS FIRST)
3. **[USAGE.md](./USAGE.md)** - How to control the arm with examples
4. **[CALIBRATION.md](./CALIBRATION.md)** - Calibration procedures
5. **[DOCKER_COMMANDS.md](./DOCKER_COMMANDS.md)** - Docker command reference

### Quick Reference

- **Start system**: `./quick_start.sh` (interactive menu)
- **Launch controller**: `ros2 launch pincherx100_control arm_control.launch.py`
- **Enter container**: `docker exec -it pincherx100_ros2 bash`
- **Stop system**: `docker-compose down`

---

## 🎯 What Can This System Do?

### Control Methods

1. **Services** (Easiest - Preset Poses)
   - `/go_home` - Move to home position
   - `/go_ready` - Move to ready position
   - `/gripper_open` - Open gripper
   - `/gripper_close` - Close gripper

2. **Position Commands** (Direct Control)
   - Topic: `/joint_position_commands`
   - Control individual joint positions

3. **Velocity Commands** (Continuous Motion)
   - Topic: `/joint_velocity_commands`
   - Control joint velocities

4. **Trajectory Execution** (Smooth Paths)
   - Action: `/follow_joint_trajectory`
   - Execute multi-waypoint trajectories

5. **Joint State Monitoring**
   - Topic: `/joint_states`
   - Real-time feedback at 50Hz

---

## 🚀 Common Use Cases

### 1. Simple Movement

```bash
# Move to home
ros2 service call /go_home std_srvs/srv/Trigger

# Move to ready position
ros2 service call /go_ready std_srvs/srv/Trigger
```

### 2. Control Individual Joints

```bash
# Move all joints (values in radians)
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [3.14, 3.14, 3.14, 3.14, 3.14]"
```

### 3. Pick and Place (Python Script)

Create a Python script (see [USAGE.md](./USAGE.md) for full example):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class SimplePickPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_place')
        self.home_client = self.create_client(Trigger, '/go_home')
        self.ready_client = self.create_client(Trigger, '/go_ready')
        self.gripper_open_client = self.create_client(Trigger, '/gripper_open')
        self.gripper_close_client = self.create_client(Trigger, '/gripper_close')
    
    def execute(self):
        # Go to home
        self.call_service(self.home_client)
        # Move to ready
        self.call_service(self.ready_client)
        # Open gripper
        self.call_service(self.gripper_open_client)
        # Close gripper (grasp)
        self.call_service(self.gripper_close_client)
        # Return home
        self.call_service(self.home_client)
    
    def call_service(self, client):
        client.wait_for_service()
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

# Run the script
def main():
    rclpy.init()
    node = SimplePickPlace()
    node.execute()
    node.destroy_node()
    rclpy.shutdown()
```

---

## 🔧 First-Time Setup Checklist

Before using the arm for the first time:

- [ ] Docker installed and running
- [ ] User added to `dialout` group for USB access
- [ ] Arm powered on and connected via USB
- [ ] Docker image built: `docker-compose build`
- [ ] Container started: `docker-compose up -d`
- [ ] **Run calibration**: `ros2 run pincherx100_control calibrate`
- [ ] Test basic movement: `ros2 service call /go_home std_srvs/srv/Trigger`

---

## 🐳 Docker Environment

### Why Docker?

- ✅ **No contamination** - Host system stays clean
- ✅ **Reproducible** - Same environment everywhere
- ✅ **Isolated** - All ROS2 dependencies contained
- ✅ **Easy cleanup** - Remove container to clean up

### How It Works

```
Host System (Raspberry Pi OS)
├── /dev/ttyUSB0 ────────────► Mapped to container
├── Source code ─────────────► Volume mounted (live editing)
└── Docker Container
    ├── ROS2 Humble
    ├── Dynamixel SDK
    └── pincherx100_control package
```

### Volume Mounting

The source code is mounted as a volume:
- **Host**: `/home/jx/Dev/Robotics/pincherx100/ros2_control/pincherx100_control`
- **Container**: `/ros2_ws/src/pincherx100_control`

**This means:**
- Edit code on your host (with your favorite editor)
- Changes immediately visible in container
- No need to rebuild Docker image for code changes
- Just restart the ROS2 node if needed

---

## 💡 Tips for Success

### 1. Use Multiple Terminals

You'll typically want 2-3 terminal windows:

**Terminal 1: Controller**
```bash
docker exec -it pincherx100_ros2 bash
ros2 launch pincherx100_control arm_control.launch.py
```

**Terminal 2: Commands**
```bash
docker exec -it pincherx100_ros2 bash
ros2 service call /go_home std_srvs/srv/Trigger
```

**Terminal 3: Monitoring**
```bash
docker exec -it pincherx100_ros2 bash
ros2 topic echo /joint_states
```

### 2. Always Calibrate First

Before serious use, run calibration to:
- Verify all servos work
- Set safe joint limits
- Define home position
- Configure gripper

```bash
docker exec -it pincherx100_ros2 bash
ros2 run pincherx100_control calibrate
```

### 3. Start Simple

Don't jump into complex trajectories right away:
1. First, test preset poses (`/go_home`, `/go_ready`)
2. Then try single joint movements
3. Then try coordinated movements
4. Finally, implement trajectories

### 4. Monitor Joint States

Always watch `/joint_states` to see what the arm is doing:

```bash
ros2 topic echo /joint_states
```

### 5. Emergency Stop

**Press Ctrl+C** in the controller terminal to stop immediately and disable torque.

---

## 🔍 Troubleshooting Quick Reference

### Container won't start
```bash
docker-compose down
docker-compose up -d
docker logs pincherx100_ros2
```

### Can't connect to servos
```bash
# Check device
ls -l /dev/ttyUSB0

# Check device in container
docker exec pincherx100_ros2 ls -l /dev/ttyUSB0

# Restart container
docker-compose restart
```

### Arm not moving
```bash
# Check if controller is running
ros2 node list | grep arm_controller

# Check joint state updates
ros2 topic hz /joint_states

# Check for errors
docker logs pincherx100_ros2
```

### Code changes not taking effect
```bash
# Restart the ROS2 node (Ctrl+C and relaunch)
# Or if setup.py changed:
docker exec pincherx100_ros2 bash -c "cd /ros2_ws && colcon build --symlink-install"
```

---

## 📝 Configuration

Main configuration file:
```
pincherx100_control/config/arm_config.yaml
```

Key settings:
- **port**: Serial port (default: `/dev/ttyUSB0`)
- **baudrate**: Communication speed (default: `1000000`)
- **limits**: Joint position limits (safety)
- **poses**: Preset positions (home, ready, sleep)
- **profile_velocity**: Movement speed (150 = slower but more torque)
- **profile_acceleration**: Movement smoothness (80 = balanced)

After editing, restart the controller to apply changes.

---

## 📖 Learning Path

### Beginner
1. Read [SETUP_GUIDE.md](./SETUP_GUIDE.md)
2. Build and start the system
3. Run calibration
4. Test preset poses
5. Try position commands from command line

### Intermediate
1. Read [USAGE.md](./USAGE.md)
2. Write Python scripts for basic control
3. Monitor joint states in your code
4. Implement simple pick and place

### Advanced
1. Study [arm_controller_node.py](./pincherx100_control/pincherx100_control/arm_controller_node.py)
2. Implement trajectory execution
3. Create custom control applications
4. Integrate with vision or other sensors

---

## 🎓 Example Scripts Location

Find example scripts in [USAGE.md](./USAGE.md):
- Service calling (preset poses)
- Position control
- Velocity control
- Trajectory execution
- Pick and place
- Joint state monitoring

---

## 🆘 Getting Help

### Documentation Order
1. **Quick issue?** → This file or [DOCKER_COMMANDS.md](./DOCKER_COMMANDS.md)
2. **Setup issue?** → [SETUP_GUIDE.md](./SETUP_GUIDE.md)
3. **Usage question?** → [USAGE.md](./USAGE.md)
4. **Calibration?** → [CALIBRATION.md](./CALIBRATION.md)
5. **Architecture?** → [README.md](./README.md)

### Debugging Steps
1. Check Docker container is running: `docker ps`
2. Check logs: `docker logs pincherx100_ros2`
3. Check USB device: `ls -l /dev/ttyUSB0`
4. Check ROS2 node: `ros2 node list`
5. Check joint states: `ros2 topic echo /joint_states`

### Common Error Messages

**"Failed to open port"**
- → Check USB connection and permissions

**"Servo not responding"**
- → Run calibration to scan servos

**"Cannot connect to Docker daemon"**
- → Start Docker service: `sudo systemctl start docker`

**"Permission denied: /dev/ttyUSB0"**
- → Add user to dialout group: `sudo usermod -a -G dialout $USER`

---

## 🎉 Next Steps

Now that you have a working system:

1. **Experiment**: Try different poses and movements
2. **Develop**: Write your own control scripts
3. **Integrate**: Connect with other ROS2 nodes
4. **Customize**: Adjust parameters for your application
5. **Extend**: Add new features or capabilities

---

## 📋 Command Cheat Sheet

```bash
# Build system
docker-compose build

# Start
docker-compose up -d

# Enter container
docker exec -it pincherx100_ros2 bash

# Launch controller
ros2 launch pincherx100_control arm_control.launch.py

# Test services
ros2 service call /go_home std_srvs/srv/Trigger
ros2 service call /gripper_open std_srvs/srv/Trigger

# Monitor
ros2 topic echo /joint_states

# Stop
docker-compose down

# Interactive menu
./quick_start.sh
```

---

## ✅ Quick Verification

After setup, verify everything works:

```bash
# 1. Container running?
docker ps | grep pincherx100

# 2. Can access USB?
docker exec pincherx100_ros2 ls -l /dev/ttyUSB0

# 3. ROS2 working?
docker exec pincherx100_ros2 bash -c "source /ros2_ws/install/setup.bash && ros2 node list"

# 4. Launch controller
docker exec -it pincherx100_ros2 bash
ros2 launch pincherx100_control arm_control.launch.py

# 5. In another terminal, test movement
docker exec -it pincherx100_ros2 bash
ros2 service call /go_home std_srvs/srv/Trigger
```

If all steps succeed, you're ready to go! 🚀

---

**Happy Robot Programming!** 🤖

For detailed information, please refer to the comprehensive documentation files in this directory.




