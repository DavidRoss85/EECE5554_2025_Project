# PincherX100 ROS2 Setup Guide

Complete setup guide for running ROS2 Humble control of PincherX100 on Raspberry Pi 5 with Docker.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Docker Installation](#docker-installation)
3. [Permissions Setup](#permissions-setup)
4. [Building the Docker Image](#building-the-docker-image)
5. [First Time Setup](#first-time-setup)
6. [Running the Controller](#running-the-controller)
7. [Verification](#verification)

---

## System Requirements

### Hardware
- Raspberry Pi 5
- PincherX100 robotic arm with XL430-W250 servos
- USB cable for Dynamixel communication
- Adequate power supply for servos (11.1V recommended)

### Software
- Raspberry Pi OS (64-bit recommended)
- Docker Engine 20.10+
- Docker Compose 1.29+
- At least 4GB RAM
- 10GB free disk space

---

## Docker Installation

### Step 1: Install Docker Engine

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade -y

# Install prerequisites
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the repository (Raspberry Pi OS is based on Debian)
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Step 2: Verify Docker Installation

```bash
# Check Docker version
docker --version

# Should output something like: Docker version 24.0.x, build xxxxx

# Test Docker
sudo docker run hello-world
```

### Step 3: Add User to Docker Group (Optional but Recommended)

```bash
# Add current user to docker group
sudo usermod -aG docker $USER

# Apply group changes (logout/login or use newgrp)
newgrp docker

# Verify (should work without sudo now)
docker ps
```

---

## Permissions Setup

### Step 1: USB Device Permissions

The arm communicates via `/dev/ttyUSB0`. You need proper permissions:

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Verify groups
groups
# Should include 'dialout'

# Log out and log back in for changes to take effect
# Or use: newgrp dialout
```

### Step 2: Verify USB Device

```bash
# Check if device exists
ls -l /dev/ttyUSB0

# Should show something like:
# crw-rw---- 1 root dialout 188, 0 Nov 21 10:00 /dev/ttyUSB0

# If not found, check with arm powered and connected:
dmesg | grep tty
# Look for lines about USB serial converter
```

### Step 3: Install udev Rules (Optional)

For consistent device naming:

```bash
# Create udev rule
sudo nano /etc/udev/rules.d/99-dynamixel.rules

# Add this line:
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", SYMLINK+="ttyDynamixel"

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Now you can also use /dev/ttyDynamixel
```

---

## Building the Docker Image

### Step 1: Navigate to Project Directory

```bash
cd /home/jx/Dev/Robotics/pincherx100/ros2_control
```

### Step 2: Review Docker Configuration

```bash
# Check Dockerfile exists
ls -la Dockerfile

# Check docker-compose.yml
cat docker-compose.yml
```

The `docker-compose.yml` should look like:
```yaml
version: '3.8'

services:
  pincherx100:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: pincherx100_ros2
    network_mode: host
    privileged: true
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    volumes:
      - ./pincherx100_control:/ros2_ws/src/pincherx100_control:rw
    environment:
      - ROS_DOMAIN_ID=0
    stdin_open: true
    tty: true
    command: bash
```

### Step 3: Build the Docker Image

```bash
# Build the image (this will take 10-15 minutes on Raspberry Pi 5)
docker-compose build

# Monitor build progress
# The build will:
# 1. Pull ROS2 Humble base image
# 2. Install dependencies
# 3. Install Dynamixel SDK
# 4. Build the workspace
```

### Step 4: Verify Build

```bash
# Check if image was created
docker images | grep pincherx100

# Should show something like:
# ros2_control-pincherx100   latest   <image-id>   <time>   <size>
```

---

## First Time Setup

### Step 1: Start the Container

```bash
# Start container in detached mode
docker-compose up -d

# Verify container is running
docker ps

# Should show:
# CONTAINER ID   IMAGE                     COMMAND   ...   STATUS          NAMES
# <container-id> ros2_control-pincherx100  "bash"   ...   Up X seconds    pincherx100_ros2
```

### Step 2: Enter the Container

```bash
# Open an interactive shell in the container
docker exec -it pincherx100_ros2 bash

# You should now be inside the container
# Prompt should show: root@<hostname>:/ros2_ws#
```

### Step 3: Verify ROS2 Installation

Inside the container:

```bash
# Check ROS2 environment
printenv | grep ROS

# Should show ROS_DISTRO=humble and other ROS variables

# List available packages
ros2 pkg list | grep pincherx100

# Should show: pincherx100_control

# Check Dynamixel SDK
python3 -c "from dynamixel_sdk import *; print('Dynamixel SDK OK')"
```

### Step 4: Run Calibration

Before first use, calibrate the arm:

```bash
# Inside container
ros2 run pincherx100_control calibrate

# Follow the interactive prompts to:
# 1. Scan for servos
# 2. Test range of motion
# 3. Set joint limits
# 4. Define home position
# 5. Set gripper positions
# 6. Generate configuration file
```

The calibration will create/update: `/ros2_ws/src/pincherx100_control/config/arm_config.yaml`

---

## Running the Controller

### Method 1: Using Launch File (Recommended)

```bash
# Inside container
ros2 launch pincherx100_control arm_control.launch.py

# With custom port/baudrate:
ros2 launch pincherx100_control arm_control.launch.py port:=/dev/ttyUSB0 baudrate:=1000000
```

### Method 2: Direct Node Execution

```bash
# Inside container
ros2 run pincherx100_control arm_controller
```

### Method 3: Using Docker Exec (From Host)

```bash
# From host (outside container)
docker exec -it pincherx100_ros2 ros2 launch pincherx100_control arm_control.launch.py
```

### Expected Output

When the controller starts successfully, you should see:

```
[INFO] [arm_controller]: Connecting to /dev/ttyUSB0 at 1000000 baud...
[INFO] [arm_controller]: Connected to servos
[INFO] [arm_controller]: Servo 1 (base) found
[INFO] [arm_controller]: Servo 2 (shoulder) found
[INFO] [arm_controller]: Servo 3 (elbow) found
[INFO] [arm_controller]: Servo 4 (wrist) found
[INFO] [arm_controller]: Servo 5 (gripper) found
[INFO] [arm_controller]: Configuring servos...
[INFO] [arm_controller]:   base: limits [512, 3584]
[INFO] [arm_controller]:   shoulder: limits [1024, 3072]
...
[INFO] [arm_controller]: Hardware initialization complete
[INFO] [arm_controller]: PincherX100 Controller Node initialized
```

---

## Verification

### Step 1: Check Node Status

In a new terminal (on host):
```bash
# Enter container
docker exec -it pincherx100_ros2 bash

# List running nodes
ros2 node list
# Should show: /arm_controller

# Get node info
ros2 node info /arm_controller
```

### Step 2: Monitor Joint States

```bash
# Inside container
ros2 topic echo /joint_states

# Should show continuous updates with joint positions, velocities, efforts
```

### Step 3: Test Services

```bash
# Inside container

# Move to home position
ros2 service call /go_home std_srvs/srv/Trigger

# Expected response:
# success: True
# message: 'Moved to home position'

# Test gripper
ros2 service call /gripper_open std_srvs/srv/Trigger
ros2 service call /gripper_close std_srvs/srv/Trigger
```

### Step 4: Test Position Commands

```bash
# Inside container

# Send position command (all joints to center: 2048 = π rad)
ros2 topic pub --once /joint_position_commands std_msgs/msg/Float64MultiArray \
  "data: [3.14, 3.14, 3.14, 3.14, 3.14]"

# Arm should move to center positions
```

### Step 5: Run Joint Test

```bash
# Inside container
ros2 run pincherx100_control test_joints

# Follow prompts to test each joint individually
```

---

## Working with Multiple Terminals

### Option 1: Multiple Docker Exec Sessions

```bash
# Terminal 1: Run controller
docker exec -it pincherx100_ros2 bash
ros2 launch pincherx100_control arm_control.launch.py

# Terminal 2: Send commands
docker exec -it pincherx100_ros2 bash
ros2 service call /go_home std_srvs/srv/Trigger

# Terminal 3: Monitor topics
docker exec -it pincherx100_ros2 bash
ros2 topic echo /joint_states
```

### Option 2: Using tmux Inside Container

```bash
# Enter container
docker exec -it pincherx100_ros2 bash

# Install tmux (if needed)
apt-get update && apt-get install -y tmux

# Start tmux
tmux

# Split windows:
# Ctrl+b then " (horizontal split)
# Ctrl+b then % (vertical split)
# Ctrl+b then arrow keys (navigate)
```

---

## Container Management

### Start/Stop Container

```bash
# Start
docker-compose up -d

# Stop
docker-compose down

# Restart
docker-compose restart

# View logs
docker-compose logs -f
```

### Rebuild After Changes

```bash
# If you modify Dockerfile or add new dependencies
docker-compose down
docker-compose build --no-cache
docker-compose up -d

# If you only change Python code (with volume mount)
# No rebuild needed! Changes are reflected immediately
# Just restart the ROS2 node
```

### Clean Up

```bash
# Stop and remove container
docker-compose down

# Remove image
docker rmi ros2_control-pincherx100

# Remove all unused Docker data
docker system prune -a
```

---

## Troubleshooting

### Container won't start

```bash
# Check logs
docker-compose logs

# Check if port is available
docker ps -a

# Remove old container if exists
docker rm -f pincherx100_ros2
docker-compose up -d
```

### USB device not accessible in container

```bash
# Verify device on host
ls -l /dev/ttyUSB0

# Check container has access
docker exec pincherx100_ros2 ls -l /dev/ttyUSB0

# If not found, restart container
docker-compose down
docker-compose up -d
```

### Build fails

```bash
# Check internet connection
ping google.com

# Clean and rebuild
docker-compose down
docker system prune
docker-compose build --no-cache
```

### Servos not responding

```bash
# Inside container, check connection
python3 << EOF
from dynamixel_sdk import *
ph = PortHandler('/dev/ttyUSB0')
pp = PacketHandler(2.0)
ph.openPort()
ph.setBaudRate(1000000)
model, result, error = pp.ping(ph, 1)
print(f"Ping result: {result}, model: {model}")
EOF
```

---

## Next Steps

After successful setup:

1. Read [USAGE.md](./USAGE.md) for detailed usage instructions
2. Experiment with different commands and trajectories
3. Adjust configuration in `config/arm_config.yaml` as needed
4. Develop your own control applications using the ROS2 interface

---

## Development Workflow

### Recommended workflow for development:

1. Code changes are made in `pincherx100_control/` directory (on host)
2. Changes are immediately visible in container (due to volume mount)
3. Restart ROS2 node to pick up changes (if needed)
4. For new files or setup.py changes, rebuild package:
   ```bash
   # Inside container
   cd /ros2_ws
   colcon build --symlink-install --packages-select pincherx100_control
   source install/setup.bash
   ```

### No contamination of host system:
- All ROS2 packages and dependencies are inside Docker
- Only the source code directory is mounted
- Host system remains clean
- Can completely remove Docker container/image anytime

---

## Support

If you encounter issues not covered here:

1. Check Docker logs: `docker-compose logs`
2. Check ROS2 logs: Inside container, check `/root/.ros/log/`
3. Verify hardware: Ensure arm is powered and USB connected
4. Test low-level control: Try the scripts in `../low-level_control/` directly on host to verify hardware




