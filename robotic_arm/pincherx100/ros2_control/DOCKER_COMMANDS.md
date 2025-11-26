# Docker Commands Reference

Quick reference guide for Docker commands used with PincherX100 ROS2 Control.

## Quick Start Script

The easiest way to manage the container:

```bash
./quick_start.sh
```

This interactive script provides menu options for all common operations.

---

## Building

### Build the Image

```bash
# Build from scratch
docker-compose build

# Build without cache (clean build)
docker-compose build --no-cache

# Build with progress output
docker-compose build --progress=plain
```

### Check Build Status

```bash
# List images
docker images | grep pincherx100

# Image details
docker image inspect ros2_control-pincherx100
```

---

## Starting and Stopping

### Start Container

```bash
# Start in background (detached mode)
docker-compose up -d

# Start in foreground (see logs)
docker-compose up

# Start and rebuild if needed
docker-compose up -d --build
```

### Stop Container

```bash
# Stop container (keeps it for restart)
docker-compose stop

# Stop and remove container
docker-compose down

# Stop, remove container, and remove volumes
docker-compose down -v
```

### Restart Container

```bash
# Restart
docker-compose restart

# Or stop then start
docker-compose down
docker-compose up -d
```

---

## Accessing the Container

### Interactive Shell

```bash
# Enter container with bash
docker exec -it pincherx100_ros2 bash

# Enter as different user (if needed)
docker exec -it -u root pincherx100_ros2 bash
```

### Run Single Command

```bash
# Run command without entering shell
docker exec pincherx100_ros2 ros2 node list

# Run command with environment sourced
docker exec pincherx100_ros2 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list"
```

### Run ROS2 Commands Directly

```bash
# Launch controller
docker exec -it pincherx100_ros2 \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 launch pincherx100_control arm_control.launch.py"

# Call service
docker exec pincherx100_ros2 \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 service call /go_home std_srvs/srv/Trigger"

# Echo topic
docker exec pincherx100_ros2 \
  bash -c "source /ros2_ws/install/setup.bash && \
  ros2 topic echo /joint_states"
```

---

## Viewing Logs

### Container Logs

```bash
# View all logs
docker logs pincherx100_ros2

# Follow logs (real-time)
docker logs -f pincherx100_ros2

# Last 100 lines
docker logs --tail 100 pincherx100_ros2

# With timestamps
docker logs -t pincherx100_ros2

# Using docker-compose
docker-compose logs -f
```

### ROS2 Logs

```bash
# Inside container, view ROS2 logs
docker exec -it pincherx100_ros2 bash
cd ~/.ros/log
ls -lt  # List log directories by time

# View latest log
cd $(ls -t | head -1)
cat *  # View all logs in directory
```

---

## Container Status and Info

### Check if Running

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# Filter by name
docker ps -f name=pincherx100
```

### Container Details

```bash
# Inspect container
docker inspect pincherx100_ros2

# Get IP address
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' pincherx100_ros2

# Check resource usage
docker stats pincherx100_ros2

# Check processes in container
docker top pincherx100_ros2
```

### Network Mode

Since we use `network_mode: host`, the container shares the host's network:

```bash
# Verify network mode
docker inspect pincherx100_ros2 | grep NetworkMode
# Should show: "NetworkMode": "host"
```

---

## File Operations

### Copy Files to/from Container

```bash
# Copy file to container
docker cp /host/path/file.txt pincherx100_ros2:/container/path/

# Copy file from container
docker cp pincherx100_ros2:/container/path/file.txt /host/path/

# Copy directory
docker cp /host/path/dir pincherx100_ros2:/container/path/
```

### Edit Files in Container

```bash
# Enter container and edit
docker exec -it pincherx100_ros2 bash
nano /ros2_ws/src/pincherx100_control/config/arm_config.yaml

# Or edit on host (files are mounted as volumes)
nano pincherx100_control/config/arm_config.yaml
```

---

## Volume Management

### List Volumes

```bash
# List all volumes
docker volume ls

# Inspect specific volume
docker volume inspect <volume_name>
```

### Mounted Directories

The source code is mounted as a volume:
- **Host**: `./pincherx100_control`
- **Container**: `/ros2_ws/src/pincherx100_control`

Changes on the host are immediately visible in the container.

---

## Rebuilding Package Inside Container

After code changes:

```bash
# Enter container
docker exec -it pincherx100_ros2 bash

# Rebuild package
cd /ros2_ws
colcon build --symlink-install --packages-select pincherx100_control

# Source the workspace
source install/setup.bash

# Run node
ros2 run pincherx100_control arm_controller
```

Or rebuild without entering container:

```bash
docker exec pincherx100_ros2 bash -c \
  "cd /ros2_ws && colcon build --symlink-install --packages-select pincherx100_control"
```

---

## Troubleshooting Commands

### Container Won't Start

```bash
# Check logs for errors
docker-compose logs

# Remove old container and try again
docker rm -f pincherx100_ros2
docker-compose up -d

# Check for port conflicts
docker ps -a
```

### USB Device Access Issues

```bash
# Check if device is accessible on host
ls -l /dev/ttyUSB0

# Check if device is accessible in container
docker exec pincherx100_ros2 ls -l /dev/ttyUSB0

# Check permissions
docker exec pincherx100_ros2 groups

# If device not found, restart container
docker-compose restart
```

### Container is Slow

```bash
# Check resource usage
docker stats pincherx100_ros2

# Check system resources
free -h
df -h
```

### ROS2 Not Working

```bash
# Check if ROS2 environment is sourced
docker exec pincherx100_ros2 printenv | grep ROS

# Manually source and test
docker exec -it pincherx100_ros2 bash
source /opt/ros/humble/setup.bash
ros2 --help
```

---

## Cleanup Operations

### Remove Container

```bash
# Stop and remove container
docker-compose down

# Force remove if stopped
docker rm -f pincherx100_ros2
```

### Remove Image

```bash
# Remove image
docker rmi ros2_control-pincherx100

# Force remove
docker rmi -f ros2_control-pincherx100
```

### Clean All Docker Data

```bash
# Remove stopped containers
docker container prune

# Remove unused images
docker image prune

# Remove unused volumes
docker volume prune

# Remove everything unused
docker system prune -a

# Free up space (removes everything not currently used)
docker system prune -a --volumes
```

---

## Multiple Terminal Sessions

### Method 1: Multiple docker exec

```bash
# Terminal 1
docker exec -it pincherx100_ros2 bash

# Terminal 2
docker exec -it pincherx100_ros2 bash

# Terminal 3
docker exec -it pincherx100_ros2 bash
```

### Method 2: tmux Inside Container

```bash
# Enter container
docker exec -it pincherx100_ros2 bash

# Install tmux if not present
apt-get update && apt-get install -y tmux

# Start tmux
tmux

# Tmux commands:
# Ctrl+b then " : split horizontally
# Ctrl+b then % : split vertically
# Ctrl+b then arrow keys : navigate between panes
# Ctrl+b then c : create new window
# Ctrl+b then d : detach from tmux (keeps running)
# tmux attach : reattach to tmux session
```

### Method 3: screen Inside Container

```bash
# Enter container
docker exec -it pincherx100_ros2 bash

# Install screen if not present
apt-get update && apt-get install -y screen

# Start screen
screen

# Screen commands:
# Ctrl+a then c : create new window
# Ctrl+a then n : next window
# Ctrl+a then p : previous window
# Ctrl+a then d : detach from screen
# screen -r : reattach to screen session
```

---

## Updating the Container

### After Dockerfile Changes

```bash
# Rebuild image
docker-compose build --no-cache

# Stop old container
docker-compose down

# Start new container
docker-compose up -d
```

### After Source Code Changes

With volume mount, most changes are immediate:

```bash
# Python code changes: No rebuild needed, just restart node
# setup.py changes: Rebuild package (see above)
# Config changes: Restart controller
```

---

## Backup and Restore

### Backup Configuration

```bash
# Backup from host
cp -r pincherx100_control/config pincherx100_control/config.backup

# Backup from container
docker cp pincherx100_ros2:/ros2_ws/src/pincherx100_control/config \
  ./config_backup_$(date +%Y%m%d)
```

### Export Container State

```bash
# Commit container to new image
docker commit pincherx100_ros2 pincherx100_backup:$(date +%Y%m%d)

# Export image to file
docker save pincherx100_backup:latest > pincherx100_backup.tar

# Import image from file (on another machine)
docker load < pincherx100_backup.tar
```

---

## Advanced Docker Operations

### Modify Container Settings

To change container settings, edit `docker-compose.yml` and recreate:

```bash
# Edit docker-compose.yml
nano docker-compose.yml

# Recreate container with new settings
docker-compose down
docker-compose up -d
```

### Add Environment Variables

```yaml
# In docker-compose.yml
environment:
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CUSTOM_VAR=value
```

### Change Device Mapping

```yaml
# In docker-compose.yml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/ttyUSB1:/dev/ttyUSB1  # Add more devices
```

---

## Performance Monitoring

### Monitor Container

```bash
# Real-time stats
docker stats pincherx100_ros2

# Detailed stats (JSON)
docker stats --no-stream --format "{{json .}}" pincherx100_ros2 | jq
```

### Monitor Processes

```bash
# List processes in container
docker top pincherx100_ros2

# More detailed process info
docker exec pincherx100_ros2 ps aux
docker exec pincherx100_ros2 htop
```

### Check Logs Size

```bash
# Check container logs size
docker inspect --format='{{.LogPath}}' pincherx100_ros2 | xargs ls -lh
```

---

## Security

### Running as Non-Root (Optional)

To run container as non-root user:

```yaml
# In docker-compose.yml
user: "1000:1000"  # Your user:group IDs
```

### Limit Resources (Optional)

```yaml
# In docker-compose.yml
deploy:
  resources:
    limits:
      cpus: '2.0'
      memory: 2G
```

---

## Common Workflows

### Daily Development Workflow

```bash
# 1. Start container (if not running)
docker-compose up -d

# 2. Edit code on host
nano pincherx100_control/pincherx100_control/arm_controller_node.py

# 3. Restart node (if needed) or it auto-reloads
docker exec -it pincherx100_ros2 bash
# Inside container, restart your ROS2 node

# 4. Test changes
ros2 service call /go_home std_srvs/srv/Trigger

# 5. Stop container when done
docker-compose down
```

### Testing Workflow

```bash
# Start container
docker-compose up -d

# Run tests
docker exec pincherx100_ros2 bash -c \
  "cd /ros2_ws && colcon test --packages-select pincherx100_control"

# View test results
docker exec pincherx100_ros2 bash -c \
  "cd /ros2_ws && colcon test-result --verbose"
```

---

## Help and Information

```bash
# Docker help
docker --help
docker-compose --help

# Command-specific help
docker exec --help
docker logs --help

# Inside container, ROS2 help
docker exec pincherx100_ros2 bash -c \
  "source /ros2_ws/install/setup.bash && ros2 --help"
```

---

## Quick Reference Card

| Operation | Command |
|-----------|---------|
| Build | `docker-compose build` |
| Start | `docker-compose up -d` |
| Stop | `docker-compose down` |
| Enter shell | `docker exec -it pincherx100_ros2 bash` |
| View logs | `docker logs -f pincherx100_ros2` |
| Restart | `docker-compose restart` |
| Run command | `docker exec pincherx100_ros2 <command>` |
| Check status | `docker ps` |
| Clean up | `docker system prune -a` |

---

For more information:
- [README.md](./README.md) - Project overview
- [SETUP_GUIDE.md](./SETUP_GUIDE.md) - Detailed setup instructions
- [USAGE.md](./USAGE.md) - Usage examples




