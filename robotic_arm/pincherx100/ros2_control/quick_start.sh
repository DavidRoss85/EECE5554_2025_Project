#!/bin/bash
# Quick start script for PincherX100 ROS2 Control

set -e

echo "=========================================="
echo "PincherX100 ROS2 Quick Start"
echo "=========================================="
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    echo "Please install Docker first. See SETUP_GUIDE.md"
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: Docker Compose is not available"
    echo "Please install Docker Compose. See SETUP_GUIDE.md"
    exit 1
fi

# Determine docker-compose command
if command -v docker-compose &> /dev/null; then
    DC_CMD="docker-compose"
else
    DC_CMD="docker compose"
fi

# Check if USB device exists
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "Warning: /dev/ttyUSB0 not found"
    echo "Please ensure the arm is powered and connected via USB"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if container is already running
if [ "$(docker ps -q -f name=pincherx100_ros2)" ]; then
    echo "Container is already running"
    ACTION="restart"
else
    echo "Container is not running"
    ACTION="start"
fi

# Main menu
echo ""
echo "Select an option:"
echo "  1) Build Docker image"
echo "  2) Start container"
echo "  3) Stop container"
echo "  4) Launch arm controller"
echo "  5) Run calibration"
echo "  6) Run joint test"
echo "  7) Enter container shell"
echo "  8) View logs"
echo "  9) Clean up (remove container and image)"
echo "  0) Exit"
echo ""
read -p "Enter choice [0-9]: " choice

case $choice in
    1)
        echo "Building Docker image..."
        $DC_CMD build
        echo "Build complete!"
        ;;
    2)
        echo "Starting container..."
        $DC_CMD up -d
        echo "Container started!"
        echo "Run '$0' again and select option 4 to launch the controller"
        ;;
    3)
        echo "Stopping container..."
        $DC_CMD down
        echo "Container stopped!"
        ;;
    4)
        echo "Launching arm controller..."
        if [ ! "$(docker ps -q -f name=pincherx100_ros2)" ]; then
            echo "Starting container first..."
            $DC_CMD up -d
            sleep 2
        fi
        docker exec -it pincherx100_ros2 \
            bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch pincherx100_control arm_control.launch.py"
        ;;
    5)
        echo "Running calibration..."
        if [ ! "$(docker ps -q -f name=pincherx100_ros2)" ]; then
            echo "Starting container first..."
            $DC_CMD up -d
            sleep 2
        fi
        docker exec -it pincherx100_ros2 \
            bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run pincherx100_control calibrate"
        ;;
    6)
        echo "Running joint test..."
        if [ ! "$(docker ps -q -f name=pincherx100_ros2)" ]; then
            echo "Starting container first..."
            $DC_CMD up -d
            sleep 2
        fi
        docker exec -it pincherx100_ros2 \
            bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run pincherx100_control test_joints"
        ;;
    7)
        echo "Entering container shell..."
        if [ ! "$(docker ps -q -f name=pincherx100_ros2)" ]; then
            echo "Starting container first..."
            $DC_CMD up -d
            sleep 2
        fi
        docker exec -it pincherx100_ros2 bash
        ;;
    8)
        echo "Viewing logs (Ctrl+C to exit)..."
        $DC_CMD logs -f
        ;;
    9)
        read -p "This will remove the container and image. Continue? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "Stopping and removing container..."
            $DC_CMD down
            echo "Removing image..."
            docker rmi ros2_control-pincherx100 || true
            echo "Cleanup complete!"
        fi
        ;;
    0)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "Done!"



