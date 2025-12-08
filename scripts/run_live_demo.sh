#!/bin/bash
echo "Launching live demo..."

# Resolve script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

FRONTEND_DIR="$PROJECT_ROOT/front_end"
DEMO_DIR="$PROJECT_ROOT/object_detection"

FRONTEND_SCRIPT="$FRONTEND_DIR/launch_frontend.sh"
APPROACH_SCRIPT="$DEMO_DIR/demo_approach.sh"

# Save starting directory for later return
START_DIR="$(pwd)"

echo "Updating permissions..."
chmod +x "$FRONTEND_SCRIPT" "$APPROACH_SCRIPT"

echo "Sourcing ROS..."
source /opt/ros/jazzy/setup.bash

echo "Launching front end..."
cd "$FRONTEND_DIR" || {
    echo "ERROR: Unable to enter directory: $FRONTEND_DIR"
    echo "Check that the front_end folder exists and contains launch_frontend.sh."
    exit 1
}
./launch_frontend.sh

# Return to project root
cd "$PROJECT_ROOT" || {
    echo "ERROR: Unable to return to project root: $PROJECT_ROOT"
    exit 1
}

sleep 2

echo "Launching main pipeline..."
cd "$DEMO_DIR" || {
    echo "ERROR: Unable to enter directory: $DEMO_DIR"
    echo "Check that the object_detection folder exists and contains demo_approach.sh."
    exit 1
}
./demo_approach.sh

# Return to previous directory
cd "$START_DIR" || {
    echo "WARNING: Could not return to starting directory: $START_DIR"
}
