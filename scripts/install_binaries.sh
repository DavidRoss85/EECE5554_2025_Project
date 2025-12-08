#!/bin/bash

# ---------------------------------------------------------
# INSTALL SYSTEM-WIDE DEPENDENCIES (ROS 2 JAZZY + NAV2 + TB4)
# ---------------------------------------------------------

echo "[INFO] Updating APT..."
sudo apt update

echo "[INFO] Installing ROS 2 Jazzy core packages..."
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-base

# ---------------------------------------------------------
# NAVIGATION 2 PACKAGES
# ---------------------------------------------------------
echo "[INFO] Installing Nav2 binaries..."

sudo apt install -y ros-jazzy-navigation2
sudo apt install -y ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-nav2-minimal-tb\*
sudo apt install -y ros-jazzy-nav2-route        # Not shown on website but valid

# ---------------------------------------------------------
# TURTLEBOT4 PACKAGES
# ---------------------------------------------------------
echo "[INFO] Installing TurtleBot4 ROS packages..."

sudo apt install -y \
    ros-jazzy-turtlebot4-description \
    ros-jazzy-turtlebot4-msgs \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-turtlebot4-node

# ---------------------------------------------------------
# TELEOP
# ---------------------------------------------------------
echo "[INFO] Installing teleop tools..."

sudo apt install -y ros-jazzy-teleop-twist-keyboard

# ---------------------------------------------------------
# ROSBRIDGE SERVER
# ---------------------------------------------------------
echo "[INFO] Installing rosbridge..."

sudo apt install -y ros-jazzy-rosbridge-server

echo "[INFO] System binary installation complete!"
