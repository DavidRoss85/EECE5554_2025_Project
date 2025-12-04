#!/bin/bash

# Setup Script for Three Bottle Pick and Place System
# This script sets up the Python virtual environment and installs dependencies

set -e  # Exit on error

echo "=========================================="
echo "Three Bottle Pick and Place System Setup"
echo "=========================================="
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check Python version
echo "Checking Python version..."
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Found Python $PYTHON_VERSION"

# Check if Python 3.8+
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 8 ]); then
    echo "ERROR: Python 3.8 or higher is required"
    echo "Current version: $PYTHON_VERSION"
    exit 1
fi

echo "✓ Python version OK"
echo ""

# Create virtual environment
if [ -d "venv" ]; then
    echo "Virtual environment already exists"
    read -p "Do you want to recreate it? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing old virtual environment..."
        rm -rf venv
        echo "Creating new virtual environment..."
        python3 -m venv venv
    fi
else
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

echo "✓ Virtual environment ready"
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

echo "✓ pip upgraded"
echo ""

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

echo "✓ Dependencies installed"
echo ""

# Check if camera calibration exists
if [ ! -f "calibration/camera_calibration.npz" ]; then
    echo "⚠ WARNING: Camera calibration file not found"
    echo "  Expected: calibration/camera_calibration.npz"
    echo "  The system will use default parameters (less accurate)"
    echo "  Please copy your camera calibration file to:"
    echo "    $SCRIPT_DIR/calibration/camera_calibration.npz"
    echo ""
fi

# Create logs directory
mkdir -p logs

echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Quick Start:"
echo "  1. Activate environment:  source venv/bin/activate"
echo "  2. Generate AprilTags:    python scripts/generate_apriltag_pdf.py --all --size 1.0"
echo "  3. Check system status:   python scripts/pick_place.py status"
echo "  4. Detect bottles:        python scripts/pick_place.py detect"
echo "  5. Pick a bottle:         python scripts/pick_place.py pick orange"
echo ""
echo "For more information, see README.md"
echo ""

