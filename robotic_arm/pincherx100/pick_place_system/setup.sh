#!/bin/bash
# Setup script for Pick and Place System

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "PincherX100 Pick and Place System Setup"
echo "=========================================="

# Create virtual environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements
echo "Installing Python packages..."
pip install -r requirements.txt

# Create necessary directories
echo "Creating directories..."
mkdir -p calibration
mkdir -p logs

# Make scripts executable
echo "Making scripts executable..."
chmod +x scripts/*.py
chmod +x vision/*.py
chmod +x calibration/*.py

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "To activate the virtual environment:"
echo "  source venv/bin/activate"
echo ""
echo "Next steps:"
echo "1. Complete hand-eye calibration:"
echo "   python calibration/hand_eye_calibration.py"
echo ""
echo "2. Test object detection:"
echo "   python vision/object_detector.py"
echo ""
echo "3. Run CLI:"
echo "   python scripts/arm_cli.py status"
echo ""
echo "4. Start web interface:"
echo "   python ui/web_interface.py"
echo ""

