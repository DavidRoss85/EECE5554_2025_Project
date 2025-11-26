#!/bin/bash
# Setup script for PincherX100 low-level control

echo "=========================================="
echo "PincherX100 Low-Level Control Setup"
echo "=========================================="

# Create virtual environment
echo ""
echo "Creating Python virtual environment..."
python3 -m venv venv

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements
echo "Installing required packages..."
pip install -r requirements.txt

# Make scripts executable
chmod +x scan_servos.py
chmod +x control_arm.py

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "To use:"
echo "  1. Activate the virtual environment:"
echo "     source venv/bin/activate"
echo ""
echo "  2. First, scan for servos:"
echo "     python scan_servos.py"
echo ""
echo "  3. Then run the control program:"
echo "     python control_arm.py"
echo ""
echo "     Or with different baudrate:"
echo "     python control_arm.py 1000000"
echo ""
echo "=========================================="

