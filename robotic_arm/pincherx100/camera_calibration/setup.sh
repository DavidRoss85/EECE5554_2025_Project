#!/bin/bash
# Setup script for camera calibration environment

echo "=========================================="
echo "PincherX100 Camera Calibration Setup"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model)
    echo "Detected: $MODEL"
else
    echo "Running on non-Raspberry Pi system"
fi

echo ""
echo "Creating virtual environment..."
python3 -m venv venv

echo "Activating virtual environment..."
source venv/bin/activate

echo "Upgrading pip..."
pip install --upgrade pip

echo ""
echo "Installing dependencies..."
pip install -r requirements.txt

echo ""
echo "=========================================="
echo "✓ Setup Complete!"
echo "=========================================="
echo ""
echo "To use the calibration tools:"
echo "  1. Activate environment:  source venv/bin/activate"
echo "  2. Start camera stream:   python camera_stream_server.py"
echo "  3. Run calibration:       python calibrate_camera.py"
echo ""
echo "See README.md for detailed instructions"
echo "=========================================="

