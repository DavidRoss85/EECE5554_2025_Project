#!/bin/bash
# Quick start script for camera calibration

echo "=========================================="
echo "PincherX100 Camera Calibration Quick Start"
echo "=========================================="
echo ""

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Running setup..."
    bash setup.sh
    echo ""
fi

# Activate venv
echo "Activating virtual environment..."
source venv/bin/activate

# Check if camera is available
echo ""
echo "Checking for camera..."
python3 -c "import cv2; cap=cv2.VideoCapture(0); print('Camera found' if cap.isOpened() else 'No camera found'); cap.release()" 

echo ""
echo "=========================================="
echo "What would you like to do?"
echo "=========================================="
echo "1. Start camera stream server (for viewing on Mac)"
echo "2. Run camera calibration"
echo "3. Test calibration (view undistorted video)"
echo "4. Exit"
echo ""
read -p "Enter your choice (1-4): " choice

case $choice in
    1)
        echo ""
        echo "Starting camera stream server..."
        echo "Press Ctrl+C to stop"
        python camera_stream_server.py
        ;;
    2)
        echo ""
        echo "Starting camera calibration..."
        echo "Make sure you have the printed checkerboard ready!"
        sleep 2
        python calibrate_camera.py
        ;;
    3)
        if [ ! -f "camera_calibration.npz" ]; then
            echo ""
            echo "No calibration file found."
            echo "Please run camera calibration first (option 2)"
        else
            echo ""
            echo "Testing calibration..."
            python test_calibration.py
        fi
        ;;
    4)
        echo "Goodbye!"
        exit 0
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

