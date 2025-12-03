# Setup Guide

Complete setup instructions for the PincherX100 Pick and Place System.

## Prerequisites

- Raspberry Pi 5 running Raspberry Pi OS
- PincherX100 robotic arm connected via USB
- 720p webcam connected to Raspberry Pi
- Camera calibration completed (see `../camera_calibration/`)

## System Requirements

- Python 3.8 or higher
- 8GB RAM (for YOLO model)
- USB port for robotic arm
- USB port or CSI for camera

## Installation Steps

### 1. Navigate to Project Directory

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_place_system
```

### 2. Run Setup Script

```bash
chmod +x setup.sh
./setup.sh
```

This will:
- Create a Python virtual environment
- Install all required packages
- Create necessary directories
- Make scripts executable

### 3. Activate Virtual Environment

```bash
source venv/bin/activate
```

### 4. Verify Installation

Test that all components are accessible:

```bash
python -c "import cv2; import ultralytics; print('OK')"
```

## Configuration

### Robot Configuration

Edit `configs/robot_config.yaml` to adjust:
- Robot dimensions (if different from default)
- Servo IDs (if different from default)
- Object dimensions
- Pick and place parameters
- Camera position relative to base

### Camera Position

The default camera position in the config assumes:
- Camera optical center is at (0, 0.505, 0.50) in robot base frame
- Camera is 50cm above platform
- Camera is 50.5cm forward from base center
- Camera is aligned with robot forward direction

Adjust these values in `configs/robot_config.yaml` if your setup differs.

## USB Permissions

Ensure your user has permission to access the USB device:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in for changes to take effect.

## Next Steps

1. Complete hand-eye calibration (see `CALIBRATION.md`)
2. Test object detection (see `TESTING.md`)
3. Run pick and place operations (see `COMMANDS.md`)

## Troubleshooting

### Camera Not Found

- Check camera is connected: `lsusb`
- Try different camera index in scripts (default is 0)
- Check permissions: `ls -l /dev/video*`

### Arm Not Responding

- Check USB connection: `lsusb | grep -i dynamixel`
- Check device: `ls -l /dev/ttyUSB*`
- Verify permissions (see USB Permissions above)
- Try different baudrate (default is 1000000)

### Import Errors

- Ensure virtual environment is activated
- Reinstall requirements: `pip install -r requirements.txt`
- Check Python version: `python --version` (should be 3.8+)

### YOLO Model Download

On first run, YOLO will download the model automatically. This requires internet connection. The model file is cached for future use.

