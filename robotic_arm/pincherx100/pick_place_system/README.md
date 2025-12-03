# PincherX100 Pick and Place System

Automated pick and place system for PincherX100 robotic arm with top-down camera vision.

## Overview

This system enables automated pick and place operations using:
- YOLO-based object detection for bottles
- Checkerboard pattern detection for drop locations
- Hand-eye calibration for coordinate transformation
- Inverse kinematics for arm control
- Command-line interface for operations
- Web interface for monitoring

## Features

- **Object Detection**: Detects orange juice bottles and yogurt bottles using YOLO
- **Drop Location Detection**: Uses checkerboard pattern to identify drop zones
- **Coordinate Transformation**: Converts camera pixel coordinates to robot base coordinates
- **Inverse Kinematics**: Calculates joint angles from end-effector positions
- **Pick and Place Operations**: Automated pick up and drop operations
- **Web Interface**: Real-time camera feed with detection overlays and telemetry

## System Architecture

```
pick_place_system/
├── scripts/          # Command-line interface and arm control
├── vision/           # Object detection, coordinate transformation, IK
├── calibration/      # Hand-eye calibration tools
├── configs/          # Configuration files
├── ui/               # Web interface
└── docs/             # Documentation
```

## Quick Start

### 1. Setup

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_place_system
./setup.sh
source venv/bin/activate
```

### 2. Calibration

Complete hand-eye calibration (see `docs/CALIBRATION.md`):

```bash
python calibration/hand_eye_calibration.py
```

### 3. Run Operations

Pick up object:
```bash
python scripts/arm_cli.py pick orange
python scripts/arm_cli.py pick yogurt
```

Drop object:
```bash
python scripts/arm_cli.py drop
```

### 4. Web Interface

Start web interface for monitoring:
```bash
python ui/web_interface.py
```

Access at: `http://<raspberry_pi_ip>:5000`

## Documentation

- **Setup Guide**: `docs/SETUP.md` - Installation and configuration
- **Calibration Guide**: `docs/CALIBRATION.md` - Hand-eye calibration procedure
- **Command Reference**: `docs/COMMANDS.md` - All available commands
- **Testing Guide**: `docs/TESTING.md` - Testing procedures

## Requirements

- Python 3.8+
- OpenCV 4.8+
- YOLOv8 (Ultralytics)
- Dynamixel SDK
- Flask
- Raspberry Pi 5 with 8GB RAM
- PincherX100 robotic arm
- 720p webcam

## Configuration

Edit `configs/robot_config.yaml` to adjust:
- Robot dimensions
- Object dimensions
- Pick and place parameters
- Camera position
- Servo settings

## Object Specifications

### Orange Juice Bottle
- Square cross-section: 5.3cm x 5.3cm
- Height: 18.5cm
- Cap: Green, radius 2.5cm

### Yogurt Bottle
- Round shape: radius 2.6cm
- Height: 15.5cm
- Cap: Pink, radius 2.0cm

## Drop Location

Uses 6x7 checkerboard pattern (12mm squares) printed on US Letter paper.
Pattern file: `../camera_calibration/checkerboard_6x7_tiny_US_letter.svg`

## Coordinate Systems

- **Camera Frame**: Pixel coordinates (u, v) with origin at top-left
- **Robot Base Frame**: (X, Y, Z) with origin at base center
  - X: Right
  - Y: Forward
  - Z: Up

## Safety

- Workspace limited to 180 degrees base rotation
- Joint limits enforced
- Position validation before movement
- Error detection and reporting

## Troubleshooting

See `docs/TESTING.md` for troubleshooting procedures.

Common issues:
- Camera not detected: Check USB connection and permissions
- Arm not responding: Verify USB connection and baudrate
- Objects not detected: Check lighting and YOLO model
- Poor accuracy: Recalibrate hand-eye transformation

## Development

### Project Structure

- `scripts/`: High-level control and CLI
- `vision/`: Computer vision and coordinate transformation
- `calibration/`: Calibration tools
- `configs/`: Configuration files
- `ui/`: Web interface

### Adding New Objects

1. Update object dimensions in `configs/robot_config.yaml`
2. Train or configure YOLO model for new object class
3. Update color filtering in `vision/object_detector.py`
4. Test detection accuracy

### Extending Functionality

- Add new commands in `scripts/arm_cli.py`
- Extend IK solver in `vision/inverse_kinematics.py`
- Add new detection methods in `vision/`
- Enhance web interface in `ui/`

## References

- PincherX100 IK: https://github.com/cychitivav/px100_ikine
- PincherX100 Description: https://github.com/cychitivav/px100_description
- YOLOv8: https://github.com/ultralytics/ultralytics

## License

See parent project license.

## Support

For issues and questions, refer to documentation in `docs/` directory.

