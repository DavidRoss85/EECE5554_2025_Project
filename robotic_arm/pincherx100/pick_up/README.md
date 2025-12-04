# Three Bottle Pick and Place System

AprilTag-based pick and place system for PincherX-100 robotic arm.

## Overview

- **Bottles**: Orange, Apple, Yogurt
- **AprilTag Size**: 1.0 inch (25.4mm)
- **Detection**: AprilTag-based (36h11 family)
- **Accuracy**: ±1-5mm with calibration

## Quick Start

```bash
# Setup
cd /home/jx/Dev/Robotics/pincherx100/pick_up
./setup.sh
source venv/bin/activate

# Generate AprilTags
python scripts/generate_apriltag_pdf.py --all --size 1.0

# Test detection
python scripts/pick_place.py detect

# Pick and place
python scripts/pick_place.py pick orange
python scripts/pick_place.py place orange
```

## Tag Assignment

| Item | Tag ID | Purpose |
|------|--------|---------|
| Orange Bottle | 0 | Object |
| Apple Bottle | 1 | Object |
| Yogurt Bottle | 2 | Object |
| Orange Zone | 10 | Drop location |
| Apple Zone | 11 | Drop location |
| Yogurt Zone | 12 | Drop location |

## Installation

### Requirements
- Python 3.8+
- PincherX-100 arm
- USB camera
- Printed AprilTags (1.0 inch)

### Dependencies
```bash
pip install -r requirements.txt
```

Required packages: opencv-python, numpy, dt-apriltags, dynamixel-sdk, pyserial, PyYAML, reportlab

## Configuration

Main configuration: `configs/robot_config.yaml`

### Key Settings

**AprilTag Size**
```yaml
apriltags:
  tag_size: 0.0254  # 1.0 inch
```

**Arm Positions**
```yaml
positions:
  home:
    values: [2048, 2048, 2048, 2048]
  retract:
    values: [2048, 1353, 3073, 937]
```

**Bottle Dimensions**
```yaml
objects:
  orange_bottle:
    height: 0.185  # meters
    grip_height_ratio: 0.5
```

## Usage

See `manual.md` for complete command reference.

**Basic workflow:**
```bash
python scripts/pick_place.py detect        # Check what's visible
python scripts/pick_place.py pos home      # Move to center
python scripts/pick_place.py pick orange   # Pick bottle
python scripts/pick_place.py place orange  # Place at zone
python scripts/pick_place.py pos retract   # Retract arm
```

**Debug mode** (troubleshoot pick operations):
```bash
python scripts/pick_place.py pick orange --debug 1  # Stop after detection
python scripts/pick_place.py pick orange --debug 2  # Stop after approach
```

## Project Structure

```
pick_up/
├── configs/
│   └── robot_config.yaml       # Main configuration
├── calibration/
│   └── camera_calibration.npz  # Camera calibration
├── scripts/
│   ├── pick_place.py           # Main script
│   ├── arm_controller_wrapper.py
│   └── generate_apriltag_pdf.py
├── vision/
│   ├── apriltag_detector.py
│   ├── coordinate_transform.py
│   └── inverse_kinematics.py
├── tools/
│   └── camera_optical_center.py  # Camera position calibration tool
├── README.md                   # This file
└── manual.md                   # Command reference
```

## AprilTag Setup

1. **Generate**: `python scripts/generate_apriltag_pdf.py --all --size 1.0`
2. **Print**: At 100% scale (no scaling)
3. **Verify**: Measure with ruler (should be exactly 25.4mm)
4. **Attach**: Mount tags on bottle tops
5. **Place**: Position drop zone tags on platform

## Camera Calibration

Copy calibration file to `calibration/camera_calibration.npz`. If not available, system uses default parameters (less accurate).

## Documentation

- `README.md` - This file (overview and setup)
- `manual.md` - Command reference and configuration guide

## License

This project uses standard open-source robotics libraries.
