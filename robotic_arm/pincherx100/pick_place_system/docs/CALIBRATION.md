# Hand-Eye Calibration Guide

Complete guide for performing hand-eye calibration to establish the relationship between camera coordinates and robot base coordinates.

## Overview

Hand-eye calibration determines the transformation matrix between the camera coordinate system and the robot base coordinate system. This is essential for accurately converting pixel coordinates to robot positions.

## Prerequisites

1. Camera calibration completed (see `../camera_calibration/`)
2. Checkerboard pattern printed (6x7 tiny US letter)
3. Checkerboard mounted on rigid surface (foam board recommended)
4. Arm controller accessible

## Checkerboard Setup

### Print Checkerboard

1. Open `../camera_calibration/checkerboard_6x7_tiny_US_letter.svg`
2. Print at 100% scale (actual size)
3. Verify square size is 12mm using a ruler
4. Mount on rigid surface (foam board, cardboard, or thin plywood)
5. Ensure checkerboard is perfectly flat

### Place Checkerboard

- Place checkerboard flat on the platform (same height as pick area)
- Position in front of the robot, visible to the camera
- Ensure good lighting and contrast

## Calibration Procedure

### Step 1: Start Calibration Tool

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_place_system
source venv/bin/activate
python calibration/hand_eye_calibration.py
```

### Step 2: Prepare Arm Control

In a separate terminal, start the low-level arm controller:

```bash
cd ../low-level_control
source venv/bin/activate
python control_arm.py
```

Keep this running to manually control the arm during calibration.

### Step 3: Capture Observations

For each observation:

1. **Move arm gripper to touch a corner of the checkerboard**
   - Use keyboard controls in `control_arm.py`:
     - A/D: Base rotation
     - W/S: Shoulder
     - E/C: Elbow
     - R/V: Wrist
   - Position gripper so it touches a specific corner (e.g., top-left corner)

2. **Check camera view**
   - In calibration window, verify checkerboard is detected
   - Status should show "Pattern: FOUND"

3. **Capture observation**
   - Press SPACE in calibration window
   - Enter joint positions when prompted:
     - Format: `base,shoulder,elbow,wrist,gripper`
     - Example: `2048,2048,2048,2048,2048`
   - Or read positions from arm controller (press 'P' to show positions)

4. **Move to different position**
   - Move arm to a different corner or position
   - Repeat steps 1-3

### Step 4: Collect Multiple Observations

Collect at least 5-10 observations:
- Use different corners of the checkerboard
- Vary base rotation
- Vary arm extension
- Ensure checkerboard is always visible to camera

### Step 5: Perform Calibration

1. After collecting observations, press 'C' in calibration window
2. Calibration will calculate transformation matrix
3. Result will be saved to `calibration/hand_eye_calibration.json`

### Step 6: Verify Calibration

The calibration result includes:
- Transformation matrix (4x4)
- Number of observations used
- Timestamp

The transformation matrix will be automatically loaded by the coordinate transformer in future operations.

## Calibration File Format

The calibration file (`hand_eye_calibration.json`) contains:

```json
{
  "calibration_date": "2025-01-XX...",
  "num_observations": 8,
  "pattern_size": [6, 7],
  "square_size_mm": 12.0,
  "transform_matrix": [[...], [...], [...], [...]],
  "observations": [...]
}
```

## Troubleshooting

### Checkerboard Not Detected

- Ensure good lighting
- Check checkerboard is flat and not warped
- Verify pattern size matches (6x7 inner corners)
- Try adjusting camera focus

### Calibration Fails

- Need at least 3 observations
- Ensure observations are from different positions
- Check that joint positions are accurate
- Verify camera calibration is correct

### Poor Accuracy

- Collect more observations (10+ recommended)
- Use more varied positions
- Ensure checkerboard is flat
- Verify square size is correct (12mm)

## Alternative: Manual Calibration

If automatic calibration is not working, you can manually measure the camera position:

1. Measure camera position relative to robot base:
   - X offset (left/right)
   - Y offset (forward/back)
   - Z offset (height)

2. Update `configs/robot_config.yaml` with measured values

3. The system will use approximate transformation based on these values

## Next Steps

After calibration:
1. Test coordinate transformation (see `TESTING.md`)
2. Verify object detection accuracy
3. Test pick and place operations

