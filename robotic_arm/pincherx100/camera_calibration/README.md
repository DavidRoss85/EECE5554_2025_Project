# Camera Calibration

Camera calibration measures the camera's intrinsic parameters (focal length, optical center, lens distortion). This is needed to convert pixel coordinates to real-world 3D coordinates.

## What We Have

- **`camera_calibration.npz`**: Camera calibration file (already done)
- **`camera_calibration.json`**: Same data in JSON format

## What Camera Calibration Does

Measures:
1. **Camera matrix** (focal length fx, fy and optical center cx, cy)
2. **Distortion coefficients** (lens distortion parameters)
3. **Image size** (1280×720 pixels)

This allows us to:
- Convert pixel (u,v) → normalized coordinates
- Account for lens distortion
- Calculate 3D position from 2D image

## Files

### Calibration Data
- `camera_calibration.npz` - NumPy format (used by Python)
- `camera_calibration.json` - JSON format (human-readable)

### Calibration Scripts
- `calibrate_camera.py` - Calibrate from checkerboard images
- `calibrate_camera_web.py` - Calibrate using web camera
- `test_calibration.py` - Test calibration quality

### Checkerboard Patterns
- `checkerboard_6x7_large_US_letter.svg` - Large checkerboard (24mm squares)
- `checkerboard_6x7_tiny_US_letter.svg` - Small checkerboard

### Other
- `calibration_images/` - Images used for calibration
- `requirements.txt` - Python dependencies
- `venv/` - Python virtual environment

## Re-calibrating (if needed)

### 1. Print Checkerboard

Print `checkerboard_6x7_large_US_letter.svg` at 100% scale on US Letter paper.
Verify square size is 24mm × 24mm.

### 2. Capture Images

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
source venv/bin/activate
python calibrate_camera_web.py
```

- Hold checkerboard at different angles and positions
- Press SPACE to capture image (need 15-20 images)
- Press Q when done
- Calibration will run automatically

### 3. Test Calibration

```bash
python test_calibration.py
```

Shows reprojection error (should be < 1 pixel for good calibration).

## Why Not Hand-Eye Calibration?

Hand-eye calibration finds the transformation between camera and robot. We **don't use it** because:

1. **Simple geometry is sufficient**: We know camera position (0, 0.21, 0.51)
2. **Faster**: No calibration procedure needed
3. **More reliable**: No accumulation of calibration errors
4. **Easier to understand**: Direct geometric transformation

We only need camera intrinsic calibration (what we have) for accurate pixel→3D conversion.

## Technical Details

Camera calibration uses Zhang's method with a planar checkerboard pattern:
1. Capture multiple images of checkerboard at different poses
2. Detect checkerboard corners in each image
3. Solve for camera matrix and distortion coefficients
4. Minimize reprojection error

The calibration is independent of the robot - it only measures the camera properties.
