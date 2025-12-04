# Calibration and Testing Tools

Utility tools for calibrating and testing the pick and place system.

## Camera Optical Center Visualizer

**File:** `camera_optical_center.py`

**Purpose:** Visualize the camera's optical center on live video feed to help measure the camera position relative to the robot base.

### Usage

```bash
cd /home/jx/Dev/Robotics/pincherx100/pick_up
source venv/bin/activate
python tools/camera_optical_center.py
```

### What It Does

1. Opens your camera and shows live video feed
2. Draws a **RED CROSSHAIR** at the camera's optical center (cx, cy)
3. This crosshair shows where the camera is "looking" at directly

### How to Use for Calibration

1. **Run the tool:**
   ```bash
   python tools/camera_optical_center.py
   ```

2. **Place a marker** (coin, tape, etc.) on the platform directly under the red crosshair

3. **Measure distances** from robot base center to the marker:
   - **X distance**: Left/right from base center (in meters)
   - **Y distance**: Forward from base center (in meters)
   - **Z distance**: Camera height above platform (in meters)

4. **Update configuration** in `configs/robot_config.yaml`:
   ```yaml
   camera:
     position:
       x: <measured_x>  # e.g., 0.0 if centered
       y: <measured_y>  # e.g., 0.16 for 16cm forward
       z: <measured_z>  # e.g., 0.70 for 70cm height
   ```

### Controls

- **Q** - Quit the tool
- **S** - Save screenshot with marker visible
- **H** - Toggle help overlay on/off

### Options

```bash
# Use different camera
python tools/camera_optical_center.py --camera 1

# Use specific calibration file
python tools/camera_optical_center.py --calibration /path/to/calibration.npz
```

### Example Output

```
==============================================================
CAMERA CALIBRATION LOADED
==============================================================
Optical Center (cx, cy): (640.0, 360.0) pixels
Focal Length (fx, fy): (800.5, 800.3) pixels
==============================================================

Camera Resolution: 1280x720
Optical Center will be marked at: (640.0, 360.0)
```

The tool will display a live feed with a large red crosshair at the optical center.

## Future Tools

More calibration and testing tools can be added to this directory:
- Hand-eye calibration tool
- Workspace visualization
- AprilTag detection tester
- Coordinate transformation checker

