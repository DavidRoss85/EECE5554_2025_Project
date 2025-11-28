# Camera Calibration Guide for PincherX100

Complete step-by-step guide for calibrating your UVC camera with the PincherX100 robotic arm.

##  What is Camera Calibration?

Camera calibration determines the camera's **intrinsic parameters**:
- **Focal length** (fx, fy): How much the camera "zooms"
- **Principal point** (cx, cy): Where the optical axis hits the image plane
- **Distortion coefficients**: How the lens distorts the image (barrel/pincushion)

This allows you to:
-  Undistort images (remove lens distortion)
-  Measure real-world distances
-  Perform 3D reconstruction
-  Enable accurate robot-camera coordination

##  Step-by-Step Calibration Process

### Step 1: Print the Checkerboard Pattern

**Use the SMALL version** (fits under 10cm x 10cm):

1. Open `checkerboard_6x5_small.svg` in a web browser or image viewer
2. Print it **at 100% scale** (very important!)
   - Do NOT use "Fit to Page" or "Scale to Fit"
   - Use actual size / 100% scale setting
3. Use white paper or cardstock (heavier is better)
4. After printing, **verify** with a ruler:
   - Each square should be exactly **15mm (1.5cm)**
   - Total pattern size: **10.5cm x 9cm**
   - If not exact, adjust printer settings and reprint

### Step 2: Mount the Checkerboard

The checkerboard MUST be perfectly flat for accurate calibration.

**Good mounting options:**
-  Foam board (best option - rigid and lightweight)
-  Cardboard (use thick cardboard)
-  Thin plywood or MDF board
-  Rigid plastic sheet

**How to mount:**
1. Use spray adhesive or glue stick (apply evenly)
2. Carefully align the printed pattern
3. Smooth out any bubbles or wrinkles
4. Let dry completely
5. Check that it's perfectly flat (no warping)

**Bad mounting (don't do this):**
-  Just holding the paper (will bend and warp)
-  Taping to a surface (creates wrinkles)
-  Using thin/flexible materials

### Step 3: Set Up Your Raspberry Pi

```bash
# Connect to Raspberry Pi via SSH or directly
ssh pi@<raspberry_pi_ip>

# Navigate to camera calibration directory
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration

# Run setup (first time only)
./setup.sh

# Activate virtual environment
source venv/bin/activate
```

### Step 4: Start Camera Stream (Optional but Recommended)

This lets you view the camera on your Mac while setting up:

```bash
# On Raspberry Pi
python camera_stream_server.py --camera 0 --port 5000

# The server will display its IP address
# Example: http://192.168.1.100:5000
```

Then on your Mac:
```bash
# Option A: Open in browser
# Go to http://<raspberry_pi_ip>:5000

# Option B: Use OpenCV client (better)
pip install opencv-python numpy
python camera_stream_client.py --ip <raspberry_pi_ip> --port 5000
```

### Step 5: Run Camera Calibration

```bash
# On Raspberry Pi
# Use small pattern: 6x5 inner corners, 15mm squares
python calibrate_camera.py --camera 0 --pattern 6x5 --square 15
```

The calibration window will open showing the live camera feed.

### Step 6: Capture Calibration Images

**Positioning Tips:**

1. **Different Distances**
   ```
   Close (20-30cm)  →  Medium (40-60cm)  →  Far (70-100cm)
   ```
   Capture 5-7 images at each distance range.

2. **Different Angles**
   ```
   Tilted Left  ←→  Straight  ←→  Tilted Right
   Tilted Up    ↕   Straight  ↕   Tilted Down
   Rotated CCW  ↺   Straight  ↻   Rotated CW
   ```
   Cover all combinations for best results.

3. **Different Positions**
   ```
   ┌─────────────────┐
   │ TL   TC    TR   │  TL = Top Left
   │                 │  TC = Top Center
   │ ML   C     MR   │  C  = Center
   │                 │  BR = Bottom Right
   │ BL   BC    BR   │  etc.
   └─────────────────┘
   ```
   Make sure the checkerboard covers all parts of the image.

**Capture Process:**

1. Hold the checkerboard in front of the camera
2. Wait for the pattern to be detected (green lines appear)
3. Hold steady and press **SPACE** to capture
4. Move to a new position/angle
5. Repeat until you have 15-20 good images

**Quality Indicators:**
-  All corners detected (complete green overlay)
-  Image is sharp (not blurry)
-  Good lighting (no shadows on pattern)
-  Checkerboard fills 30-70% of frame

**Common Issues:**
-  Pattern not detected → Improve lighting or move closer
-  Partial detection → Tilt board or change angle
-  Blurry images → Hold steadier or improve lighting

### Step 7: Calculate Calibration

After capturing 15-20 images:

1. Press **'C'** to start calibration
2. Wait for calculation (takes a few seconds)
3. Check the results:
   ```
   Reprojection Error: 0.35 pixels  ← Lower is better
   
   Good:       < 0.5 pixels  
   Acceptable: 0.5 - 1.0 pixels  
   Poor:       > 1.0 pixels   (recalibrate)
   ```

If error is too high:
- Delete `calibration_images/` folder
- Recalibrate with more varied images
- Ensure checkerboard is perfectly flat
- Check lighting conditions

### Step 8: Save and Verify

Calibration is automatically saved to:
- `camera_calibration.npz` - For use with OpenCV
- `camera_calibration.json` - Human-readable format

**Test the calibration:**
```bash
python test_calibration.py --camera 0 --calib camera_calibration.npz
```

This shows side-by-side comparison of original (distorted) and corrected (undistorted) images.

##  Understanding Calibration Results

### Camera Matrix
```
[[fx,  0, cx],
 [ 0, fy, cy],
 [ 0,  0,  1]]
```

Example:
```
[[1000,  0, 640],
 [   0, 1000, 360],
 [   0,    0,   1]]
```

- **fx, fy** ≈ 1000 for typical webcam at 1280x720
- **cx** ≈ width/2 (image center x)
- **cy** ≈ height/2 (image center y)

### Distortion Coefficients
```
[k1, k2, p1, p2, k3]
```

Example:
```
[-0.35, 0.15, 0.001, 0.002, -0.05]
```

- **k1, k2, k3**: Radial distortion (barrel/pincushion effect)
- **p1, p2**: Tangential distortion (lens not parallel to sensor)

Typical values:
- **Wide-angle lens**: k1 < -0.3 (barrel distortion)
- **Normal lens**: -0.3 < k1 < 0 (slight barrel)
- **Telephoto lens**: k1 > 0 (pincushion distortion)

##  Next Steps: Hand-Eye Calibration

After camera calibration, you can perform **hand-eye calibration** to relate:
- Camera coordinates → Robot base coordinates

This enables:
- Pick and place objects detected by camera
- Visual servoing
- Automatic object tracking

```bash
# Run hand-eye calibration (simplified version)
python hand_eye_calibration.py --camera 0 --calib camera_calibration.npz
```

*Note: Full hand-eye calibration requires robot forward kinematics implementation.*

##  Camera Streaming

### On Raspberry Pi:
```bash
# Start server
python camera_stream_server.py --camera 0 --port 5000
```

### On Mac:
```bash
# Option 1: Web Browser
open http://<raspberry_pi_ip>:5000

# Option 2: OpenCV Client
python camera_stream_client.py --ip <raspberry_pi_ip> --port 5000

# Press 'S' to save snapshots
# Press 'Q' or ESC to quit
```

##  Troubleshooting

### Camera not detected

```bash
# Check available cameras
ls /dev/video*

# Test camera with OpenCV
python -c "import cv2; cap=cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL')"

# Check permissions
groups  # Should include 'video' group
sudo usermod -a -G video $USER  # Add if missing
```

### Pattern not detected

**Lighting:**
- Use bright, even lighting
- Avoid shadows on the pattern
- No glare or reflections

**Distance:**
- Too close: Pattern doesn't fit in frame
- Too far: Corners too small to detect
- Sweet spot: Pattern fills 30-70% of frame

**Focus:**
- Auto-focus cameras: Wait for focus
- Manual focus: Pre-focus before capturing
- Image should be sharp and clear

### High calibration error

**Common causes:**
1. **Warped checkerboard** → Mount on rigid surface
2. **Too few images** → Capture 20-25 images
3. **Limited variety** → Use more angles/positions
4. **Moving during capture** → Hold steady when pressing SPACE
5. **Blurry images** → Ensure sharp focus

**Solutions:**
```bash
# Delete old images and recalibrate
rm -rf calibration_images/
python calibrate_camera.py
```

### Stream not accessible from Mac

```bash
# On Raspberry Pi - check firewall
sudo ufw status
sudo ufw allow 5000/tcp

# Test locally first
curl http://localhost:5000

# Check server is running
ps aux | grep camera_stream_server
netstat -tuln | grep 5000

# Find Raspberry Pi IP
hostname -I
```

##  Theory and Background

### Why Calibrate?

Real cameras have imperfections:
- **Lens distortion**: Straight lines appear curved
- **Manufacturing tolerances**: Lens not perfectly centered
- **Sensor alignment**: Sensor not perfectly perpendicular

Calibration measures these imperfections so we can:
1. **Correct distortion**: Make straight lines straight
2. **3D reconstruction**: Convert 2D image → 3D world coordinates
3. **Accurate measurements**: Measure real distances in images

### Pinhole Camera Model

Ideal camera (no distortion):
```
[u]   [fx  0  cx]   [X]
[v] = [ 0 fy  cy] * [Y]
[1]   [ 0  0   1]   [Z]
```

With distortion correction, we can use this model for real cameras.

### Checkerboard Pattern

Why checkerboards work well:
-  High contrast (easy detection)
-  Clear corners (precise localization)
-  Known geometry (known 3D coordinates)
-  Oriented pattern (can detect orientation)

##  Additional Resources

- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Zhang's Calibration Method (original paper)](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/tr98-71.pdf)
- [Camera Calibration Explained](https://learnopencv.com/camera-calibration-using-opencv/)

##  Quick Reference

### Common Commands

```bash
# Setup (first time)
./setup.sh

# Quick start menu
./quick_start.sh

# Stream camera
python camera_stream_server.py

# Calibrate camera
python calibrate_camera.py

# Test calibration
python test_calibration.py

# Hand-eye calibration
python hand_eye_calibration.py
```

### File Descriptions

| File | Purpose |
|------|---------|
| `checkerboard_8x6.svg` | Printable calibration pattern |
| `camera_calibration.npz` | Saved calibration (NumPy) |
| `camera_calibration.json` | Saved calibration (JSON) |
| `calibration_images/` | Captured calibration images |
| `hand_eye_observations.json` | Hand-eye calibration data |

### Calibration Quality Guidelines

| Error (pixels) | Quality | Action |
|---------------|---------|--------|
| < 0.3 | Excellent | Perfect! |
| 0.3 - 0.5 | Good | Use as-is |
| 0.5 - 1.0 | Acceptable | Consider recalibrating |
| > 1.0 | Poor | Must recalibrate |

---

**Need help?** Check the README.md or create an issue on GitHub.

