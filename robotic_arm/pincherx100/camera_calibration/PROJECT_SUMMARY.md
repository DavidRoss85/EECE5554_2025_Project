# PincherX100 Camera Calibration - Project Summary

##  What Was Created

A complete camera calibration and streaming system for the PincherX100 robotic arm with UVC camera.

##  Files Created

### 🖼 Checkerboard Patterns
- **`checkerboard_8x6.svg`** - Printable SVG pattern (8x6 inner corners, 25mm squares)
- **`generate_checkerboard_pdf.py`** - Script to generate high-quality PDF patterns

###  Camera Streaming
- **`camera_stream_server.py`** - HTTP/MJPEG streaming server for Raspberry Pi
- **`camera_stream_client.py`** - OpenCV-based viewer for Mac

###  Calibration Tools
- **`calibrate_camera.py`** - Interactive camera calibration tool
- **`test_calibration.py`** - Visualize calibration results (undistortion)
- **`hand_eye_calibration.py`** - Hand-eye calibration (camera-to-robot)

###  Setup Scripts
- **`setup.sh`** - One-time setup (creates venv, installs dependencies)
- **`quick_start.sh`** - Interactive menu for common tasks
- **`requirements.txt`** - Python dependencies

###  Documentation
- **`README.md`** - Complete reference and documentation
- **`GETTING_STARTED.md`** - Quick start guide (5 minutes)
- **`CALIBRATION_GUIDE.md`** - Detailed calibration walkthrough
- **`PROJECT_SUMMARY.md`** - This file

##  How to Use

### Step 1: Print Checkerboard (5 min)

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration

# Option A: Print the SVG directly
# Open checkerboard_8x6.svg in browser and print at 100% scale

# Option B: Generate PDF for better quality
./setup.sh  # First time only
source venv/bin/activate
python generate_checkerboard_pdf.py
# Print checkerboard.pdf at 100% scale
```

**Important:** Mount on rigid flat surface (foam board/cardboard) and verify each square is exactly 25mm!

### Step 2: View Camera Stream on Mac (Optional) (2 min)

**On Raspberry Pi:**
```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
source venv/bin/activate
python camera_stream_server.py

# Note the IP address shown, e.g., http://192.168.1.100:5000
```

**On Mac:**
```bash
# Option A: Open in web browser
open http://192.168.1.100:5000  # Replace with your Pi's IP

# Option B: Use OpenCV client (better for calibration)
pip install opencv-python numpy
python camera_stream_client.py --ip 192.168.1.100
# Press S to save snapshots, Q to quit
```

### Step 3: Calibrate Camera (10-15 min)

**On Raspberry Pi:**
```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
source venv/bin/activate
python calibrate_camera.py --camera 0 --pattern 8x6 --square 25
```

**Calibration Process:**
1. Hold checkerboard in front of camera
2. When corners are detected (green overlay), press **SPACE**
3. Move to different positions and angles
4. Capture **15-20 images** (more is better)
   - Different distances (close, medium, far)
   - Different angles (tilted in all directions)
   - Different positions (corners, edges, center)
5. Press **'C'** to calculate calibration
6. Check reprojection error (should be < 0.5 pixels)

**Output:**
- `camera_calibration.npz` - Calibration data (NumPy format)
- `camera_calibration.json` - Calibration data (human-readable)
- `calibration_images/` - Captured images

### Step 4: Test Calibration (2 min)

```bash
python test_calibration.py --camera 0 --calib camera_calibration.npz
```

This shows side-by-side comparison of original (distorted) vs undistorted video.

### Step 5: Hand-Eye Calibration (Optional) (30+ min)

```bash
python hand_eye_calibration.py --camera 0 --calib camera_calibration.npz
```

This will help calibrate the transformation between camera and robot base coordinates.

**Note:** Full hand-eye calibration requires forward kinematics implementation (coming soon).

##  Understanding the Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                    CALIBRATION WORKFLOW                       │
└─────────────────────────────────────────────────────────────┘

1. Print Checkerboard
   ↓
2. Mount on Rigid Surface
   ↓
3. Camera Streaming (optional)
   └─> View on Mac for easier setup
   ↓
4. Camera Calibration
   ├─> Capture 15-20 images
   ├─> Calculate intrinsic parameters
   └─> Save calibration data
   ↓
5. Test Calibration
   └─> Verify undistortion works
   ↓
6. Hand-Eye Calibration (optional)
   ├─> Relate camera to robot coordinates
   └─> Enable vision-guided manipulation
   ↓
7. Integration
   └─> Use in your robot vision pipeline
```

##  What Gets Calibrated

### Camera Intrinsic Parameters
```
Camera Matrix:          Distortion Coefficients:
[[fx,  0, cx],          [k1, k2, p1, p2, k3]
 [ 0, fy, cy],          
 [ 0,  0,  1]]          
```

- **fx, fy**: Focal lengths (how much camera "zooms")
- **cx, cy**: Principal point (optical center)
- **k1, k2, k3**: Radial distortion (barrel/pincushion)
- **p1, p2**: Tangential distortion

### Benefits
-  Remove lens distortion
-  Accurate 3D measurements
-  Enable visual servoing
-  Object detection with real-world coordinates

##  System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    RASPBERRY PI                          │
│                                                          │
│  ┌────────────┐   ┌──────────────────┐                │
│  │ UVC Camera │──▶│ Camera Stream    │                │
│  └────────────┘   │ Server (Flask)   │                │
│                   └────────┬─────────┘                │
│                            │ HTTP/MJPEG               │
│                            │ :5000                    │
│                   ┌────────▼─────────┐                │
│                   │ Calibration Tool │                │
│                   │ (OpenCV)         │                │
│                   └──────────────────┘                │
│                            │                          │
│                            ▼                          │
│                   camera_calibration.npz              │
└──────────────────────────────────────────────────────────┘
                             │
                             │ Network
                             ▼
┌──────────────────────────────────────────────────────────┐
│                         MAC                              │
│                                                          │
│  ┌─────────────┐    ┌──────────────────┐               │
│  │ Web Browser │ or │ Stream Client    │               │
│  │             │    │ (OpenCV)         │               │
│  └─────────────┘    └──────────────────┘               │
│                                                          │
│  View live camera feed for setup and calibration        │
└──────────────────────────────────────────────────────────┘
```

##  Quick Command Reference

```bash
# === SETUP ===
./setup.sh              # First-time setup
./quick_start.sh        # Interactive menu

# === CHECKERBOARD ===
python generate_checkerboard_pdf.py --size 8x6 --square 25

# === STREAMING ===
# Raspberry Pi:
python camera_stream_server.py --camera 0 --port 5000
# Mac:
python camera_stream_client.py --ip <pi_ip> --port 5000

# === CALIBRATION ===
python calibrate_camera.py --camera 0 --pattern 8x6 --square 25
python test_calibration.py --camera 0 --calib camera_calibration.npz
python hand_eye_calibration.py --camera 0 --calib camera_calibration.npz

# === UTILITIES ===
ls /dev/video*          # List cameras
v4l2-ctl --list-devices # Detailed camera info
```

##  Next Steps for Development

### Immediate Use
1.  Camera calibration (complete)
2.  Stream viewing (complete)
3.  Undistortion testing (complete)

### Future Enhancements
-  Complete hand-eye calibration with forward kinematics
-  Object detection and tracking
-  Visual servoing integration
-  Pick-and-place with vision guidance
-  ROS2 integration for camera node

##  Pro Tips

### For Best Calibration Results:
1. **Checkerboard Quality**
   - Print at exactly 100% scale
   - Verify with ruler (25mm squares)
   - Mount on rigid flat surface
   - Ensure no warping or bending

2. **Capture Strategy**
   - Take 20-25 images (more is better)
   - Cover entire image area
   - Vary distance (close to far)
   - Vary angle (all orientations)
   - Hold steady (avoid blur)

3. **Environment**
   - Use bright, even lighting
   - Avoid shadows on pattern
   - No glare or reflections
   - Clean camera lens

4. **Quality Check**
   - Reprojection error < 0.5 pixels = excellent
   - Error 0.5-1.0 pixels = good
   - Error > 1.0 pixels = recalibrate

##  Troubleshooting

### Camera not detected
```bash
ls /dev/video*
python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### Pattern not detected
- Improve lighting
- Move checkerboard closer/farther
- Ensure pattern is flat and clean
- Check camera focus

### Stream not accessible from Mac
```bash
# On Pi:
hostname -I                 # Get IP
sudo ufw allow 5000/tcp     # Open firewall
netstat -tuln | grep 5000   # Check server running
```

### High calibration error
- Recapture with more varied images
- Ensure checkerboard is perfectly flat
- Check for motion blur in images
- Use better lighting

##  Learning Resources

- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Understanding Camera Calibration](https://learnopencv.com/camera-calibration-using-opencv/)
- [Hand-Eye Calibration Guide](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

##  Success Criteria

You'll know calibration is successful when:
-  Reprojection error < 0.5 pixels
-  Straight lines appear straight in undistorted image
-  Lens distortion is visibly corrected
-  Calibration file saved successfully

##  What You Can Do Now

With calibrated camera:
1. **Undistort images** for accurate measurements
2. **Detect objects** with correct dimensions
3. **3D reconstruction** from 2D images
4. **Visual servoing** (after hand-eye calibration)
5. **Pick and place** with vision guidance
6. **Augmented reality** overlays

##  Support

For issues:
1. Check GETTING_STARTED.md for quick fixes
2. Review CALIBRATION_GUIDE.md for detailed steps
3. Verify hardware connections and settings
4. Check terminal output for error messages

##  License

MIT License - See main project LICENSE

---

**Ready to calibrate?** Start with `./quick_start.sh`! 

**Questions?** Check the documentation files in this directory.

**Enjoy your calibrated camera!** 

