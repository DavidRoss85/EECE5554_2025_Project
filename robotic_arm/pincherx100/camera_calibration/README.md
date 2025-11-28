# PincherX100 Camera Calibration

Complete camera calibration and streaming setup for the PincherX100 robotic arm with a UVC camera.

## Overview

This package provides:
- **Printable checkerboard pattern** for camera calibration (small 10.5cm x 9cm version)
- **Camera streaming** from Raspberry Pi to Mac
- **Camera calibration** tools using OpenCV (Zhang's method)
- **Hand-eye calibration** (coming soon) to relate camera coordinates to robot coordinates

## Quick Start

### 1. Print the Checkerboard

**For US Letter printers (8.5" × 11"):**

Open `checkerboard_6x7_tiny_US_letter.svg` and print it:
- **Print at 100% scale** (do not scale to fit page)
- Print on white paper or cardstock
- **CRITICAL:** After printing, **measure actual square size with ruler**
- Designed size: 12mm squares, but use YOUR measured size!
- Total size: **8.4cm x 9.6cm** (fits within 8.5cm x 10cm limit ✓)
- Mount on a **rigid flat surface** (foam board, cardboard, or wood)
- Ensure the pattern is perfectly flat

**Important:** The actual printed size doesn't matter as long as you measure it accurately!

**For US Letter vs A4:**
- US Letter (8.5"×11"): Use `checkerboard_6x7_tiny_US_letter.svg`
- A4 (210×297mm): Use `checkerboard_6x7_tiny.svg`

See `PRINTING_INSTRUCTIONS_US_LETTER.md` for detailed printing instructions.

*(Alternative patterns: `checkerboard_6x5_small.svg`, `checkerboard_8x6.svg`)*

### 2. Install Dependencies on Raspberry Pi

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Start Camera Stream Server (on Raspberry Pi)

```bash
# Make sure you're in the virtual environment
source venv/bin/activate

# Start the streaming server
python camera_stream_server.py --camera 0 --port 5000

# The server will display its IP address
# Example output:
#   Access camera stream from your Mac:
#      Browser:  http://192.168.1.100:5000
#      Direct:   http://192.168.1.100:5000/video_feed
```

### 4. View Stream on Mac

**Option A: Web Browser**
```bash
# Open browser and go to:
http://<raspberry_pi_ip>:5000
```

**Option B: OpenCV Client (better for calibration)**
```bash
# On your Mac, install dependencies first
pip install opencv-python numpy

# Run the client
python camera_stream_client.py --ip <raspberry_pi_ip> --port 5000

# Controls:
#   ESC or Q - Quit
#   S - Save snapshot
```

### 5. Calibrate Camera (on Raspberry Pi)

```bash
# Make sure camera is connected
source venv/bin/activate

# For 8.5cm x 10cm printer limitation:
# Use TINY checkerboard (6x7 inner corners)
# IMPORTANT: Use YOUR measured square size (e.g., if you measured 12mm):
python calibrate_camera.py --camera 0 --pattern 6x7 --square 12

# If your squares measured differently (e.g., 11mm), use that:
# python calibrate_camera.py --camera 0 --pattern 6x7 --square 11

# Follow the on-screen instructions:
# 1. Hold checkerboard in front of camera
# 2. Move it to different positions and orientations
# 3. When green corners appear, press SPACE to capture
# 4. Capture 15-20 images from various angles
# 5. Press 'C' to calculate calibration
```

**Tips for good calibration:**
- Capture images at different distances (close and far)
- Capture images at different angles (tilted in all directions)
- Cover all parts of the image (corners, edges, center)
- Make sure the pattern is detected clearly (green corners visible)
- Aim for reprojection error < 0.5 pixels (< 1.0 is acceptable)

### 6. Test Calibration

```bash
# View undistorted video to verify calibration
python test_calibration.py --camera 0 --calib camera_calibration.npz
```

## Files Description

| File | Description |
|------|-------------|
| `checkerboard_6x7_tiny.svg` | **FOR 8.5×10cm PRINTER:** Tiny (6×7 corners, 12mm, 8.4×9.6cm) |
| `checkerboard_6x5_small.svg` | Small checkerboard (6×5 corners, 15mm, 10.5×9cm) |
| `checkerboard_8x6.svg` | Large checkerboard (8×6 corners, 25mm, 22.5×17.5cm) |
| `PRINTER_LIMITATION_GUIDE.md` | **READ THIS if printer limited to 8.5×10cm** |
| `camera_stream_server.py` | HTTP/MJPEG streaming server for Raspberry Pi |
| `camera_stream_client.py` | OpenCV-based client viewer for Mac |
| `calibrate_camera.py` | Interactive camera calibration tool |
| `test_calibration.py` | Test and visualize calibration results |
| `requirements.txt` | Python dependencies |
| `camera_calibration.npz` | Saved calibration data (created after calibration) |
| `camera_calibration.json` | Human-readable calibration data (created after calibration) |
| `calibration_images/` | Directory with captured calibration images (created during calibration) |

## Calibration Data Format

After calibration, two files are created:

**camera_calibration.npz** (NumPy format for OpenCV):
- `camera_matrix` - 3x3 camera intrinsic matrix
- `dist_coeffs` - Distortion coefficients (k1, k2, p1, p2, k3)
- `image_size` - Image width and height
- `calibration_error` - Reprojection error in pixels

**camera_calibration.json** (Human-readable):
```json
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [[k1, k2, p1, p2, k3]],
  "image_size": [width, height],
  "calibration_error": 0.25,
  "pattern_size": [8, 6],
  "square_size": 25.0,
  "calibration_date": "2025-11-26T...",
  "num_images": 18
}
```

## Camera Stream Details

The streaming server provides:
- **HTTP/MJPEG stream** accessible from any browser
- **Real-time video** at 30 FPS (1280x720)
- **Web interface** with instructions and info
- **Direct stream URL** for integration with other tools

Stream endpoints:
- `http://<ip>:5000/` - Web interface
- `http://<ip>:5000/video_feed` - Direct MJPEG stream

## Next Steps: Hand-Eye Calibration

After camera calibration, you can perform hand-eye calibration to determine the transformation between:
- Camera coordinate system
- Robot base coordinate system

This allows you to:
- Move the robot to positions seen by the camera
- Pick and place objects detected by the camera
- Perform visual servoing

*Hand-eye calibration script coming soon!*

## Understanding Calibration Results

### Camera Matrix (Intrinsic Parameters)
```
[[fx,  0, cx],
 [ 0, fy, cy],
 [ 0,  0,  1]]
```
- `fx`, `fy`: Focal lengths in pixels
- `cx`, `cy`: Principal point (optical center)

### Distortion Coefficients
- `k1`, `k2`, `k3`: Radial distortion
- `p1`, `p2`: Tangential distortion

### Reprojection Error
- **< 0.5 pixels**: Excellent calibration
- **0.5 - 1.0 pixels**: Good calibration
- **> 1.0 pixels**: May need recalibration

## Troubleshooting

### Camera not detected
```bash
# List available cameras
ls /dev/video*

# Test camera directly
python -c "import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened())"
```

### Checkerboard not detected
- Ensure good lighting
- Make sure pattern is flat and not warped
- Try adjusting camera focus
- Move pattern closer or farther (small pattern may need to be closer)
- Verify pattern size matches (6x5 inner corners for small version)

### Stream not accessible from Mac
```bash
# On Raspberry Pi, check firewall
sudo ufw status
sudo ufw allow 5000/tcp

# Test locally first
curl http://localhost:5000

# Check if server is listening
netstat -tuln | grep 5000
```

### Poor calibration results (high error)
- Capture more images (20-25)
- Use more varied positions and angles
- Ensure checkerboard is perfectly flat
- Check lighting conditions
- Verify square size is exactly 15mm (for small version)

### About Zhang's Calibration Method
- The calibration uses **Zhang's method** (Zhengyou Zhang, 1999)
- This is OpenCV's standard `cv2.calibrateCamera()` function
- It's the most widely used camera calibration technique
- Calculates camera intrinsics from multiple checkerboard views

## References

- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Camera Calibration Theory](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [Hand-Eye Calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b)

## License

MIT License - See main project LICENSE file

---

**Need Help?**
- Check camera connections and permissions
- Verify Python dependencies are installed
- Review error messages for specific issues
- Test camera with simple OpenCV script first

