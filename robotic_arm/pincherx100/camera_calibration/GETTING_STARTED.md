# Getting Started with Camera Calibration

**Quick guide to get you up and running in 5 minutes!**

## Quick Start (3 Steps)

### 1. Print the Checkerboard

**Option A: SVG (easiest) - US LETTER VERSION**
```bash
# For US Letter (8.5" × 11") printers:
firefox checkerboard_6x7_tiny_US_letter.svg
# or
chromium-browser checkerboard_6x7_tiny_US_letter.svg
```

**Option B: Generate PDF (best quality)**
```bash
./setup.sh  # First time only
source venv/bin/activate
python generate_checkerboard_pdf.py --size 6x7 --square 12 --output checkerboard.pdf
# Then print checkerboard.pdf at 100% scale on US Letter
```

**Important:**
- Print at **100% scale** on **US Letter (8.5" × 11")** paper
- NO "Fit to Page" or scaling!
- After printing: **MEASURE** one square with ruler
- Use YOUR measured size (e.g., 12mm, 11mm, 13mm)
- Mount on **rigid flat surface** (foam board or cardboard)

**See:** `PRINTING_INSTRUCTIONS_US_LETTER.md` for detailed printing guide

### 2. Setup and Run

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration

# Interactive menu (easiest!)
./quick_start.sh

# Or manually:
./setup.sh                    # First time only
source venv/bin/activate
python camera_stream_server.py   # View camera on Mac
# In new terminal:
python calibrate_camera.py       # Run calibration
```

### 3. Calibrate

1. Hold checkerboard in front of camera
2. When green corners appear, press **SPACE**
3. Move to different positions/angles
4. Capture **15-20 images**
5. Press **'C'** to calibrate
6. Done!

## What You'll Need

### Hardware
- Raspberry Pi with UVC camera connected
- Printed checkerboard pattern on rigid surface
- Good lighting (avoid shadows)

### Software (auto-installed by setup.sh)
- Python 3.7+
- OpenCV
- NumPy
- Flask (for streaming)

## Common Tasks

### View Camera on Mac

**On Raspberry Pi:**
```bash
source venv/bin/activate
python camera_stream_server.py
# Note the IP address shown
```

**On Mac:**
```bash
# In browser:
http://192.168.1.100:5000  # Replace with your Pi's IP

# Or use client:
pip install opencv-python numpy
python camera_stream_client.py --ip 192.168.1.100
```

### Run Calibration

```bash
source venv/bin/activate
python calibrate_camera.py --camera 0 --pattern 6x5 --square 15

# Controls:
#   SPACE - Capture image (when pattern is detected)
#   C - Calculate calibration
#   Q - Quit
```

### Test Calibration

```bash
python test_calibration.py --camera 0 --calib camera_calibration.npz

# This shows original vs undistorted side-by-side
# Press Q to quit
```

## Understanding Results

Good calibration:
```
Reprojection Error: 0.35 pixels
Camera Matrix:
[[1000,  0, 640],
 [   0, 1000, 360],
 [   0,    0,   1]]
```

Quality guide:
- **< 0.5 pixels**: Excellent
- **0.5-1.0 pixels**: Good
- **> 1.0 pixels**: Recalibrate

## Troubleshooting

### Camera not found
```bash
ls /dev/video*  # Check camera exists
python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### Pattern not detected
- Improve lighting
- Move closer/farther
- Ensure pattern is flat
- Clean camera lens

### Can't access stream from Mac
```bash
# On Pi:
hostname -I  # Get Pi's IP
sudo ufw allow 5000/tcp  # Open firewall
ping <mac_ip>  # Test connectivity
```

## Documentation

- **README.md** - Complete overview and reference
- **CALIBRATION_GUIDE.md** - Detailed step-by-step guide
- **GETTING_STARTED.md** - This file (quick start)

## Next Steps

After camera calibration:

1. **Test the calibration**
   ```bash
   python test_calibration.py
   ```

2. **Hand-eye calibration** (relate camera to robot)
   ```bash
   python hand_eye_calibration.py
   ```

3. **Integrate with robot control**
   - Use calibration data in your robot vision pipeline
   - Detect objects and convert to robot coordinates
   - Implement pick-and-place with vision

## Tips for Best Results

### Checkerboard
- Must be perfectly flat
- High contrast (clean black/white)
- Proper size verification (15mm squares, 10.5cm x 9cm total)
- Small size makes it easy to handle and position

### Lighting
- Bright, even lighting
- No shadows on pattern
- Avoid glare/reflections

### Calibration Images
- Capture 15-20 images minimum
- Vary distance (close, medium, far)
- Vary angle (tilt in all directions)
- Cover all parts of image (corners, edges, center)
- Hold steady (avoid motion blur)

### During Capture
- Wait for full detection (all corners green)
- Keep pattern in focus
- Don't rush - quality over quantity
- Review captured images folder if unsure

## Need Help?

1. Check **CALIBRATION_GUIDE.md** for detailed instructions
2. Review **README.md** troubleshooting section
3. Verify hardware connections
4. Check camera with: `python -c "import cv2; cv2.VideoCapture(0).read()"`

## Quick Commands Reference

```bash
# Setup (first time)
./setup.sh

# Interactive menu
./quick_start.sh

# Stream camera
python camera_stream_server.py

# Calibrate
python calibrate_camera.py

# Test calibration
python test_calibration.py

# Generate PDF checkerboard (small version, 6x5 corners, 15mm squares)
python generate_checkerboard_pdf.py --size 6x5 --square 15

# Hand-eye calibration
python hand_eye_calibration.py
```

---

**Ready to start?** Run `./quick_start.sh` and follow the prompts!

