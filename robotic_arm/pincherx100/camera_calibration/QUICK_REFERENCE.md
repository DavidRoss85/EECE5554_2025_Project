# Quick Reference - 8.5cm x 10cm Printer

##  Your Setup

**Printer:** US Letter (8.5" × 11")
**Pattern to Use:** `checkerboard_6x7_tiny_US_letter.svg` ⭐

##  3-Step Process

### 1. Print & Measure

```bash
# For US Letter: Print checkerboard_6x7_tiny_US_letter.svg at 100% scale
# Make sure: Page size = US Letter, Scale = 100%, NO "Fit to Page"
# Then MEASURE one square with ruler
```

**Example:** If you measure 12mm → use `--square 12`
**Example:** If you measure 11mm → use `--square 11`

### 2. Mount

- Glue to foam board or cardboard
- Ensure perfectly flat
- Let dry completely

### 3. Calibrate

```bash
cd /home/jx/Dev/Robotics/pincherx100/camera_calibration
source venv/bin/activate

# Use YOUR measured square size (e.g., 12mm):
python calibrate_camera.py --pattern 6x7 --square 12

# Follow prompts:
# - Press SPACE to capture (15-20 images)
# - Vary angles and distances
# - Press C to calibrate
```

##  Quick Commands

```bash
# Setup (first time only)
./setup.sh

# Interactive menu
./quick_start.sh

# Stream camera to Mac
python camera_stream_server.py

# Calibrate (adjust --square to your measurement!)
python calibrate_camera.py --pattern 6x7 --square 12

# Test calibration
python test_calibration.py
```

##  Pattern Specifications

**checkerboard_6x7_tiny_US_letter.svg:**
- Paper: US Letter (8.5" × 11")
- Inner corners: 6 × 7
- Total squares: 7 × 8 = 56 squares
- Designed square size: 12mm
- Total size: 8.4cm × 9.6cm
- **Fits within: 8.5cm × 10cm ✓**

##  Key Points

1. **Size doesn't matter** - any size works for calibration!
2. **MUST measure** - use ruler to get actual square size
3. **Use measured size** - put it in `--square` parameter
4. **Zhang's method** - already implemented in OpenCV

##  The Code Handles Everything

The calibration code works with ANY square size because:

```python
# OpenCV calibrateCamera() scales automatically
# You just provide the square size you measured
# Math works the same for 8mm, 12mm, 15mm, or 25mm squares!
```

**No code changes needed - just use correct --square value!**

##  Common Mistakes

 Don't assume square size is 12mm without measuring
 Always measure with ruler and use actual size

 Don't worry if it prints smaller/larger than designed
 Just measure and use that value

 Don't try to scale pattern to exact size
 Print at 100%, measure, use measured value

##  Full Documentation

- **PRINTER_LIMITATION_GUIDE.md** - Detailed explanation
- **README.md** - Complete reference
- **GETTING_STARTED.md** - Quick start guide

##  Remember

**The actual printed size does NOT affect calibration accuracy!**

What matters:
1.  Measure accurately
2.  Pattern is flat
3.  Capture varied images
4.  Use measured size in command

That's it! 

