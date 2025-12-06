# Perspective Effect on Camera Frame Coordinates

## The Problem

When you move an AprilTag **vertically** (change Z height), its **Y coordinate in camera frame changes** even if its X,Y position in base frame stays the same!

## Why This Happens

### Camera Frame Calculation

```
Y_cam = y_norm × Z_cam
```

Where:
- `y_norm = (pixel_y - cy) / fy` (normalized pixel offset)
- `Z_cam` = distance from camera to tag

### Example Scenario

**Tag at Platform (Z_base = 0):**
- Z_cam = 70cm (camera height above platform)
- Pixel Y = 281.8
- y_norm = (281.8 - 410.9) / 1376.3 = -0.0938
- **Y_cam = -0.0938 × 0.70 = -0.0657m = -6.57cm**

**Tag Lifted 12cm (Z_base = 0.12m):**
- Z_cam = 70cm - 12cm = 58cm (closer to camera!)
- Pixel Y might shift slightly due to perspective
- Let's say y_norm stays similar: -0.0938
- **Y_cam = -0.0938 × 0.58 = -0.0544m = -5.44cm**

**Result**: Y_cam changed from -6.57cm to -5.44cm even though the tag's horizontal position (X, Y in base frame) didn't change!

## Visual Explanation

```
Camera (top-down view, side perspective)

Platform Level (Z=0):
    Camera (70cm high)
         |
         |  Z_cam = 70cm
         |
    [Tag] ← Y_cam = y_norm × 70cm
    
Lifted 12cm (Z=0.12m):
    Camera (70cm high)
         |
         |  Z_cam = 58cm (closer!)
         |
    [Tag] ← Y_cam = y_norm × 58cm (smaller!)
         ↑
      12cm up
```

## The Real Issue

**The camera frame Y coordinate is NOT a direct measure of the tag's Y position in base frame!**

It's affected by:
1. **Pixel position** (where tag appears in image)
2. **Z distance** (how far tag is from camera)

## Why This Matters for Your Calibration

When you measured:
- **Actual position**: (0, 0.16, 0.12) meters
- **Detected Base Frame (RAW)**: (0.0036, 0.0357, 0.1189) meters

The Y difference (0.16 - 0.0357 = 0.1243m = 12.43cm) might be partly due to:
1. **Camera position error** (camera_y in config might be wrong)
2. **Perspective effect** (the tag's height affects Y_cam calculation)
3. **Coordinate transformation error**

## Solution

The coordinate transformation should account for the tag's **actual height** when converting from camera frame to base frame. The current transformation assumes the tag is at platform level (Z=0), but if it's at Z=0.12m, this affects the calculation.

This is why the `pixel_to_base_3d` function has an `object_height` parameter - to account for objects that are not on the platform!

