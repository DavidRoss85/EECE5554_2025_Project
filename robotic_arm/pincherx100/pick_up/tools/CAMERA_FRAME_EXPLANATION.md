# Camera Frame Coordinate System Explanation

## Overview

The **Camera Frame** coordinates (X_cam, Y_cam, Z_cam) represent the 3D position of an object **relative to the camera's optical center and orientation**.

## Reference Point: Optical Center

**YES, the optical center (cx, cy) is the reference point!**

The camera frame uses:
- **Origin**: The camera's optical center (principal point)
- **X-axis**: Right in the image (positive = right of optical center)
- **Y-axis**: Down in the image (positive = below optical center)  
- **Z-axis**: Forward from camera (positive = away from camera, into the scene)

## How Camera Frame Coordinates are Calculated

### Step 1: Pixel to Normalized Coordinates

From your output:
- **Pixel**: (611.8, 281.8)
- **Optical Center (cx, cy)**: (603.2, 410.9) from calibration
- **Focal Length (fx, fy)**: (1378.3, 1376.3) pixels

First, the pixel is **undistorted** (removes lens distortion), then converted to normalized coordinates:

```
x_norm = (pixel_x - cx) / fx
y_norm = (pixel_y - cy) / fy
```

For your tag:
```
x_norm = (611.8 - 603.2) / 1378.3 = 8.6 / 1378.3 = 0.00624
y_norm = (281.8 - 410.9) / 1376.3 = -129.1 / 1376.3 = -0.0938
```

**Meaning**:
- `x_norm = 0.00624`: Tag is **0.00624 units to the right** of optical center (normalized)
- `y_norm = -0.0938`: Tag is **0.0938 units above** optical center (negative = above, positive = below)

### Step 2: Normalized to 3D Camera Coordinates

The normalized coordinates are multiplied by the **Z distance** (depth) to get real-world 3D coordinates:

```
X_cam = x_norm * Z_distance
Y_cam = y_norm * Z_distance
Z_cam = Z_distance
```

From your output:
- **Z_distance**: 57.87cm = 0.5787m (detected by AprilTag pose estimation)

So:
```
X_cam = 0.00624 * 0.5787 = 0.0036m = 0.36cm
Y_cam = -0.0938 * 0.5787 = -0.0542m = -5.42cm
Z_cam = 0.5787m = 57.87cm
```

## What the Values Mean

### X_cam = 0.36cm
- **Positive** = object is to the **right** of optical center
- **Negative** = object is to the **left** of optical center
- **0.36cm** = Tag is **0.36cm to the right** of where the optical center points

### Y_cam = -5.42cm
- **Positive** = object is **below** optical center (down in image)
- **Negative** = object is **above** optical center (up in image)
- **-5.42cm** = Tag is **5.42cm above** where the optical center points

### Z_cam = 57.87cm
- **Distance from camera** to the tag
- Measured along the camera's viewing direction

## Visual Representation

```
                    Camera (top view, looking down)
                          
                    Optical Center (cx, cy)
                         |
                         |  Z_cam (forward)
                         |
                    [Tag] ← 57.87cm away
                    ↑
                    0.36cm to the right
                    
                    
                    Camera (side view)
                          
                    Optical Center
                         |
                         |  Z_cam
                         |
                    [Tag] ← 57.87cm away
                    ↑
                    5.42cm above optical center
```

## Why Y_cam is Negative

In your case, Y_cam = -5.42cm means the tag is **above** the optical center in the image.

This happens because:
- Pixel Y = 281.8 (in image coordinates, Y increases downward)
- Optical center cy = 410.9
- Since 281.8 < 410.9, the tag is **above** the optical center
- In normalized coordinates: y_norm = (281.8 - 410.9) / 1376.3 = **negative**
- Therefore Y_cam is **negative** (above optical center)

## Summary

**Camera Frame coordinates are relative to the optical center:**
- **X_cam**: Horizontal offset (right/left) from optical center
- **Y_cam**: Vertical offset (above/below) from optical center  
- **Z_cam**: Distance from camera along viewing direction

The optical center (cx, cy) is the **reference point** - it's where the camera "looks" straight ahead. All camera frame coordinates are measured relative to this point.

