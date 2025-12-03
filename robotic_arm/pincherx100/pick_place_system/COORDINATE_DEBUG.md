# Coordinate Transformation Debug

## Problem
- Pixel: (1212, 408) in 1280x720 image
- Result: (0.115, 0.820, 0.000) in base frame
- Distance: 0.8281m (way too far! Max reach is 0.2068m)

## Camera Setup
- Camera position: (0.0, 0.505, 0.50) - 50.5cm forward, 50cm above platform
- Camera pointing DOWN (pitch = -90 degrees)
- Image size: 1280x720

## Camera Frame Definition (Current)
- X_cam: right (image width direction)
- Y_cam: down (image height direction)
- Z_cam: distance DOWN from camera

## Transformation (Current)
```
X_base = X_cam + camera_x
Y_base = camera_y - Y_cam  # Inverted because pixel_y increases downward
Z_base = camera_z - Z_cam
```

## Analysis Needed

1. **Pixel to Camera 3D:**
   - pixel_x = 1212 (right side of image, cx = 599.85)
   - pixel_y = 408 (middle-bottom, cy = 326.36)
   - z_distance = 0.50 - 0.185 = 0.315m (camera to cap)
   
   Calculate:
   - x_norm = (1212 - 599.85) / 1680.8 = 0.364
   - y_norm = (408 - 326.36) / 1681.7 = 0.049
   - X_cam = 0.364 * 0.315 = 0.115m
   - Y_cam = 0.049 * 0.315 = 0.015m
   - Z_cam = 0.315m

2. **Camera 3D to Base 3D:**
   - X_base = 0.115 + 0.0 = 0.115m ✓
   - Y_base = 0.505 - 0.015 = 0.490m (should be closer to camera_y)
   - Z_base = 0.50 - 0.315 = 0.185m (cap height) ✓

Wait, this gives Y_base = 0.490m, but the error shows 0.820m!

Let me check if the transformation was applied correctly...

Actually, the error shows Y_base = 0.820m, which suggests the OLD transformation was used:
- Y_base = Z_cam + camera_y = 0.315 + 0.505 = 0.820m

So the fix I made should work! But we need to verify the coordinate system is correct.

## Questions to Verify

1. **Camera orientation:** Is the camera aligned with robot forward (Y)? Or rotated?
2. **Image orientation:** When pixel_y increases, does the object move forward or backward in robot frame?
3. **Camera position:** Is camera_y = 0.505m correct? (50.5cm forward from base)

## Test Cases

1. **Image center (640, 360):**
   - Should map to approximately (0.0, 0.505, 0.0) - camera forward position
   
2. **Top of image (640, 0):**
   - Should map to further forward than center
   
3. **Bottom of image (640, 720):**
   - Should map to closer to base than center

