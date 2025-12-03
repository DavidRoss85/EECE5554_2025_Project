#!/usr/bin/env python3
"""
Debug script to understand coordinate transformation issue.
The arm moves left when bottle is in front - suggests X/Y swap or sign issue.
"""

import numpy as np
import math

# From IK solver: q[0] = math.atan2(p[1], p[0])
# This means:
# - p[0] = X_base (right)
# - p[1] = Y_base (forward)
# - atan2(Y, X) gives angle from X-axis

# If bottle is in front (positive Y), and X is small:
# - atan2(positive_Y, small_X) ≈ 90° (π/2) = forward
# - This should rotate base forward, not left

# If arm moves LEFT, it means:
# - Base rotation is negative (rotating left)
# - This happens if X is negative OR if Y is negative

# Test case: Bottle detected at pixel (1212, 408) in 1280x720 image
# Image center: (640, 360)
# Camera center: cx=599.85, cy=326.36

pixel_x = 1212
pixel_y = 408
cx = 599.85
cy = 326.36

# Calculate normalized coordinates
# x_norm = (pixel_x - cx) / fx
# y_norm = (pixel_y - cy) / fy

# For pixel (1212, 408):
# - pixel_x > cx (right of center) → positive x_norm → positive X_cam → positive X_base
# - pixel_y > cy (below center) → positive y_norm → positive Y_cam

# Current transformation:
# X_base = X_cam + camera_x
# Y_base = camera_y - Y_cam  # INVERTED!

# If Y_cam is positive (below center), then:
# Y_base = 0.20 - positive_Y_cam = smaller value (closer to base)
# This is WRONG if the bottle is in front!

# The issue: For a top-down camera, when pixel_y increases (going down in image),
# does that mean the object is further forward or further back?

# Actually, for a top-down camera pointing down:
# - Lower pixel_y (top of image) = objects further from camera = further forward
# - Higher pixel_y (bottom of image) = objects closer to camera = closer to base

# So the transformation Y_base = camera_y - Y_cam is correct IF:
# - Y_cam positive = lower in image = further forward
# - But wait, that's backwards!

# Let me think about the camera coordinate system:
# - X_cam: right (positive = right)
# - Y_cam: down in image (positive = down in image)
# - Z_cam: distance down from camera

# For a top-down camera:
# - Top of image (low pixel_y) = objects further away = further forward in robot
# - Bottom of image (high pixel_y) = objects closer = closer to base

# So if pixel_y is high (408, which is below center 326), the object is closer to camera.
# But "closer to camera" means closer to the camera position, which is 20cm forward.
# So the object should be closer to base (smaller Y_base).

# But the user says the bottle is IN FRONT, meaning it should have a larger Y_base.

# This suggests the coordinate system might be inverted, OR the camera is oriented differently.

print("Coordinate System Analysis:")
print("="*60)
print(f"Pixel: ({pixel_x}, {pixel_y})")
print(f"Image center: (640, 360)")
print(f"Camera center: ({cx}, {cy})")
print(f"\nPixel relative to center:")
print(f"  X: {pixel_x - 640:.0f} pixels right")
print(f"  Y: {pixel_y - 360:.0f} pixels down")
print(f"\nIf bottle is in FRONT of arm:")
print(f"  Expected: Positive Y_base (forward)")
print(f"  Expected: Small X_base (centered)")
print(f"  Expected: Base rotation ≈ 90° (forward)")
print(f"\nIf arm moves LEFT:")
print(f"  Actual: Base rotation is negative (left)")
print(f"  This means: atan2(Y_base, X_base) is negative")
print(f"  Possible causes:")
print(f"    1. X_base is negative (bottle to left)")
print(f"    2. Y_base is negative (bottle behind)")
print(f"    3. X and Y are swapped")

