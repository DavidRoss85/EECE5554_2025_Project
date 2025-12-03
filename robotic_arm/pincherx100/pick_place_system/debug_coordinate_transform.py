#!/usr/bin/env python3
"""
Debug script to check coordinate transformation values.
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'vision'))
from coordinate_transform import CoordinateTransformer

# Load configuration
calib_file = "../camera_calibration/camera_calibration.npz"
config_file = "configs/robot_config.yaml"

transformer = CoordinateTransformer(calib_file, config_file)

# From error.txt
pixel_x = 1212.0
pixel_y = 408.0
object_height = 0.185  # 18.5cm

print("="*60)
print("COORDINATE TRANSFORMATION DEBUG")
print("="*60)
print(f"\nInput:")
print(f"  Pixel: ({pixel_x}, {pixel_y})")
print(f"  Object height: {object_height*100:.1f}cm")
print(f"\nCamera position: ({transformer.camera_x}, {transformer.camera_y}, {transformer.camera_z})")
print(f"Camera matrix:\n{transformer.camera_matrix}")

# Calculate z_distance
platform_height = 0.0
camera_to_platform = transformer.camera_z - platform_height
z_distance = camera_to_platform - object_height
print(f"\nDistance calculation:")
print(f"  Camera height: {transformer.camera_z*100:.1f}cm")
print(f"  Platform height: {platform_height*100:.1f}cm")
print(f"  Camera to platform: {camera_to_platform*100:.1f}cm")
print(f"  Object height: {object_height*100:.1f}cm")
print(f"  Z distance (camera to cap): {z_distance*100:.1f}cm")

# Step 1: Pixel to camera 3D
point_cam = transformer.pixel_to_camera_3d(pixel_x, pixel_y, z_distance)
print(f"\nStep 1: Pixel to Camera 3D")
print(f"  Camera coordinates: ({point_cam[0]:.4f}, {point_cam[1]:.4f}, {point_cam[2]:.4f})")

# Step 2: Camera 3D to Base 3D
point_base = transformer.camera_3d_to_base_3d(point_cam)
print(f"\nStep 2: Camera 3D to Base 3D")
print(f"  Base coordinates: ({point_base[0]:.4f}, {point_base[1]:.4f}, {point_base[2]:.4f})")

# Calculate distance from base
distance = np.sqrt(point_base[0]**2 + point_base[1]**2)
print(f"\nDistance from base center: {distance*100:.1f}cm")
print(f"Max reach: ~20.7cm")

# Check what the transformation is doing
print(f"\nTransformation breakdown:")
print(f"  X_cam = {point_cam[0]:.4f} -> X_base = {point_cam[0]:.4f} + {transformer.camera_x:.4f} = {point_base[0]:.4f}")
print(f"  Y_cam = {point_cam[1]:.4f} -> Z_base = {-point_cam[1]:.4f} + {transformer.camera_z:.4f} = {point_base[2]:.4f}")
print(f"  Z_cam = {point_cam[2]:.4f} -> Y_base = {point_cam[2]:.4f} + {transformer.camera_y:.4f} = {point_base[1]:.4f}")

# Check image center
print(f"\n" + "="*60)
print("IMAGE CENTER TEST")
print("="*60)
center_x, center_y = 640, 360  # Center of 1280x720
center_base = transformer.pixel_to_base_3d(center_x, center_y, object_height=0.0)
print(f"Image center ({center_x}, {center_y}) -> Base: ({center_base[0]:.4f}, {center_base[1]:.4f}, {center_base[2]:.4f})")
print(f"Expected: Should be near (0.0, 0.505, 0.0) - camera forward position")

