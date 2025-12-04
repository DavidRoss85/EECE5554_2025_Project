#!/usr/bin/env python3
"""
Coordinate Transformation Module - Simplified

Converts between camera pixel coordinates and robot base coordinates using simple geometry.
No hand-eye calibration required - uses known camera position.

Coordinate System:
- Robot Base: (0, 0, 0) at bottom of base motor
  - +X: Right (base at 180°, servo 999)
  - +Y: Forward (base at 90°, servo 2048)
  - +Z: Up (height above platform)

- Camera: Located at (0, 0.21, 0.51) in base frame
  - Pointing down (-Z direction)
  - Image: 1280 (width/X) x 720 (height/Y) pixels
  - X_cam: Right in image
  - Y_cam: Down in image  
  - Z_cam: Distance down from camera

Transformation:
  Pixel (u,v) → Camera 3D → Base 3D → IK → Servo positions
"""

import numpy as np
import cv2
import json
import os


class CoordinateTransformer:
    """
    Transforms coordinates between camera frame and robot base frame.
    Uses simple geometry based on known camera position.
    """
    
    def __init__(self, camera_calib_file, robot_config_file=None):
        """
        Initialize coordinate transformer.
        
        Args:
            camera_calib_file: Path to camera calibration file (.npz or .json)
            robot_config_file: Path to robot config YAML (optional)
        """
        # Load camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None
        
        self._load_camera_calibration(camera_calib_file)
        
        # Camera position relative to robot base
        if robot_config_file and os.path.exists(robot_config_file):
            self._load_robot_config(robot_config_file)
        else:
            # Default values: camera at (0, 0.21, 0.51) in base frame
            self.camera_x = 0.0    # Centered on base in X (0mm)
            self.camera_y = 0.21   # 210mm forward from base center
            self.camera_z = 0.51   # 510mm above platform
        
        print(f"Camera position in base frame: ({self.camera_x:.3f}, {self.camera_y:.3f}, {self.camera_z:.3f}) m")
    
    def _load_camera_calibration(self, calib_file):
        """Load camera intrinsic parameters."""
        if calib_file.endswith('.npz'):
            data = np.load(calib_file)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            if 'image_size' in data:
                self.image_size = tuple(data['image_size'])
        elif calib_file.endswith('.json'):
            with open(calib_file, 'r') as f:
                data = json.load(f)
            self.camera_matrix = np.array(data['camera_matrix'])
            self.dist_coeffs = np.array(data['dist_coeffs'])
            if 'image_size' in data:
                self.image_size = tuple(data['image_size'])
        else:
            raise ValueError(f"Unknown calibration file format: {calib_file}")
        
        print(f"Loaded camera calibration from {calib_file}")
        if self.image_size:
            print(f"  Image size: {self.image_size}")
    
    def _load_robot_config(self, config_file):
        """Load robot configuration (camera position)."""
        import yaml
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        cam_pos = config['camera']['position']
        self.camera_x = cam_pos['x']
        self.camera_y = cam_pos['y']
        self.camera_z = cam_pos['z']
    
    def pixel_to_camera_3d(self, pixel_x, pixel_y, z_distance):
        """
        Convert pixel coordinates to 3D position in camera frame.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            z_distance: Distance down from camera (meters)
        
        Returns:
            [X_cam, Y_cam, Z_cam] in camera frame (meters)
        """
        # Undistort pixel
        pixel = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        pixel_undist = cv2.undistortPoints(pixel, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
        
        u_undist = pixel_undist[0][0][0]
        v_undist = pixel_undist[0][0][1]
        
        # Convert to normalized camera coordinates
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x_norm = (u_undist - cx) / fx
        y_norm = (v_undist - cy) / fy
        
        # DEBUG
        print(f"  [TRANSFORM] Pixel: ({pixel_x:.1f}, {pixel_y:.1f})")
        print(f"  [TRANSFORM] Optical center: ({cx:.1f}, {cy:.1f})")
        print(f"  [TRANSFORM] Offset from center: ({pixel_x - cx:.1f}, {pixel_y - cy:.1f}) pixels")
        print(f"  [TRANSFORM] Normalized: ({x_norm:.4f}, {y_norm:.4f})")
        print(f"  [TRANSFORM] Z distance: {z_distance:.4f}m")
        
        # Convert to 3D camera coordinates
        # Camera points down: Z_cam is distance down (positive)
        # X_cam: right in image (positive x_norm → positive X_cam)
        # Y_cam: down in image (positive y_norm → positive Y_cam)
        X_cam = x_norm * z_distance
        Y_cam = y_norm * z_distance
        Z_cam = z_distance
        
        print(f"  [TRANSFORM] Camera 3D: ({X_cam:.4f}, {Y_cam:.4f}, {Z_cam:.4f})")
        
        return np.array([X_cam, Y_cam, Z_cam])
    
    def camera_3d_to_base_3d(self, point_cam):
        """
        Convert 3D point from camera frame to robot base frame.
        
        Simple geometry transformation:
        - Camera is at (camera_x, camera_y, camera_z) in base frame
        - Camera points down (-Z direction in base frame)
        - X_cam (right) → X_base (right)
        - Y_cam (down in image) → forward in base frame
        - Z_cam (distance down) → height in base frame
        
        Args:
            point_cam: [X_cam, Y_cam, Z_cam] in camera frame (meters)
        
        Returns:
            point_base: [X_base, Y_base, Z_base] in robot base frame (meters)
        """
        X_cam, Y_cam, Z_cam = point_cam
        
        # Simple geometric transformation
        # Camera at (0, 0.21, 0.51), pointing down
        # Object in camera frame → object in base frame
        X_base = X_cam + self.camera_x  # Right is right
        Y_base = Y_cam + self.camera_y   # Down in image = forward
        Z_base = self.camera_z - Z_cam   # Distance down from camera → height above platform
        
        print(f"  [TRANSFORM] Camera position: ({self.camera_x:.3f}, {self.camera_y:.3f}, {self.camera_z:.3f})")
        print(f"  [TRANSFORM] Base 3D: ({X_base:.4f}, {Y_base:.4f}, {Z_base:.4f})")
        
        return np.array([X_base, Y_base, Z_base])
    
    def pixel_to_base_3d(self, pixel_x, pixel_y, object_height=0.0):
        """
        Convert pixel coordinates directly to robot base 3D coordinates.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            object_height: Height of object above platform (meters)
                         For bottle cap: height of bottle
                         For platform objects: 0.0
        
        Returns:
            [X_base, Y_base, Z_base] in robot base frame (meters)
        """
        # Distance from camera to object = camera_z - object_height
        z_distance = self.camera_z - object_height
        
        # Convert pixel to camera 3D
        point_cam = self.pixel_to_camera_3d(pixel_x, pixel_y, z_distance)
        
        # Convert camera 3D to base 3D
        point_base = self.camera_3d_to_base_3d(point_cam)
        
        return point_base


def test_transformation():
    """Test the coordinate transformation."""
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'camera_calibration'))
    
    # Load calibration
    calib_file = '../../camera_calibration/camera_calibration.npz'
    if not os.path.exists(calib_file):
        print(f"Calibration file not found: {calib_file}")
        return
    
    transformer = CoordinateTransformer(calib_file)
    
    # Test: center of image at platform level (Z=0)
    print("\n" + "="*60)
    print("TEST: Center of image at platform level")
    print("="*60)
    
    # Image center
    if transformer.image_size:
        center_x = transformer.image_size[0] / 2
        center_y = transformer.image_size[1] / 2
    else:
        center_x = 640  # Default
        center_y = 360
    
    print(f"Pixel: ({center_x:.0f}, {center_y:.0f})")
    print(f"Object height: 0.0 m (at platform)")
    
    point_base = transformer.pixel_to_base_3d(center_x, center_y, object_height=0.0)
    print(f"Base coordinates: ({point_base[0]:.4f}, {point_base[1]:.4f}, {point_base[2]:.4f}) m")
    print(f"Expected: (~0.0, ~0.21, ~0.0) - directly below camera at platform level")


if __name__ == '__main__':
    test_transformation()
