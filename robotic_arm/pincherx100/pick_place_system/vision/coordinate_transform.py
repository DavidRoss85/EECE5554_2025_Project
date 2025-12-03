#!/usr/bin/env python3
"""
Coordinate Transformation Module

Converts between camera pixel coordinates and robot base coordinates.
Handles the transformation chain:
- Camera pixel (u, v) -> Camera 3D (X_cam, Y_cam, Z_cam)
- Camera 3D -> Robot base 3D (X_base, Y_base, Z_base)

For top-down camera setup:
- Camera is 50cm above platform
- Camera optical center is 50.5cm forward from base center
- Camera is aligned with robot forward direction
"""

import numpy as np
import cv2
import json
import os


class CoordinateTransformer:
    """
    Transforms coordinates between camera frame and robot base frame.
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
        
        # Camera position relative to robot base (from config or defaults)
        if robot_config_file and os.path.exists(robot_config_file):
            self._load_robot_config(robot_config_file)
        else:
            # Default values from user specification
            self.camera_x = 0.0  # Camera centered on base in X (0mm)
            self.camera_y = 0.21  # 210mm forward from base center
            self.camera_z = 0.51  # 510mm above platform
        
        # Hand-eye calibration transform (will be set after calibration)
        self.hand_eye_transform = None
        self.hand_eye_loaded = False
        
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
        print(f"  Image size: {self.image_size}")
        print(f"  Camera matrix:\n{self.camera_matrix}")
    
    def _load_robot_config(self, config_file):
        """Load robot configuration (camera position)."""
        import yaml
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        cam_pos = config['camera']['position']
        self.camera_x = cam_pos['x']
        self.camera_y = cam_pos['y']
        self.camera_z = cam_pos['z']
        
        print(f"Loaded camera position from config: ({self.camera_x}, {self.camera_y}, {self.camera_z})")
    
    def load_hand_eye_calibration(self, hand_eye_file):
        """
        Load hand-eye calibration result.
        
        Args:
            hand_eye_file: Path to hand-eye calibration JSON file
        """
        with open(hand_eye_file, 'r') as f:
            data = json.load(f)
        
        # Load transformation matrix (4x4)
        if 'transform_matrix' in data:
            self.hand_eye_transform = np.array(data['transform_matrix'])
            self.hand_eye_loaded = True
            print(f"Loaded hand-eye calibration from {hand_eye_file}")
        else:
            print(f"Warning: No transform_matrix found in {hand_eye_file}")
            self.hand_eye_loaded = False
    
    def pixel_to_camera_3d(self, pixel_x, pixel_y, z_distance):
        """
        Convert pixel coordinates to 3D camera coordinates.
        
        For top-down camera, we assume objects are on a known plane (z_distance from camera).
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            z_distance: Distance from camera to object plane (in meters)
                      For top-down: z_distance = camera_height - object_height
        
        Returns:
            (X_cam, Y_cam, Z_cam): 3D coordinates in camera frame (meters)
        """
        # Undistort pixel coordinates
        pixel_undist = cv2.undistortPoints(
            np.array([[[pixel_x, pixel_y]]], dtype=np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            P=self.camera_matrix
        )[0][0]
        
        u_undist = pixel_undist[0]
        v_undist = pixel_undist[1]
        
        # Convert to normalized camera coordinates
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x_norm = (u_undist - cx) / fx
        y_norm = (v_undist - cy) / fy
        
        # Convert to 3D camera coordinates
        # For top-down camera: Z_cam is positive downward
        X_cam = x_norm * z_distance
        Y_cam = y_norm * z_distance
        Z_cam = z_distance
        
        # DEBUG: Print pixel to camera 3D conversion
        print(f"    [DEBUG] Pixel to Camera 3D:")
        print(f"      Pixel: ({pixel_x:.1f}, {pixel_y:.1f})")
        print(f"      Undistorted: ({u_undist:.1f}, {v_undist:.1f})")
        print(f"      Normalized: x_norm={x_norm:.4f}, y_norm={y_norm:.4f}")
        print(f"      Z distance: {z_distance:.4f}m")
        print(f"      Camera 3D: X_cam={X_cam:.4f}, Y_cam={Y_cam:.4f}, Z_cam={Z_cam:.4f}")
        
        return np.array([X_cam, Y_cam, Z_cam])
    
    def camera_3d_to_base_3d(self, point_cam):
        """
        Convert 3D point from camera frame to robot base frame.
        
        Camera frame (top-down, pointing down):
        - X_cam: right in image (image x-axis, 0-1280 pixels)
        - Y_cam: down in image (image y-axis, 0-720 pixels, increases downward)
        - Z_cam: distance DOWN from camera (depth)
        
        Robot base frame:
        - X_base: right (+X) = base at 180° (servo 999), left (-X) = base at 0° (servo 3093)
        - Y_base: forward (+Y) = base at 90° (servo 2048)
        - Z_base: up (height above platform)
        
        Coordinate mapping:
        - Image x-axis (pixel_x, 0-1280) -> Robot X-axis (right/left)
        - Image y-axis (pixel_y, 0-720, increases downward) -> Robot Y-axis (forward/backward)
        
        For top-down camera:
        - X_cam (right in image) -> X_base (right in robot)
        - Y_cam (down in image) -> Y_base (forward in robot)
          Note: pixel_y increases downward, so lower pixel_y = further forward
          Therefore: Y_base = camera_y - Y_cam (inverted)
        - Z_cam (distance down) -> Z_base (height above platform)
          Z_base = camera_z - Z_cam
        
        Args:
            point_cam: [X_cam, Y_cam, Z_cam] in camera frame (meters)
        
        Returns:
            point_base: [X_base, Y_base, Z_base] in robot base frame (meters)
        """
        if self.hand_eye_loaded and self.hand_eye_transform is not None:
            # Use calibrated transformation
            print(f"    [DEBUG] Using hand-eye calibration transform")
            print(f"    [DEBUG] Camera 3D (input): X_cam={point_cam[0]:.4f}, Y_cam={point_cam[1]:.4f}, Z_cam={point_cam[2]:.4f}")
            point_cam_homogeneous = np.append(point_cam, 1.0)
            point_base_homogeneous = self.hand_eye_transform @ point_cam_homogeneous
            point_base = point_base_homogeneous[0:3]
            print(f"    [DEBUG] Base 3D (from hand-eye): X_base={point_base[0]:.4f}, Y_base={point_base[1]:.4f}, Z_base={point_base[2]:.4f}")
            print(f"    [DEBUG] Hand-eye transform matrix:")
            print(f"    {self.hand_eye_transform}")
            return point_base
        else:
            # Use approximate transformation based on camera position
            X_cam, Y_cam, Z_cam = point_cam
            
            # Transform to base frame
            # X_cam: right in image -> X_base (right in robot)
            X_base = X_cam + self.camera_x
            
            # Y_cam: down in image -> Y_base (forward in robot)
            # pixel_y increases downward, so lower pixel_y = further forward
            # Therefore: Y_base = camera_y - Y_cam (inverted)
            Y_base = self.camera_y - Y_cam
            
            # Z_cam: distance down from camera -> Z_base (height above platform)
            Z_base = self.camera_z - Z_cam
            
            # DEBUG: Print transformation details
            print(f"    [DEBUG] Camera 3D: X_cam={X_cam:.4f}, Y_cam={Y_cam:.4f}, Z_cam={Z_cam:.4f}")
            print(f"    [DEBUG] Base 3D: X_base={X_base:.4f}, Y_base={Y_base:.4f}, Z_base={Z_base:.4f}")
            print(f"    [DEBUG] Camera position: ({self.camera_x}, {self.camera_y}, {self.camera_z})")
            
            return np.array([X_base, Y_base, Z_base])
    
    def pixel_to_base_3d(self, pixel_x, pixel_y, object_height=0.0):
        """
        Convert pixel coordinates directly to robot base coordinates.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            object_height: Height of object above platform (meters)
                          Default 0.0 for objects on platform
        
        Returns:
            (X_base, Y_base, Z_base): 3D coordinates in robot base frame (meters)
        """
        # Calculate distance from camera to object
        platform_height = 0.0  # Platform is at Z=0 in base frame
        camera_to_platform = self.camera_z - platform_height
        z_distance = camera_to_platform - object_height
        
        # Convert pixel to camera 3D
        point_cam = self.pixel_to_camera_3d(pixel_x, pixel_y, z_distance)
        
        # Convert camera 3D to base 3D
        point_base = self.camera_3d_to_base_3d(point_cam)
        
        return point_base
    
    def base_3d_to_pixel(self, point_base):
        """
        Convert robot base coordinates to pixel coordinates.
        
        Args:
            point_base: [X_base, Y_base, Z_base] in robot base frame (meters)
        
        Returns:
            (pixel_x, pixel_y): Pixel coordinates in image
        """
        # Transform base to camera frame
        if self.hand_eye_loaded and self.hand_eye_transform is not None:
            # Use inverse of hand-eye transform
            point_base_homogeneous = np.append(point_base, 1.0)
            transform_inv = np.linalg.inv(self.hand_eye_transform)
            point_cam_homogeneous = transform_inv @ point_base_homogeneous
            point_cam = point_cam_homogeneous[0:3]
        else:
            # Approximate transformation (inverse of camera_3d_to_base_3d)
            X_base, Y_base, Z_base = point_base
            # Inverse transformation:
            X_cam = X_base - self.camera_x  # Right
            Y_cam = self.camera_y - Y_base  # Forward (inverted)
            Z_cam = self.camera_z - Z_base  # Height (inverted)
            point_cam = np.array([X_cam, Y_cam, Z_cam])
        
        # Project to image plane
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        if point_cam[2] <= 0:
            return None, None  # Behind camera
        
        x_norm = point_cam[0] / point_cam[2]
        y_norm = point_cam[1] / point_cam[2]
        
        pixel_x = x_norm * fx + cx
        pixel_y = y_norm * fy + cy
        
        # Apply distortion
        pixel_dist = cv2.projectPoints(
            point_cam.reshape(1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            self.camera_matrix,
            self.dist_coeffs
        )[0][0][0]
        
        return pixel_dist[0], pixel_dist[1]


if __name__ == '__main__':
    # Test coordinate transformation
    calib_file = "../camera_calibration/camera_calibration.npz"
    
    transformer = CoordinateTransformer(calib_file)
    
    # Test: center of image should map to approximately (0, 0.505, 0) in base frame
    center_x, center_y = 640, 360  # Center of 1280x720 image
    
    point_base = transformer.pixel_to_base_3d(center_x, center_y, object_height=0.0)
    print(f"Image center ({center_x}, {center_y}) -> Base: ({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})")

