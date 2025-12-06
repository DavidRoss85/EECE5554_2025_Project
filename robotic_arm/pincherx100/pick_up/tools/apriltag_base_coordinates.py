#!/usr/bin/env python3
"""
AprilTag Base Coordinates Tool

Live camera tool that detects AprilTags and shows their coordinates
in the robot base frame. This is for debugging coordinate transformation
accuracy.

Usage:
    python apriltag_base_coordinates.py --size 0.0254

The tool shows:
    - Camera frame coordinates (X, Y, Z in cm)
    - Base frame coordinates RAW (before offsets, in cm)
    - Base frame coordinates ADJUSTED (after offsets, in cm)
    - Distance from base center (2D and 3D)

Dependencies:
    - Camera calibration: calibration/camera_calibration.npz
    - Robot config: configs/robot_config.yaml
      - camera.position (x, y, z)
      - coordinate_transform.x_offset, y_offset, z_offset
      - apriltags.tag_size (default if --size not provided)
"""

import sys
import os
import argparse
import cv2
import numpy as np
import yaml
from datetime import datetime

# Add parent directories to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from apriltag_detector import AprilTagDetector
from coordinate_transform import CoordinateTransformer


class AprilTagBaseCoordinateTool:
    """Tool to display AprilTag coordinates in robot base frame."""
    
    def __init__(self, tag_size_m, camera_index=0, config_file=None, calib_file=None):
        """
        Initialize the tool.
        
        Args:
            tag_size_m: AprilTag size in meters
            camera_index: Camera device index
            config_file: Path to robot_config.yaml
            calib_file: Path to camera_calibration.npz
        """
        self.tag_size = tag_size_m
        
        # Resolve paths
        base_dir = os.path.join(os.path.dirname(__file__), '..')
        
        if config_file is None:
            config_file = os.path.join(base_dir, 'configs', 'robot_config.yaml')
        
        if calib_file is None:
            calib_file = os.path.join(base_dir, 'calibration', 'camera_calibration.npz')
        
        # Load configuration
        print("="*70)
        print("LOADING CONFIGURATION")
        print("="*70)
        print(f"Config file: {config_file}")
        print(f"Calibration file: {calib_file}")
        
        if not os.path.exists(config_file):
            raise FileNotFoundError(f"Config file not found: {config_file}")
        if not os.path.exists(calib_file):
            raise FileNotFoundError(f"Calibration file not found: {calib_file}")
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Load camera calibration
        calib_data = np.load(calib_file)
        camera_matrix = calib_data['camera_matrix']
        dist_coeffs = calib_data['dist_coeffs']
        
        print(f"\nCamera Matrix:\n{camera_matrix}")
        print(f"Distortion: {dist_coeffs.ravel()}")
        
        # Initialize AprilTag detector
        print(f"\nInitializing AprilTag detector (size: {tag_size_m*1000:.1f}mm)...")
        self.tag_detector = AprilTagDetector(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            tag_size_m=tag_size_m
        )
        
        # Initialize coordinate transformer
        print("Initializing coordinate transformer...")
        self.transformer = CoordinateTransformer(calib_file, config_file)
        
        # Get offsets from config
        coord_config = self.config.get('coordinate_transform', {})
        self.x_offset = coord_config.get('x_offset', 0.0)
        self.y_offset = coord_config.get('y_offset', 0.0)
        self.z_offset = coord_config.get('z_offset', 0.0)
        
        print(f"\nCoordinate offsets (from config):")
        print(f"  X: {self.x_offset*100:.2f} cm")
        print(f"  Y: {self.y_offset*100:.2f} cm")
        print(f"  Z: {self.z_offset*100:.2f} cm")
        
        # Open camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"\nCamera opened: {width}x{height}")
        
        print("="*70)
        print("READY - Starting live detection...")
        print("="*70)
        print("Press Q to quit, S to save frame")
        print()
    
    def format_coord(self, value_m, unit='cm'):
        """Format coordinate value."""
        if unit == 'cm':
            return f"{value_m*100:.2f}"
        return f"{value_m:.4f}"
    
    def print_tag_info(self, detection):
        """Print tag coordinate information."""
        tag_id = detection['tag_id']
        pos_camera = detection['position_camera']  # [x, y, z] in camera frame (meters)
        center_px = detection['center_pixel']
        
        # Transform to base frame
        pos_base_raw = self.transformer.camera_3d_to_base_3d(pos_camera)
        
        # Apply offsets
        pos_base_adj = pos_base_raw.copy()
        pos_base_adj[0] += self.x_offset
        pos_base_adj[1] += self.y_offset
        pos_base_adj[2] += self.z_offset
        
        # Calculate distances
        # 2D: Horizontal distance in XY plane (ignoring height Z)
        distance_2d = np.sqrt(pos_base_adj[0]**2 + pos_base_adj[1]**2)
        # 3D: Full Euclidean distance from base origin (0,0,0) including height
        distance_3d = np.linalg.norm(pos_base_adj)
        
        # Print formatted output
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        print(f"\n[{timestamp}] Tag ID: {tag_id}")
        print(f"  Pixel: ({center_px[0]:.1f}, {center_px[1]:.1f})")
        print(f"  Camera Frame: X={self.format_coord(pos_camera[0])}cm, "
              f"Y={self.format_coord(pos_camera[1])}cm, "
              f"Z={self.format_coord(pos_camera[2])}cm")
        print(f"  Base Frame (RAW): X={self.format_coord(pos_base_raw[0])}cm, "
              f"Y={self.format_coord(pos_base_raw[1])}cm, "
              f"Z={self.format_coord(pos_base_raw[2])}cm")
        print(f"  Base Frame (ADJ): X={self.format_coord(pos_base_adj[0])}cm, "
              f"Y={self.format_coord(pos_base_adj[1])}cm, "
              f"Z={self.format_coord(pos_base_adj[2])}cm")
        print(f"  Distance from base origin:")
        print(f"    2D (XY plane, horizontal): {self.format_coord(distance_2d)}cm")
        print(f"    3D (Euclidean, includes Z): {self.format_coord(distance_3d)}cm")
        print(f"  [Robot uses Base[ADJ] coordinates for IK calculation]")
        print(f"  Quality: H={detection['hamming']}, M={detection['decision_margin']:.1f}")
    
    def run(self):
        """Run the live detection loop."""
        frame_count = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame")
                    break
                
                # Detect tags
                detections = self.tag_detector.detect_tags(frame)
                
                # Print info for each detected tag
                if len(detections) > 0:
                    # Clear previous output (simple approach - just print separator)
                    if frame_count % 30 == 0:  # Print header every 30 frames
                        print("\n" + "="*70)
                        print(f"FRAME {frame_count} - Detected {len(detections)} tag(s)")
                        print("="*70)
                    
                    for det in detections:
                        self.print_tag_info(det)
                else:
                    if frame_count % 60 == 0:  # Print "no tags" less frequently
                        print(f"[Frame {frame_count}] No tags detected")
                
                # Show live feed (simple, no overlay for performance)
                cv2.imshow('AprilTag Detection (Press Q to quit)', frame)
                
                # Handle keys
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('s') or key == ord('S'):
                    filename = f"apriltag_base_coords_{frame_count:04d}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"\nSaved frame to {filename}")
                
                frame_count += 1
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            print("\n" + "="*70)
            print("TOOL SHUTDOWN")
            print("="*70)
            print(f"Total frames processed: {frame_count}")
            print("\nTo verify accuracy:")
            print("  1. Measure actual distance from robot base center to tag")
            print("  2. Compare with 'Distance from base: 2D' value")
            print("  3. Adjust offsets in configs/robot_config.yaml if needed")
            print("="*70)


def main():
    parser = argparse.ArgumentParser(
        description='AprilTag Base Coordinates Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default tag size from config
  python apriltag_base_coordinates.py
  
  # Specify tag size (1 inch = 0.0254m)
  python apriltag_base_coordinates.py --size 0.0254
  
  # Use different camera
  python apriltag_base_coordinates.py --size 0.0254 --camera 1

Dependencies:
  - calibration/camera_calibration.npz (camera intrinsics)
  - configs/robot_config.yaml (camera position, offsets)
  
Output shows:
  - Camera frame coordinates (where tag is relative to camera)
  - Base frame RAW (transformed coordinates before offsets)
  - Base frame ADJUSTED (after applying x_offset, y_offset, z_offset)
  - 2D and 3D distance from robot base center
        """
    )
    
    parser.add_argument('--size', type=float, default=None,
                       help='AprilTag size in meters (default: from robot_config.yaml)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--config', type=str, default=None,
                       help='Path to robot_config.yaml (default: ../configs/robot_config.yaml)')
    parser.add_argument('--calib', type=str, default=None,
                       help='Path to camera_calibration.npz (default: ../calibration/camera_calibration.npz)')
    
    args = parser.parse_args()
    
    # Get default tag size from config if not provided
    tag_size = args.size
    if tag_size is None:
        config_path = args.config or os.path.join(
            os.path.dirname(__file__), '..', 'configs', 'robot_config.yaml'
        )
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            tag_size = config.get('apriltags', {}).get('tag_size', 0.0254)
            print(f"Using tag size from config: {tag_size*1000:.1f}mm")
        else:
            tag_size = 0.0254  # Default 1 inch
            print(f"Config not found, using default tag size: {tag_size*1000:.1f}mm")
    
    try:
        tool = AprilTagBaseCoordinateTool(
            tag_size_m=tag_size,
            camera_index=args.camera,
            config_file=args.config,
            calib_file=args.calib
        )
        tool.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

