#!/usr/bin/env python3
"""
Camera Optical Center Visualization Tool

This tool displays the camera's optical center (cx, cy) on the live video feed
with a red crosshair marker. Use this to physically measure the distance from
the robot base to where the optical center points on the ground/platform.

Usage:
    python camera_optical_center.py

Instructions:
    1. Run this script to see the red crosshair marking the optical center
    2. Place a marker (coin, tape, etc.) on the platform directly under the crosshair
    3. Measure the distance from robot base center to this marker
    4. Use these measurements to update camera position in robot_config.yaml

Controls:
    - Press 'Q' to quit
    - Press 'S' to save a screenshot
    - Press 'H' to toggle help text
"""

import sys
import os
import cv2
import numpy as np
from datetime import datetime

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class OpticalCenterVisualizer:
    """Visualize camera optical center on live feed."""
    
    def __init__(self, camera_index=0, calibration_file=None):
        """
        Initialize visualizer.
        
        Args:
            camera_index: Camera device index
            calibration_file: Path to camera calibration .npz file
        """
        self.camera_index = camera_index
        self.show_help = True
        self.show_grid = False
        
        # Load camera calibration
        if calibration_file is None:
            calibration_file = os.path.join(
                os.path.dirname(__file__), 
                '..', 
                'calibration', 
                'camera_calibration.npz'
            )
        
        # Load calibration data
        self.calib_data = None
        if os.path.exists(calibration_file):
            self.calib_data = np.load(calibration_file)
            self.camera_matrix = self.calib_data['camera_matrix']
            self.dist_coeffs = self.calib_data['dist_coeffs']
            
            # Extract optical center (principal point) from calibration
            self.cx_calib = self.camera_matrix[0, 2]
            self.cy_calib = self.camera_matrix[1, 2]
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            
            print("="*60)
            print("CAMERA CALIBRATION LOADED")
            print("="*60)
            print(f"Calibration Optical Center (cx, cy): ({self.cx_calib:.1f}, {self.cy_calib:.1f}) pixels")
            print(f"Focal Length (fx, fy): ({self.fx:.1f}, {self.fy:.1f}) pixels")
            print("="*60)
        else:
            print(f"WARNING: Calibration file not found: {calibration_file}")
            print("Using image center as optical center")
            self.camera_matrix = None
            self.dist_coeffs = None
            self.cx_calib = None
            self.cy_calib = None
            self.fx = None
            self.fy = None
        
        # Open camera
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.camera_index}")
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Get actual resolution (read a frame first to ensure resolution is set)
        ret, test_frame = self.cap.read()
        if ret:
            self.height, self.width = test_frame.shape[:2]
        else:
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Calculate optical center based on actual frame size
        # If calibration exists, scale the optical center to match current resolution
        if self.camera_matrix is not None and self.calib_data is not None:
            # Get calibration image size if available
            # Note: image_size is stored as [width, height] in calibration files
            if 'image_size' in self.calib_data:
                calib_width, calib_height = self.calib_data['image_size']
            else:
                # Assume calibration was done at same resolution
                calib_width = self.width
                calib_height = self.height
            
            # Scale optical center if resolution differs
            if calib_width != self.width or calib_height != self.height:
                scale_x = self.width / calib_width
                scale_y = self.height / calib_height
                self.cx = self.cx_calib * scale_x
                self.cy = self.cy_calib * scale_y
                print(f"Scaled optical center from calibration ({calib_width}x{calib_height})")
                print(f"  to current resolution ({self.width}x{self.height})")
            else:
                # Same resolution - use calibration values directly
                self.cx = self.cx_calib
                self.cy = self.cy_calib
                print(f"Using calibration optical center directly (resolution matches: {self.width}x{self.height})")
            
            # Note: We don't apply undistortion for this tool because:
            # 1. The optical center (cx, cy) is in raw image coordinates
            # 2. Undistortion would remap pixels and make crosshair position incorrect
            # 3. For calibration, we need to see where the camera actually points in raw image
        else:
            # If no calibration, use image center
            self.cx = self.width / 2
            self.cy = self.height / 2
        
        print(f"\nCamera Resolution: {self.width}x{self.height}")
        print(f"Optical Center will be marked at: ({self.cx:.1f}, {self.cy:.1f})")
        print("Note: Showing RAW image (no undistortion) for accurate calibration")
        print("      Optical center coordinates are in raw image space")
        print("\n")
    
    def draw_crosshair(self, frame, x, y, color=(0, 0, 255), size=40, thickness=2):
        """
        Draw crosshair marker.
        
        Args:
            frame: Image frame
            x, y: Center coordinates
            color: BGR color (default: red)
            size: Crosshair size
            thickness: Line thickness
        """
        x, y = int(x), int(y)
        
        # Draw crosshair lines (longer for better visibility)
        cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
        cv2.line(frame, (x, y - size), (x, y + size), color, thickness)
        
        # Draw center circle (filled)
        cv2.circle(frame, (x, y), 5, color, -1)
        
        # Draw outer circle
        cv2.circle(frame, (x, y), size, color, thickness)
        
        return frame
    
    def draw_reference_grid(self, frame, step=100):
        """
        Draw a reference grid to help verify alignment.
        
        Args:
            frame: Image frame
            step: Grid spacing in pixels
        """
        h, w = frame.shape[:2]
        
        # Draw vertical lines
        for x in range(0, w, step):
            cv2.line(frame, (x, 0), (x, h), (100, 100, 100), 1)
        
        # Draw horizontal lines
        for y in range(0, h, step):
            cv2.line(frame, (0, y), (w, y), (100, 100, 100), 1)
        
        # Draw center lines (thicker)
        cv2.line(frame, (w//2, 0), (w//2, h), (150, 150, 150), 2)
        cv2.line(frame, (0, h//2), (w, h//2), (150, 150, 150), 2)
        
        return frame
    
    def draw_info(self, frame):
        """Draw information overlay on frame."""
        # Semi-transparent background for text
        overlay = frame.copy()
        
        if self.show_help:
            # Help text background
            cv2.rectangle(overlay, (10, 10), (620, 280), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
            
            # Title
            cv2.putText(frame, "OPTICAL CENTER VISUALIZATION", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Instructions
            y_pos = 70
            instructions = [
                "The RED CROSSHAIR marks the optical center (cx, cy)",
                "of your camera. This is where the camera 'looks'.",
                "",
                "TO CALIBRATE CAMERA POSITION:",
                "1. Place a marker on the platform under the crosshair",
                "2. Measure X distance (left/right from base center)",
                "3. Measure Y distance (forward from base center)",
                "4. Update camera.position in robot_config.yaml",
                "",
                "GREEN CORNERS verify full frame is visible (no crop)",
                "",
                "CONTROLS:",
                "  Q - Quit",
                "  S - Save screenshot",
                "  H - Toggle this help",
                "  G - Toggle reference grid"
            ]
            
            for line in instructions:
                cv2.putText(frame, line, (20, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                y_pos += 25
        else:
            # Minimal info
            cv2.rectangle(overlay, (10, 10), (300, 60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
            
            cv2.putText(frame, "Optical Center Marker", (20, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, "Press H for help", (20, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Optical center coordinates and status (always show)
        coord_text = f"Optical Center: ({self.cx:.1f}, {self.cy:.1f}) px"
        status_y = frame.shape[0] - 20
        cv2.putText(frame, coord_text, (10, status_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Show that we're using raw image coordinates
        raw_text = "RAW image (for accurate calibration)"
        cv2.putText(frame, raw_text, (10, status_y - 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return frame
    
    def run(self):
        """Run the visualization loop."""
        print("="*60)
        print("STARTING LIVE VIEW")
        print("="*60)
        print("Controls:")
        print("  Q - Quit")
        print("  S - Save screenshot")
        print("  H - Toggle help overlay")
        print("  G - Toggle reference grid")
        print("="*60)
        print("\nShowing optical center marker...")
        print("Place a marker on the platform under the RED CROSSHAIR")
        print("Then measure the distance from robot base to this marker")
        print("Note: Showing RAW image (no undistortion) for accurate calibration")
        print("      The optical center coordinates are in raw image space")
        print("Green corner markers verify full frame is visible (no cropping)\n")
        
        frame_count = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame")
                    break
                
                # IMPORTANT: We show the RAW (distorted) image because the optical center
                # (cx, cy) from calibration is in the raw image coordinate system.
                # Undistortion would remap pixels and make the crosshair position incorrect.
                # For calibration measurements, we need the raw image coordinates.
                
                # Draw reference grid (optional, can be toggled with 'G' key)
                if self.show_grid:
                    frame = self.draw_reference_grid(frame, step=100)
                
                # Draw crosshair at optical center on RAW image
                # cx, cy are in the original (distorted) image coordinate system
                frame = self.draw_crosshair(frame, self.cx, self.cy, 
                                           color=(0, 0, 255), size=50, thickness=3)
                
                # Draw additional reference circles
                cv2.circle(frame, (int(self.cx), int(self.cy)), 100, (0, 0, 255), 2)
                cv2.circle(frame, (int(self.cx), int(self.cy)), 150, (0, 0, 255), 1)
                
                # Draw corner markers to verify no cropping
                corner_size = 20
                cv2.line(frame, (0, 0), (corner_size, 0), (0, 255, 0), 2)  # Top-left
                cv2.line(frame, (0, 0), (0, corner_size), (0, 255, 0), 2)
                cv2.line(frame, (self.width-1, 0), (self.width-1-corner_size, 0), (0, 255, 0), 2)  # Top-right
                cv2.line(frame, (self.width-1, 0), (self.width-1, corner_size), (0, 255, 0), 2)
                cv2.line(frame, (0, self.height-1), (corner_size, self.height-1), (0, 255, 0), 2)  # Bottom-left
                cv2.line(frame, (0, self.height-1), (0, self.height-1-corner_size), (0, 255, 0), 2)
                cv2.line(frame, (self.width-1, self.height-1), (self.width-1-corner_size, self.height-1), (0, 255, 0), 2)  # Bottom-right
                cv2.line(frame, (self.width-1, self.height-1), (self.width-1, self.height-1-corner_size), (0, 255, 0), 2)
                
                # Draw info overlay
                frame = self.draw_info(frame)
                
                # Show frame
                cv2.imshow('Camera Optical Center', frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("\nQuitting...")
                    break
                elif key == ord('s') or key == ord('S'):
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"optical_center_{timestamp}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Screenshot saved: {filename}")
                elif key == ord('h') or key == ord('H'):
                    self.show_help = not self.show_help
                elif key == ord('g') or key == ord('G'):
                    self.show_grid = not self.show_grid
                    print(f"Reference grid: {'ON' if self.show_grid else 'OFF'}")
                
                frame_count += 1
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            print("\n" + "="*60)
            print("MEASUREMENT GUIDE")
            print("="*60)
            print("\nYou should have placed a marker under the red crosshair.")
            print("\nNow measure:")
            print("  1. X distance: left(-) or right(+) from base center")
            print("  2. Y distance: forward from base center")
            print("  3. Z distance: camera height above platform")
            print("\nUpdate these in configs/robot_config.yaml:")
            print("\ncamera:")
            print("  position:")
            print("    x: <your_measured_x>  # in meters")
            print("    y: <your_measured_y>  # in meters")
            print("    z: <your_measured_z>  # in meters")
            print("="*60)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Camera Optical Center Visualization Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python camera_optical_center.py
  python camera_optical_center.py --camera 1
  python camera_optical_center.py --calibration ../calibration/camera_calibration.npz

This tool helps you calibrate the camera position by showing where
the optical center points on the platform.
        """
    )
    
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--calibration', type=str, default=None,
                       help='Path to camera calibration file (.npz)')
    
    args = parser.parse_args()
    
    try:
        visualizer = OpticalCenterVisualizer(
            camera_index=args.camera,
            calibration_file=args.calibration
        )
        visualizer.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

