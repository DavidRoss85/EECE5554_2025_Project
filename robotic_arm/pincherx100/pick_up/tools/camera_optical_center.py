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
        
        # Load camera calibration
        if calibration_file is None:
            calibration_file = os.path.join(
                os.path.dirname(__file__), 
                '..', 
                'calibration', 
                'camera_calibration.npz'
            )
        
        if os.path.exists(calibration_file):
            calib_data = np.load(calibration_file)
            self.camera_matrix = calib_data['camera_matrix']
            self.dist_coeffs = calib_data['dist_coeffs']
            
            # Extract optical center (principal point)
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            
            print("="*60)
            print("CAMERA CALIBRATION LOADED")
            print("="*60)
            print(f"Optical Center (cx, cy): ({self.cx:.1f}, {self.cy:.1f}) pixels")
            print(f"Focal Length (fx, fy): ({self.fx:.1f}, {self.fy:.1f}) pixels")
            print("="*60)
        else:
            print(f"WARNING: Calibration file not found: {calibration_file}")
            print("Using image center as optical center")
            self.camera_matrix = None
            self.cx = None
            self.cy = None
        
        # Open camera
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.camera_index}")
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Get actual resolution
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # If no calibration, use image center
        if self.cx is None:
            self.cx = self.width / 2
            self.cy = self.height / 2
        
        print(f"\nCamera Resolution: {self.width}x{self.height}")
        print(f"Optical Center will be marked at: ({self.cx:.1f}, {self.cy:.1f})")
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
        
        # Draw crosshair lines
        cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
        cv2.line(frame, (x, y - size), (x, y + size), color, thickness)
        
        # Draw center circle
        cv2.circle(frame, (x, y), 5, color, -1)
        
        # Draw outer circle
        cv2.circle(frame, (x, y), size, color, thickness)
        
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
                "CONTROLS:",
                "  Q - Quit",
                "  S - Save screenshot",
                "  H - Toggle this help"
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
        
        # Optical center coordinates (always show)
        coord_text = f"Optical Center: ({self.cx:.1f}, {self.cy:.1f}) px"
        cv2.putText(frame, coord_text, (10, frame.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
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
        print("="*60)
        print("\nShowing optical center marker...")
        print("Place a marker on the platform under the RED CROSSHAIR")
        print("Then measure the distance from robot base to this marker\n")
        
        frame_count = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame")
                    break
                
                # Draw crosshair at optical center
                frame = self.draw_crosshair(frame, self.cx, self.cy, 
                                           color=(0, 0, 255), size=50, thickness=3)
                
                # Draw additional reference circles
                cv2.circle(frame, (int(self.cx), int(self.cy)), 100, (0, 0, 255), 2)
                cv2.circle(frame, (int(self.cx), int(self.cy)), 150, (0, 0, 255), 1)
                
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

