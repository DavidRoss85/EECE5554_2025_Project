#!/usr/bin/env python3
"""
Camera Calibration Script for PincherX100
Calibrates UVC camera using checkerboard pattern

This script will:
1. Open camera feed
2. Detect checkerboard pattern in real-time
3. Capture calibration images when you press SPACE
4. Calculate camera intrinsic parameters
5. Save calibration data

Usage:
    python calibrate_camera.py [--camera 0] [--pattern 8x6] [--square 25]
"""

import cv2
import numpy as np
import glob
import os
import json
import argparse
from datetime import datetime

class CameraCalibration:
    def __init__(self, pattern_size=(8, 6), square_size=25.0, camera_index=0):
        """
        Initialize camera calibration
        
        Args:
            pattern_size: (width, height) of inner corners in checkerboard
            square_size: size of each square in mm
            camera_index: camera device index
        """
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.camera_index = camera_index
        
        # Prepare object points (3D points in real world space)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Arrays to store object points and image points from all images
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        
        self.image_size = None
        self.captured_count = 0
        
        # Calibration results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.calibration_error = None
        
    def find_corners(self, image):
        """
        Find checkerboard corners in image
        
        Returns:
            (found, corners, drawn_image)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, 
            self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        drawn_image = image.copy()
        
        if ret:
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw corners
            cv2.drawChessboardCorners(drawn_image, self.pattern_size, corners_refined, ret)
            
            return True, corners_refined, drawn_image
        
        return False, None, drawn_image
    
    def capture_image(self, image, corners):
        """Capture calibration image"""
        if self.image_size is None:
            self.image_size = (image.shape[1], image.shape[0])
        
        self.objpoints.append(self.objp)
        self.imgpoints.append(corners)
        self.captured_count += 1
        
        # Save the image
        os.makedirs('calibration_images', exist_ok=True)
        filename = f'calibration_images/calib_{self.captured_count:03d}.jpg'
        cv2.imwrite(filename, image)
        
        return filename
    
    def calibrate(self):
        """Perform camera calibration"""
        if len(self.objpoints) < 10:
            return False, "Need at least 10 images for calibration"
        
        print("\n" + "=" * 70)
        print("Calibrating camera...")
        print("=" * 70)
        
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, 
            self.imgpoints, 
            self.image_size,
            None, 
            None
        )
        
        if not ret:
            return False, "Calibration failed"
        
        # Calculate reprojection error
        total_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                self.objpoints[i], 
                self.rvecs[i], 
                self.tvecs[i], 
                self.camera_matrix, 
                self.dist_coeffs
            )
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        
        self.calibration_error = total_error / len(self.objpoints)
        
        print(f"Calibration successful!")
        print(f"  Reprojection error: {self.calibration_error:.4f} pixels")
        print(f"  (Good calibration: < 0.5 pixels, Acceptable: < 1.0 pixels)")
        
        return True, "Calibration successful"
    
    def save_calibration(self, filename='camera_calibration.npz'):
        """Save calibration data"""
        if self.camera_matrix is None:
            return False, "No calibration data to save"
        
        # Save as NumPy format (for OpenCV use)
        np.savez(
            filename,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            image_size=self.image_size,
            calibration_error=self.calibration_error,
            pattern_size=self.pattern_size,
            square_size=self.square_size
        )
        
        # Also save as JSON (for easy reading and other applications)
        json_filename = filename.replace('.npz', '.json')
        calib_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'image_size': list(self.image_size),
            'calibration_error': float(self.calibration_error),
            'pattern_size': list(self.pattern_size),
            'square_size': float(self.square_size),
            'calibration_date': datetime.now().isoformat(),
            'num_images': len(self.objpoints)
        }
        
        with open(json_filename, 'w') as f:
            json.dump(calib_data, f, indent=2)
        
        print(f"\nCalibration saved:")
        print(f"  NumPy format: {filename}")
        print(f"  JSON format:  {json_filename}")
        
        return True, "Calibration saved successfully"
    
    def print_calibration_info(self):
        """Print calibration information"""
        if self.camera_matrix is None:
            print("No calibration data available")
            return
        
        print("\n" + "=" * 70)
        print("Camera Calibration Results")
        print("=" * 70)
        
        print("\nCamera Matrix (Intrinsic Parameters):")
        print(self.camera_matrix)
        
        print("\nDistortion Coefficients:")
        print(f"  k1={self.dist_coeffs[0][0]:.6f}")
        print(f"  k2={self.dist_coeffs[0][1]:.6f}")
        print(f"  p1={self.dist_coeffs[0][2]:.6f}")
        print(f"  p2={self.dist_coeffs[0][3]:.6f}")
        print(f"  k3={self.dist_coeffs[0][4]:.6f}")
        
        fx = self.camera_matrix[0][0]
        fy = self.camera_matrix[1][1]
        cx = self.camera_matrix[0][2]
        cy = self.camera_matrix[1][2]
        
        print(f"\nFocal Length:")
        print(f"  fx = {fx:.2f} pixels")
        print(f"  fy = {fy:.2f} pixels")
        
        print(f"\nPrincipal Point (Optical Center):")
        print(f"  cx = {cx:.2f} pixels")
        print(f"  cy = {cy:.2f} pixels")
        
        print(f"\nImage Size: {self.image_size[0]} x {self.image_size[1]}")
        print(f"Reprojection Error: {self.calibration_error:.4f} pixels")
        print(f"Number of Images Used: {len(self.objpoints)}")
        
        print("=" * 70)

def main():
    parser = argparse.ArgumentParser(description='Camera Calibration for PincherX100 (Zhang\'s Method)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--pattern', type=str, default='6x7',
                       help='Checkerboard pattern inner corners (default: 6x7 for 8.5x10cm printer)')
    parser.add_argument('--square', type=float, default=12.0,
                       help='Square size in mm (default: 12mm - MEASURE YOUR PRINTED SIZE!)')
    args = parser.parse_args()
    
    # Parse pattern size
    pattern_parts = args.pattern.lower().split('x')
    if len(pattern_parts) != 2:
        print("Invalid pattern format. Use format like '8x6'")
        return
    
    pattern_size = (int(pattern_parts[0]), int(pattern_parts[1]))
    
    print("=" * 70)
    print("PincherX100 Camera Calibration (Zhang's Method)")
    print("=" * 70)
    print(f"\nConfiguration:")
    print(f"  Camera Index: {args.camera}")
    print(f"  Pattern Size: {pattern_size[0]}x{pattern_size[1]} inner corners")
    print(f"  Square Size:  {args.square}mm")
    print(f"\nIMPORTANT: Verify square size matches your printed pattern!")
    print(f"    Measure one square with a ruler and adjust --square if needed.")
    
    # Initialize calibration
    calib = CameraCalibration(pattern_size, args.square, args.camera)
    
    # Open camera
    print(f"\nOpening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        return
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Camera opened successfully")
    
    print("\n" + "=" * 70)
    print("Instructions:")
    print("=" * 70)
    print("1. Hold the checkerboard pattern in front of the camera")
    print("2. Move it to different positions and orientations:")
    print("   - Different distances (close and far)")
    print("   - Different angles (tilted left/right, up/down)")
    print("   - Different positions (corners and center of image)")
    print("3. When corners are detected (green lines), press SPACE to capture")
    print("4. Capture 15-20 good images")
    print("5. Press 'C' to calibrate when done")
    print("6. Press 'Q' or ESC to quit")
    print("=" * 70 + "\n")
    
    window_name = 'Camera Calibration - Press SPACE to capture, C to calibrate, Q to quit'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    last_found = False
    last_corners = None
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to read frame")
                break
            
            # Find corners
            found, corners, drawn_frame = calib.find_corners(frame)
            
            # Update state
            if found:
                last_found = True
                last_corners = corners
            else:
                last_found = False
                last_corners = None
            
            # Add status text
            status_color = (0, 255, 0) if found else (0, 0, 255)
            status_text = f"Pattern: {'FOUND' if found else 'NOT FOUND'} | Captured: {calib.captured_count}"
            cv2.putText(drawn_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            instructions = "SPACE: Capture | C: Calibrate | Q: Quit"
            cv2.putText(drawn_frame, instructions, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow(window_name, drawn_frame)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):  # ESC or Q
                break
            
            elif key == ord(' ') and last_found:  # SPACE - capture
                filename = calib.capture_image(frame, last_corners)
                print(f"Captured image {calib.captured_count}: {filename}")
                
                if calib.captured_count >= 15:
                    print(f"Good! You have {calib.captured_count} images. You can calibrate now (press 'C')")
                elif calib.captured_count >= 10:
                    print(f"  Recommended: Capture a few more images (at least 15 total)")
            
            elif key == ord('c'):  # C - calibrate
                if calib.captured_count < 10:
                    print(f"Need at least 10 images. Current: {calib.captured_count}")
                    continue
                
                # Calibrate
                success, message = calib.calibrate()
                
                if success:
                    calib.print_calibration_info()
                    calib.save_calibration()
                    
                    print("\nCalibration complete!")
                    print("\nYou can now use the calibration with:")
                    print("  - Hand-eye calibration: python hand_eye_calibration.py")
                    print("  - Test undistortion: python test_calibration.py")
                else:
                    print(f"{message}")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\nCamera released. Goodbye!")

if __name__ == '__main__':
    main()

