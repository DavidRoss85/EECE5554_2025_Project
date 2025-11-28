#!/usr/bin/env python3
"""
Test camera calibration by showing undistorted video

Usage:
    python test_calibration.py [--camera 0] [--calib camera_calibration.npz]
"""

import cv2
import numpy as np
import argparse
import os

def main():
    parser = argparse.ArgumentParser(description='Test camera calibration')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--calib', type=str, default='camera_calibration.npz',
                       help='Calibration file (default: camera_calibration.npz)')
    args = parser.parse_args()
    
    # Load calibration
    if not os.path.exists(args.calib):
        print(f"Calibration file not found: {args.calib}")
        print("Please run camera calibration first: python calibrate_camera.py")
        return
    
    print("=" * 70)
    print("Testing Camera Calibration")
    print("=" * 70)
    print(f"\nLoading calibration from: {args.calib}")
    
    calib_data = np.load(args.calib)
    camera_matrix = calib_data['camera_matrix']
    dist_coeffs = calib_data['dist_coeffs']
    image_size = tuple(calib_data['image_size'])
    
    print("Calibration loaded")
    print(f"  Image size: {image_size[0]} x {image_size[1]}")
    print(f"  Reprojection error: {calib_data['calibration_error']:.4f} pixels")
    
    # Open camera
    print(f"\nOpening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        return
    
    print("Camera opened")
    
    # Get optimal new camera matrix
    h, w = image_size[1], image_size[0]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    
    print("\n" + "=" * 70)
    print("Press Q or ESC to quit")
    print("=" * 70 + "\n")
    
    # Create windows
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Undistorted', cv2.WINDOW_NORMAL)
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to read frame")
                break
            
            # Undistort image
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
            
            # Crop the image (optional)
            x, y, w, h = roi
            if w > 0 and h > 0:
                undistorted_cropped = undistorted[y:y+h, x:x+w]
            else:
                undistorted_cropped = undistorted
            
            # Add text
            cv2.putText(frame, "Original (Distorted)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(undistorted, "Undistorted", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show images
            cv2.imshow('Original', frame)
            cv2.imshow('Undistorted', undistorted)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):  # ESC or Q
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Done!")

if __name__ == '__main__':
    main()

