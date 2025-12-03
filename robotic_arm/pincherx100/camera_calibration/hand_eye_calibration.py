#!/usr/bin/env python3
"""
Hand-Eye Calibration for PincherX100 + Camera

This script performs hand-eye calibration to determine the transformation
between the camera coordinate system and the robot base coordinate system.

Two approaches are supported:
1. Eye-in-Hand: Camera mounted on robot end-effector
2. Eye-to-Hand: Camera mounted above/beside robot (fixed)

For PincherX100 with camera mounted above (Eye-to-Hand), we will:
1. Move robot to multiple known positions
2. Detect checkerboard in each position
3. Calculate transformation from camera to robot base

Usage:
    python hand_eye_calibration.py [--camera 0] [--calib camera_calibration.npz]

Prerequisites:
    - Camera calibration completed (camera_calibration.npz exists)
    - Robot can be controlled (via control_arm.py or ROS2)
    - Checkerboard pattern available
"""

import cv2
import numpy as np
import argparse
import os
import json
import sys
from datetime import datetime

# Add parent directory to path for importing arm control
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'low-level_control'))

class HandEyeCalibration:
    def __init__(self, camera_calib_file, pattern_size=(8, 6), square_size=25.0):
        """
        Initialize hand-eye calibration
        
        Args:
            camera_calib_file: Path to camera calibration file
            pattern_size: (width, height) of inner corners in checkerboard
            square_size: Size of each square in mm
        """
        # Load camera calibration
        if not os.path.exists(camera_calib_file):
            raise FileNotFoundError(f"Camera calibration file not found: {camera_calib_file}")
        
        calib_data = np.load(camera_calib_file)
        self.camera_matrix = calib_data['camera_matrix']
        self.dist_coeffs = calib_data['dist_coeffs']
        
        self.pattern_size = pattern_size
        self.square_size = square_size
        
        # Prepare object points (3D points in real world space)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Storage for calibration data
        self.R_gripper2base = []  # Robot gripper to base transformations
        self.t_gripper2base = []
        self.R_target2cam = []    # Checkerboard to camera transformations
        self.t_target2cam = []
        
        self.robot_poses = []     # Robot joint positions
        
        print("Hand-Eye Calibration initialized")
        print(f"  Pattern: {pattern_size[0]}x{pattern_size[1]} corners")
        print(f"  Square size: {square_size}mm")
    
    def detect_checkerboard_pose(self, image):
        """
        Detect checkerboard and calculate its pose relative to camera
        
        Returns:
            (success, rvec, tvec, drawn_image)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
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
            
            # Calculate pose
            success, rvec, tvec = cv2.solvePnP(
                self.objp,
                corners_refined,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            if success:
                # Draw axis with adaptive length to fit in frame
                distance = np.linalg.norm(tvec)
                axis_length = min(self.square_size * 1.5, distance * 0.25)
                # Suppress OpenCV warnings for axes out of frame
                import warnings
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    cv2.drawFrameAxes(drawn_image, self.camera_matrix, self.dist_coeffs,
                                    rvec, tvec, axis_length, 2)
                
                return True, rvec, tvec, drawn_image
        
        return False, None, None, drawn_image
    
    def add_observation(self, robot_pose, rvec_target2cam, tvec_target2cam):
        """
        Add an observation (robot pose + checkerboard detection)
        
        Args:
            robot_pose: Dictionary with robot joint positions and/or end-effector pose
            rvec_target2cam: Rotation vector of checkerboard relative to camera
            tvec_target2cam: Translation vector of checkerboard relative to camera
        """
        # Convert rotation vector to rotation matrix
        R_target2cam, _ = cv2.Rodrigues(rvec_target2cam)
        
        self.R_target2cam.append(R_target2cam)
        self.t_target2cam.append(tvec_target2cam)
        
        # For now, we'll use a simplified approach
        # In a full implementation, you would calculate the actual gripper pose
        # using forward kinematics from the joint positions
        
        # Placeholder: Store robot pose data
        self.robot_poses.append(robot_pose)
        
        print(f"Added observation #{len(self.robot_poses)}")
        print(f"  Checkerboard position (camera frame): {tvec_target2cam.flatten()}")
    
    def calibrate_eye_to_hand(self):
        """
        Perform eye-to-hand calibration
        
        For eye-to-hand (fixed camera above robot), we calculate:
        - Transformation from camera to robot base
        
        Note: This is a simplified version. Full implementation requires
        robot forward kinematics to get actual gripper poses.
        """
        if len(self.robot_poses) < 3:
            return False, "Need at least 3 observations for calibration"
        
        print("\n" + "=" * 70)
        print("Performing Hand-Eye Calibration...")
        print("=" * 70)
        print(f"Number of observations: {len(self.robot_poses)}")
        
        # TODO: Implement full hand-eye calibration using cv2.calibrateHandEye()
        # This requires proper robot forward kinematics implementation
        
        print("\nWARNING: Full hand-eye calibration requires robot forward kinematics")
        print("This feature is under development.")
        print("\nFor now, you can:")
        print("1. Use the camera calibration for undistortion")
        print("2. Detect objects in camera frame")
        print("3. Manually measure camera-to-base transformation")
        
        return False, "Full implementation coming soon"
    
    def save_observations(self, filename='hand_eye_observations.json'):
        """Save observations for later use"""
        data = {
            'num_observations': len(self.robot_poses),
            'robot_poses': self.robot_poses,
            'pattern_size': list(self.pattern_size),
            'square_size': float(self.square_size),
            'capture_date': datetime.now().isoformat()
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\nObservations saved to: {filename}")

def main():
    parser = argparse.ArgumentParser(description='Hand-Eye Calibration for PincherX100')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--calib', type=str, default='camera_calibration.npz',
                       help='Camera calibration file (default: camera_calibration.npz)')
    parser.add_argument('--pattern', type=str, default='6x7',
                       help='Checkerboard pattern inner corners (default: 6x7)')
    parser.add_argument('--square', type=float, default=24.0,
                       help='Square size in mm (default: 24mm for large pattern - use YOUR measured size)')
    args = parser.parse_args()
    
    # Parse pattern size
    pattern_parts = args.pattern.lower().split('x')
    pattern_size = (int(pattern_parts[0]), int(pattern_parts[1]))
    
    print("=" * 70)
    print("PincherX100 Hand-Eye Calibration")
    print("=" * 70)
    print("\nNOTE: This is a simplified calibration tool")
    print("Full hand-eye calibration requires robot forward kinematics")
    print("\nThis tool will help you:")
    print("  1. Visualize checkerboard detection in camera frame")
    print("  2. Record robot positions for later calibration")
    print("  3. Save observations for offline processing")
    
    # Initialize calibration
    try:
        calib = HandEyeCalibration(args.calib, pattern_size, args.square)
    except FileNotFoundError as e:
        print(f"\n{e}")
        print("Please run camera calibration first: python calibrate_camera.py")
        return
    
    # Open camera
    print(f"\nOpening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Camera opened")
    
    print("\n" + "=" * 70)
    print("Instructions:")
    print("=" * 70)
    print("1. Place checkerboard in robot workspace (visible to camera)")
    print("2. Move robot to a position where it touches the checkerboard")
    print("3. When checkerboard is detected, press SPACE to record")
    print("4. Move robot to different positions (5-10 positions)")
    print("5. Press 'S' to save observations")
    print("6. Press 'Q' or ESC to quit")
    print("=" * 70 + "\n")
    
    window_name = 'Hand-Eye Calibration - SPACE: capture, S: save, Q: quit'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    observation_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to read frame")
                break
            
            # Detect checkerboard pose
            success, rvec, tvec, drawn_frame = calib.detect_checkerboard_pose(frame)
            
            # Add status text
            if success:
                status_color = (0, 255, 0)
                status_text = f"Pattern: FOUND | Observations: {observation_count}"
                
                # Display translation
                tx, ty, tz = tvec.flatten()
                pos_text = f"Position: X={tx:.1f} Y={ty:.1f} Z={tz:.1f} mm"
                cv2.putText(drawn_frame, pos_text, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                status_color = (0, 0, 255)
                status_text = f"Pattern: NOT FOUND | Observations: {observation_count}"
            
            cv2.putText(drawn_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            instructions = "SPACE: Capture | S: Save | Q: Quit"
            cv2.putText(drawn_frame, instructions, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow(window_name, drawn_frame)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):  # ESC or Q
                break
            
            elif key == ord(' ') and success:  # SPACE - capture
                # Get robot position (placeholder - you'll need to implement this)
                # For example, read from control_arm.py or ROS2
                robot_pose = {
                    'timestamp': datetime.now().isoformat(),
                    'note': 'Manual capture - add robot joint positions here'
                }
                
                calib.add_observation(robot_pose, rvec, tvec)
                observation_count += 1
                
                print(f"Captured observation {observation_count}")
                if observation_count >= 5:
                    print(f"  Good! You can save now (press 'S')")
            
            elif key == ord('s'):  # S - save
                if observation_count == 0:
                    print("No observations to save")
                else:
                    calib.save_observations()
                    print(f"Saved {observation_count} observations")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\nHand-Eye calibration tool closed")
        
        if observation_count > 0:
            print(f"\nYou captured {observation_count} observations.")
            print("To complete full calibration, you'll need to:")
            print("1. Implement forward kinematics for PincherX100")
            print("2. Record actual gripper poses during capture")
            print("3. Use cv2.calibrateHandEye() with proper transformations")

if __name__ == '__main__':
    main()

