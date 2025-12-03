#!/usr/bin/env python3
"""
Hand-Eye Calibration Tool for PincherX100

This tool performs eye-to-hand calibration to determine the transformation
between the camera coordinate system and the robot base coordinate system.

CALIBRATION PROCEDURE:
1. Print the checkerboard pattern (6x7 tiny US letter)
2. Place checkerboard flat on the platform in front of the robot
3. Move robot gripper to touch specific corners of the checkerboard
4. Capture robot joint positions and checkerboard pose
5. Repeat for 5-10 different positions
6. Calculate transformation matrix

The checkerboard should be placed on the platform (same height as pick area).
"""

import cv2
import numpy as np
import argparse
import os
import json
import sys
import yaml
from datetime import datetime

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'low-level_control'))

from inverse_kinematics import PincherX100IK, create_pose_matrix
from coordinate_transform import CoordinateTransformer


class HandEyeCalibrator:
    """
    Hand-eye calibration for eye-to-hand setup (fixed camera above robot).
    """
    
    def __init__(self, camera_calib_file, robot_config_file, pattern_size=(6, 7), square_size=12.0):
        """
        Initialize hand-eye calibration.
        
        Args:
            camera_calib_file: Path to camera calibration file
            robot_config_file: Path to robot config YAML
            pattern_size: (width, height) of inner corners in checkerboard
            square_size: Size of each square in mm
        """
        # Load camera calibration
        self.transformer = CoordinateTransformer(camera_calib_file, robot_config_file)
        self.camera_matrix = self.transformer.camera_matrix
        self.dist_coeffs = self.transformer.dist_coeffs
        
        # Checkerboard parameters
        self.pattern_size = pattern_size
        self.square_size = square_size / 1000.0  # Convert mm to meters
        
        # Prepare object points (3D points in checkerboard frame)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Storage for calibration data
        self.observations = []
        
        # IK solver for forward kinematics
        self.ik = PincherX100IK()
        
        print("Hand-Eye Calibration initialized")
        print(f"  Pattern: {pattern_size[0]}x{pattern_size[1]} corners")
        print(f"  Square size: {square_size}mm")
    
    def detect_checkerboard_pose(self, image):
        """
        Detect checkerboard and calculate its pose relative to camera.
        
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
                axis_length = min(self.square_size * 1.5 * 1000, distance * 0.25 * 1000)
                # Suppress OpenCV warnings for axes out of frame
                import warnings
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    cv2.drawFrameAxes(drawn_image, self.camera_matrix, self.dist_coeffs,
                                    rvec, tvec, axis_length, 2)
                
                return True, rvec, tvec, drawn_image
        
        return False, None, None, drawn_image
    
    def get_gripper_pose_from_joints(self, joint_positions):
        """
        Calculate gripper pose (4x4 transform) from joint positions.
        
        This is a simplified forward kinematics. For accurate calibration,
        you may need to implement full forward kinematics or use measured poses.
        
        Args:
            joint_positions: [base_pos, shoulder_pos, elbow_pos, wrist_pos] in servo units (0-4095)
        
        Returns:
            T: 4x4 homogeneous transformation matrix (gripper to base)
        """
        # Convert servo positions to angles
        q = self.ik.servo_positions_to_angles(np.array(joint_positions[0:4]))
        
        # For now, use a simplified approach:
        # We'll use the IK solver in reverse by assuming we know the end position
        # In a full implementation, you would implement forward kinematics here
        
        # For calibration purposes, we can use the fact that when the gripper
        # touches the checkerboard, we know the Z coordinate (platform height)
        # and we can estimate X, Y from the base rotation
        
        # Simplified: Use base rotation and approximate reach
        base_angle = q[0]
        
        # Approximate reach based on joint angles
        # This is a placeholder - full FK needed for accurate calibration
        L1 = 0.0445
        L2 = 0.1010
        L3 = 0.1010
        L4 = 0.1090
        
        # Estimate end-effector position (simplified)
        # In practice, you should implement full forward kinematics
        reach = L2 + L3 + L4  # Approximate
        x_approx = reach * np.cos(base_angle) * 0.7  # Rough estimate
        y_approx = reach * np.sin(base_angle) * 0.7
        z_approx = L1 + 0.05  # Platform height + small offset
        
        # Create pose matrix (gripper horizontal)
        T = create_pose_matrix([x_approx, y_approx, z_approx], orientation='horizontal')
        return T
    
    def add_observation(self, joint_positions, rvec_target2cam, tvec_target2cam):
        """
        Add an observation (robot pose + checkerboard detection).
        
        Args:
            joint_positions: [base, shoulder, elbow, wrist, gripper] in servo units
            rvec_target2cam: Rotation vector of checkerboard relative to camera
            tvec_target2cam: Translation vector of checkerboard relative to camera
        """
        # Convert rotation vector to rotation matrix
        R_target2cam, _ = cv2.Rodrigues(rvec_target2cam)
        
        # Get gripper pose in base frame
        T_gripper2base = self.get_gripper_pose_from_joints(joint_positions)
        
        # For calibration, we need the transformation from checkerboard to gripper
        # When gripper touches checkerboard corner, we know the relative position
        # This is a simplified approach - full calibration requires precise measurements
        
        observation = {
            'timestamp': datetime.now().isoformat(),
            'joint_positions': joint_positions.tolist() if isinstance(joint_positions, np.ndarray) else joint_positions,
            'R_target2cam': R_target2cam.tolist(),
            't_target2cam': tvec_target2cam.flatten().tolist(),
            'T_gripper2base': T_gripper2base.tolist()
        }
        
        self.observations.append(observation)
        print(f"Added observation #{len(self.observations)}")
    
    def calibrate(self):
        """
        Perform hand-eye calibration using collected observations.
        
        For eye-to-hand calibration, we solve:
        T_cam2base = T_gripper2base * T_target2gripper * T_cam2target
        
        Returns:
            (success, transform_matrix, error)
        """
        if len(self.observations) < 3:
            return False, None, "Need at least 3 observations for calibration"
        
        print(f"\nPerforming hand-eye calibration with {len(self.observations)} observations...")
        
        # For simplified calibration, we'll use the known camera position
        # and calculate a refinement transform
        
        # Build transformation matrices
        R_gripper2base_list = []
        t_gripper2base_list = []
        R_target2cam_list = []
        t_target2cam_list = []
        
        for obs in self.observations:
            T_gripper2base = np.array(obs['T_gripper2base'])
            R_gripper2base_list.append(T_gripper2base[0:3, 0:3])
            t_gripper2base_list.append(T_gripper2base[0:3, 3])
            
            R_target2cam = np.array(obs['R_target2cam'])
            t_target2cam = np.array(obs['t_target2cam'])
            R_target2cam_list.append(R_target2cam)
            t_target2cam_list.append(t_target2cam)
        
        # Use OpenCV's calibrateHandEye for eye-to-hand
        try:
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_gripper2base_list,
                t_gripper2base_list,
                R_target2cam_list,
                t_target2cam_list
            )
            
            # Build 4x4 transformation matrix
            T_cam2base = np.eye(4)
            T_cam2base[0:3, 0:3] = R_cam2base
            T_cam2base[0:3, 3] = t_cam2base.flatten()
            
            print("Calibration successful!")
            print(f"Transformation matrix:\n{T_cam2base}")
            
            return True, T_cam2base, None
            
        except Exception as e:
            return False, None, f"Calibration failed: {str(e)}"
    
    def save_calibration(self, output_file, transform_matrix):
        """
        Save calibration result to file.
        
        Args:
            output_file: Path to output JSON file
            transform_matrix: 4x4 transformation matrix
        """
        data = {
            'calibration_date': datetime.now().isoformat(),
            'num_observations': len(self.observations),
            'pattern_size': list(self.pattern_size),
            'square_size_mm': self.square_size * 1000,
            'transform_matrix': transform_matrix.tolist(),
            'observations': self.observations
        }
        
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\nCalibration saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Hand-Eye Calibration for PincherX100')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--calib', type=str, 
                       default='../../camera_calibration/camera_calibration.npz',
                       help='Camera calibration file')
    parser.add_argument('--config', type=str,
                       default='../configs/robot_config.yaml',
                       help='Robot config file')
    parser.add_argument('--pattern', type=str, default='6x7',
                       help='Checkerboard pattern (default: 6x7)')
    parser.add_argument('--square', type=float, default=24.0,
                       help='Square size in mm (default: 24mm for large pattern)')
    parser.add_argument('--load', type=str, default=None,
                       help='Load existing observations from file')
    parser.add_argument('--output', type=str, default='hand_eye_calibration.json',
                       help='Output file for calibration result')
    
    args = parser.parse_args()
    
    # Parse pattern size
    pattern_parts = args.pattern.lower().split('x')
    pattern_size = (int(pattern_parts[0]), int(pattern_parts[1]))
    
    print("=" * 70)
    print("PincherX100 Hand-Eye Calibration")
    print("=" * 70)
    
    # Initialize calibrator
    try:
        calibrator = HandEyeCalibrator(
            args.calib,
            args.config,
            pattern_size,
            args.square
        )
    except Exception as e:
        print(f"Error initializing calibrator: {e}")
        return
    
    # Load existing observations if provided
    if args.load:
        with open(args.load, 'r') as f:
            data = json.load(f)
            calibrator.observations = data.get('observations', [])
            print(f"Loaded {len(calibrator.observations)} observations from {args.load}")
    
    # Open camera
    print(f"\nOpening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("\n" + "=" * 70)
    print("CALIBRATION INSTRUCTIONS:")
    print("=" * 70)
    print("1. Place checkerboard flat on platform in front of robot")
    print("2. Use low-level control to move robot gripper to touch a corner")
    print("3. When checkerboard is detected, press SPACE to capture")
    print("4. Enter joint positions when prompted:")
    print("   Format: base,shoulder,elbow,wrist,gripper")
    print("   Example: 2048,2048,2048,2048,2048")
    print("5. Move to different positions (5-10 positions recommended)")
    print("6. Press 'C' to calibrate and save")
    print("7. Press 'Q' to quit")
    print("=" * 70 + "\n")
    
    window_name = 'Hand-Eye Calibration'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to read frame")
                break
            
            # Detect checkerboard
            success, rvec, tvec, drawn_frame = calibrator.detect_checkerboard_pose(frame)
            
            # Add status
            status_text = f"Observations: {len(calibrator.observations)} | Pattern: {'FOUND' if success else 'NOT FOUND'}"
            cv2.putText(drawn_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if success else (0, 0, 255), 2)
            
            instructions = "SPACE: Capture | C: Calibrate | Q: Quit"
            cv2.putText(drawn_frame, instructions, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow(window_name, drawn_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):
                break
            elif key == ord(' ') and success:
                # Prompt for joint positions
                print("\nEnter joint positions (base,shoulder,elbow,wrist,gripper):")
                print("Or press Enter to use current positions from arm controller")
                user_input = input().strip()
                
                if user_input:
                    try:
                        positions = [int(x.strip()) for x in user_input.split(',')]
                        if len(positions) != 5:
                            print("Error: Need 5 joint positions")
                            continue
                    except ValueError:
                        print("Error: Invalid input format")
                        continue
                else:
                    # TODO: Read from arm controller
                    print("Reading from arm controller not yet implemented")
                    print("Please enter positions manually")
                    continue
                
                calibrator.add_observation(positions, rvec, tvec)
                
            elif key == ord('c'):
                if len(calibrator.observations) < 3:
                    print("Need at least 3 observations")
                else:
                    success, transform, error = calibrator.calibrate()
                    if success:
                        calibrator.save_calibration(args.output, transform)
                        print(f"\nCalibration complete! Result saved to {args.output}")
                    else:
                        print(f"\nCalibration failed: {error}")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        
        if len(calibrator.observations) > 0:
            # Save observations
            obs_file = args.output.replace('.json', '_observations.json')
            with open(obs_file, 'w') as f:
                json.dump({'observations': calibrator.observations}, f, indent=2)
            print(f"\nObservations saved to: {obs_file}")


if __name__ == '__main__':
    main()

