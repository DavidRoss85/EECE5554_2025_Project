#!/usr/bin/env python3
"""
Improved Hand-Eye Calibration Tool

This version solves the occlusion problem by:
1. Detecting checkerboard FIRST (with gripper out of view)
2. Then moving gripper to touch/near checkerboard
3. Recording joint positions
4. Associating checkerboard pose with gripper pose

Usage:
    python hand_eye_calibration_improved.py [--camera 0] [--calib camera_calibration.npz]
"""

import cv2
import numpy as np
import argparse
import os
import json
import sys
import time
from datetime import datetime

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'low-level_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pick_place_system', 'vision'))

try:
    from control_arm import ArmController, ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER
    ARM_AVAILABLE = True
except ImportError:
    ARM_AVAILABLE = False
    print("Warning: Arm controller not available. Joint positions must be entered manually.")

from forward_kinematics import PincherX100FK


class ImprovedHandEyeCalibration:
    """
    Improved hand-eye calibration that handles gripper occlusion.
    """
    
    def __init__(self, camera_calib_file, pattern_size=(6, 7), square_size=24.0):
        """
        Initialize calibration.
        
        Args:
            camera_calib_file: Path to camera calibration file
            pattern_size: (width, height) of inner corners
            square_size: Size of each square in mm
        """
        # Load camera calibration
        if not os.path.exists(camera_calib_file):
            raise FileNotFoundError(f"Camera calibration file not found: {camera_calib_file}")
        
        calib_data = np.load(camera_calib_file)
        self.camera_matrix = calib_data['camera_matrix']
        self.dist_coeffs = calib_data['dist_coeffs']
        
        # Checkerboard parameters
        self.pattern_size = pattern_size
        self.square_size = square_size / 1000.0  # Convert to meters
        
        # Prepare object points
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Storage
        self.observations = []
        self.current_checkerboard_pose = None  # Store detected pose
        
        # Forward kinematics
        self.fk = PincherX100FK()
        
        # Arm controller (if available)
        self.arm_controller = None
        if ARM_AVAILABLE:
            try:
                self.arm_controller = ArmController()
                if self.arm_controller.initialize():
                    print("Arm controller initialized")
                else:
                    print("Warning: Arm controller failed to initialize")
                    self.arm_controller = None
            except Exception as e:
                print(f"Warning: Could not initialize arm controller: {e}")
                self.arm_controller = None
        
        print("Improved Hand-Eye Calibration initialized")
        print(f"  Pattern: {pattern_size[0]}x{pattern_size[1]} corners")
        print(f"  Square size: {square_size}mm")
        print(f"  Arm controller: {'Available' if self.arm_controller else 'Not available'}")
    
    def detect_checkerboard_pose(self, image):
        """
        Detect checkerboard and calculate pose.
        
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
            # Refine corners
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
    
    def capture_checkerboard_pose(self, rvec, tvec):
        """
        Store checkerboard pose for later use.
        """
        R_target2cam, _ = cv2.Rodrigues(rvec)
        self.current_checkerboard_pose = {
            'rvec': rvec,
            'tvec': tvec,
            'R_target2cam': R_target2cam,
            't_target2cam': tvec.flatten()
        }
        print("Checkerboard pose captured and stored")
    
    def capture_gripper_pose(self, joint_positions=None):
        """
        Capture gripper pose and associate with stored checkerboard pose.
        
        Args:
            joint_positions: [base, shoulder, elbow, wrist] in servo units.
                           If None, reads from arm controller.
        """
        if self.current_checkerboard_pose is None:
            print("Error: No checkerboard pose stored. Detect checkerboard first (press 'D')")
            return False
        
        # Get joint positions
        if joint_positions is None:
            if self.arm_controller:
                positions = []
                for servo_id in [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]:
                    pos = self.arm_controller.get_present_position(servo_id)
                    if pos is None:
                        print(f"Error: Could not read position from servo {servo_id}")
                        return False
                    positions.append(pos)
                joint_positions = positions
            else:
                print("Error: No arm controller and no joint positions provided")
                return False
        
        # Calculate gripper pose using forward kinematics
        try:
            T_gripper2base = self.fk.solve_from_servo_positions(np.array(joint_positions))
            R_gripper2base = T_gripper2base[0:3, 0:3]
            t_gripper2base = T_gripper2base[0:3, 3]
        except Exception as e:
            print(f"Error calculating forward kinematics: {e}")
            return False
        
        # Create observation
        observation = {
            'timestamp': datetime.now().isoformat(),
            'joint_positions': joint_positions,
            'R_gripper2base': R_gripper2base.tolist(),
            't_gripper2base': t_gripper2base.tolist(),
            'R_target2cam': self.current_checkerboard_pose['R_target2cam'].tolist(),
            't_target2cam': self.current_checkerboard_pose['t_target2cam'].tolist(),
            'rvec': self.current_checkerboard_pose['rvec'].flatten().tolist(),
            'tvec': self.current_checkerboard_pose['tvec'].flatten().tolist()
        }
        
        self.observations.append(observation)
        print(f"Observation #{len(self.observations)} captured")
        print(f"  Joint positions: {joint_positions}")
        print(f"  Gripper position: ({t_gripper2base[0]:.3f}, {t_gripper2base[1]:.3f}, {t_gripper2base[2]:.3f})")
        
        # Clear stored checkerboard pose (force re-detection for next observation)
        self.current_checkerboard_pose = None
        
        return True
    
    def calibrate(self):
        """
        Perform hand-eye calibration using collected observations.
        """
        if len(self.observations) < 3:
            return False, None, f"Need at least 3 observations, have {len(self.observations)}"
        
        print(f"\nPerforming calibration with {len(self.observations)} observations...")
        
        # Prepare data for cv2.calibrateHandEye
        R_gripper2base_list = []
        t_gripper2base_list = []
        R_target2cam_list = []
        t_target2cam_list = []
        
        for obs in self.observations:
            R_gripper2base_list.append(np.array(obs['R_gripper2base']))
            t_gripper2base_list.append(np.array(obs['t_gripper2base']))
            R_target2cam_list.append(np.array(obs['R_target2cam']))
            t_target2cam_list.append(np.array(obs['t_target2cam']))
        
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
            
            return True, T_cam2base, None
            
        except Exception as e:
            return False, None, f"Calibration failed: {str(e)}"
    
    def save_calibration(self, output_file, transform_matrix):
        """Save calibration result."""
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
    parser = argparse.ArgumentParser(description='Improved Hand-Eye Calibration')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--calib', type=str, default='camera_calibration.npz',
                       help='Camera calibration file')
    parser.add_argument('--pattern', type=str, default='6x7',
                       help='Checkerboard pattern (default: 6x7)')
    parser.add_argument('--square', type=float, default=24.0,
                       help='Square size in mm (default: 24mm)')
    parser.add_argument('--output', type=str, default='hand_eye_calibration.json',
                       help='Output file')
    
    args = parser.parse_args()
    
    # Parse pattern size
    pattern_parts = args.pattern.lower().split('x')
    pattern_size = (int(pattern_parts[0]), int(pattern_parts[1]))
    
    print("=" * 70)
    print("Improved Hand-Eye Calibration")
    print("=" * 70)
    print("\nThis tool solves the occlusion problem by:")
    print("1. Detecting checkerboard FIRST (gripper out of view)")
    print("2. Storing checkerboard pose")
    print("3. Moving gripper to touch/near checkerboard")
    print("4. Capturing joint positions")
    print("5. Associating the two")
    
    # Initialize
    try:
        calib = ImprovedHandEyeCalibration(args.calib, pattern_size, args.square)
    except Exception as e:
        print(f"Error: {e}")
        return
    
    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera}")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("\n" + "=" * 70)
    print("CALIBRATION WORKFLOW:")
    print("=" * 70)
    print("1. Move gripper OUT OF VIEW (so checkerboard is visible)")
    print("2. Press 'D' to DETECT and store checkerboard pose")
    print("3. Move gripper to touch/near the checkerboard corner")
    print("4. Press 'C' to CAPTURE gripper pose (reads from arm or prompts)")
    print("5. Repeat steps 1-4 for 5-10 different positions")
    print("6. Press 'K' to CALIBRATE and save")
    print("7. Press 'Q' to quit")
    print("=" * 70 + "\n")
    
    window_name = 'Improved Hand-Eye Calibration'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect checkerboard (for display)
            success, rvec, tvec, drawn_frame = calib.detect_checkerboard_pose(frame)
            
            # Status display
            status_parts = []
            status_parts.append(f"Observations: {len(calib.observations)}")
            if calib.current_checkerboard_pose:
                status_parts.append("Checkerboard: STORED")
            elif success:
                status_parts.append("Checkerboard: DETECTED (press 'D' to store)")
            else:
                status_parts.append("Checkerboard: NOT FOUND")
            
            status_text = " | ".join(status_parts)
            color = (0, 255, 0) if (success or calib.current_checkerboard_pose) else (0, 0, 255)
            cv2.putText(drawn_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            instructions = "D: Detect & Store | C: Capture Gripper | K: Calibrate | Q: Quit"
            cv2.putText(drawn_frame, instructions, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if calib.current_checkerboard_pose:
                cv2.putText(drawn_frame, "READY: Move gripper and press 'C'", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow(window_name, drawn_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):
                break
            
            elif key == ord('d') and success:  # D - Detect and store
                calib.capture_checkerboard_pose(rvec, tvec)
            
            elif key == ord('c'):  # C - Capture gripper pose
                if not calib.current_checkerboard_pose:
                    print("Error: No checkerboard pose stored. Press 'D' first.")
                    continue
                
                # Try to read from arm controller, or prompt
                if calib.arm_controller:
                    success = calib.capture_gripper_pose()
                    if not success:
                        print("Failed to read from arm controller")
                else:
                    # Manual entry
                    print("\nEnter joint positions (base,shoulder,elbow,wrist):")
                    user_input = input("Positions: ").strip()
                    if user_input:
                        try:
                            positions = [int(x.strip()) for x in user_input.split(',')]
                            if len(positions) == 4:
                                calib.capture_gripper_pose(positions)
                            else:
                                print("Error: Need 4 values")
                        except ValueError:
                            print("Error: Invalid format")
            
            elif key == ord('k'):  # K - Calibrate
                if len(calib.observations) < 3:
                    print(f"Error: Need at least 3 observations, have {len(calib.observations)}")
                else:
                    success, transform, error = calib.calibrate()
                    if success:
                        calib.save_calibration(args.output, transform)
                        print(f"\nCalibration complete! Saved to {args.output}")
                        print(f"Transformation matrix:\n{transform}")
                    else:
                        print(f"\nCalibration failed: {error}")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if calib.arm_controller:
            calib.arm_controller.shutdown()
        
        if len(calib.observations) > 0:
            # Save observations
            obs_file = args.output.replace('.json', '_observations.json')
            with open(obs_file, 'w') as f:
                json.dump({'observations': calib.observations}, f, indent=2)
            print(f"\nObservations saved to: {obs_file}")


if __name__ == '__main__':
    main()

