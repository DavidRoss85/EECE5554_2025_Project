#!/usr/bin/env python3
"""
Hand-Eye Calibration - Touch Method (Approach 2)

This tool implements the touch method where:
1. Gripper center (TCP) touches a specific corner of the checkerboard
2. Checkerboard pose is detected (before gripper moves in)
3. Gripper pose is captured when touching the corner
4. The relationship between gripper TCP and checkerboard corner is used

Usage:
    python hand_eye_calibration_touch_method.py [--camera 0] [--calib camera_calibration.npz]
"""

import cv2
import numpy as np
import argparse
import os
import json
import sys
import time
import io
from datetime import datetime
from contextlib import redirect_stderr

# Suppress OpenCV warnings
# OpenCV 4.x prints warnings directly from C++ code
# We'll use stderr redirection at the file descriptor level in drawFrameAxes calls
# Also set environment variable to reduce OpenCV verbosity
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'

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


class TouchMethodCalibration:
    """
    Hand-eye calibration using touch method (gripper center touches checkerboard corner).
    """
    
    def __init__(self, camera_calib_file, pattern_size=(7, 6), square_size=24.0):
        """
        Initialize calibration.
        
        Args:
            camera_calib_file: Path to camera calibration file
            pattern_size: (width, height) of inner corners
                         width = number of inner corners horizontally (columns)
                         height = number of inner corners vertically (rows)
                         Default: (7, 6) = 7 columns x 6 rows
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
        
        # Prepare object points (3D points in checkerboard frame)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Storage
        self.observations = []
        self.current_checkerboard_pose = None
        self.current_corner_index = None  # Which corner is being touched
        
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
        
        print("Touch Method Calibration initialized")
        print(f"  Pattern: {pattern_size[0]}x{pattern_size[1]} corners")
        print(f"  Square size: {square_size}mm")
        print(f"  Arm controller: {'Available' if self.arm_controller else 'Not available'}")
    
    def detect_checkerboard_pose(self, image):
        """
        Detect checkerboard and calculate pose.
        
        Returns:
            (success, rvec, tvec, corners, drawn_image)
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
                # Optionally draw coordinate axes from the center of the checkerboard
                # (Axes are drawn from origin by default, which is at a corner)
                # Set draw_axes=False to disable axes visualization
                draw_axes = False  # Set to True to enable axes visualization
                
                if draw_axes:
                    # Calculate checkerboard center in 3D (in checkerboard coordinate frame)
                    center_3d = np.array([
                        (self.pattern_size[0] - 1) * self.square_size / 2.0,
                        (self.pattern_size[1] - 1) * self.square_size / 2.0,
                        0.0
                    ])
                    
                    # Transform center to camera frame
                    R, _ = cv2.Rodrigues(rvec)
                    center_camera = R @ center_3d.reshape(3, 1) + tvec.reshape(3, 1)
                    
                    # Draw axes from center instead of corner
                    # Calculate appropriate axis length
                    distance = np.linalg.norm(center_camera)
                    axis_length = min(self.square_size * 2.0 * 1000, distance * 0.3 * 1000)
                    
                    # Suppress OpenCV warnings for axes out of frame
                    if os.name != 'nt':  # Unix-like systems (Linux, macOS)
                        try:
                            original_stderr_fd = sys.stderr.fileno()
                            devnull_fd = os.open(os.devnull, os.O_WRONLY)
                            saved_stderr_fd = os.dup(original_stderr_fd)
                            os.dup2(devnull_fd, original_stderr_fd)
                            try:
                                # Draw axes from center (use same rotation, but center translation)
                                cv2.drawFrameAxes(drawn_image, self.camera_matrix, self.dist_coeffs,
                                                rvec, center_camera.flatten(), axis_length, 2)
                            finally:
                                os.dup2(saved_stderr_fd, original_stderr_fd)
                                os.close(saved_stderr_fd)
                                os.close(devnull_fd)
                        except (OSError, AttributeError, ValueError):
                            cv2.drawFrameAxes(drawn_image, self.camera_matrix, self.dist_coeffs,
                                            rvec, center_camera.flatten(), axis_length, 2)
                    else:
                        f = io.StringIO()
                        with redirect_stderr(f):
                            cv2.drawFrameAxes(drawn_image, self.camera_matrix, self.dist_coeffs,
                                            rvec, center_camera.flatten(), axis_length, 2)
                
                # Draw corner numbers for selection
                # Use detected corner positions directly (solvePnP ensures objp[i] matches corners_refined[i])
                # This ensures labels appear at the actual detected corner locations
                for i, corner_2d in enumerate(corners_refined):
                    px, py = int(corner_2d[0][0]), int(corner_2d[0][1])
                    if 0 <= px < image.shape[1] and 0 <= py < image.shape[0]:
                        # Draw circle at corner (magenta/purple)
                        cv2.circle(drawn_image, (px, py), 6, (255, 0, 255), 2)
                        # Draw corner number label with background for visibility
                        label = str(i)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.5
                        thickness = 1
                        (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
                        # Draw background rectangle
                        cv2.rectangle(drawn_image, 
                                     (px + 8, py - text_height - 8),
                                     (px + 8 + text_width, py + 2),
                                     (0, 0, 0), -1)
                        # Draw text
                        cv2.putText(drawn_image, label, (px + 8, py - 8), 
                                   font, font_scale, (255, 0, 255), thickness)
                
                return True, rvec, tvec, corners_refined, drawn_image
        
        return False, None, None, None, drawn_image
    
    def get_corner_3d_position(self, corner_index, rvec, tvec):
        """
        Get 3D position of a specific corner in camera frame.
        
        Args:
            corner_index: Index of corner (0 to pattern_size[0]*pattern_size[1]-1)
            rvec: Rotation vector of checkerboard
            tvec: Translation vector of checkerboard
        
        Returns:
            corner_3d_cam: 3D position in camera frame (meters)
        """
        if corner_index < 0 or corner_index >= len(self.objp):
            return None
        
        # Get corner position in checkerboard frame
        corner_obj = self.objp[corner_index]
        
        # Transform to camera frame
        R, _ = cv2.Rodrigues(rvec)
        corner_3d_cam = R @ corner_obj + tvec.flatten()
        
        return corner_3d_cam
    
    def capture_checkerboard_pose(self, rvec, tvec, corners):
        """
        Store checkerboard pose and allow corner selection.
        
        Returns:
            corner_index: Index of selected corner, or None if cancelled
        """
        R_target2cam, _ = cv2.Rodrigues(rvec)
        self.current_checkerboard_pose = {
            'rvec': rvec,
            'tvec': tvec,
            'R_target2cam': R_target2cam,
            't_target2cam': tvec.flatten(),
            'corners': corners
        }
        print("Checkerboard pose captured")
        print(f"  Available corners: 0 to {len(self.objp)-1}")
        print(f"  Corner layout: {self.pattern_size[0]} columns x {self.pattern_size[1]} rows")
        return True
    
    def select_corner(self, corner_index):
        """
        Select which corner the gripper will touch.
        
        Args:
            corner_index: Index of corner (0-based)
        """
        if corner_index < 0 or corner_index >= len(self.objp):
            print(f"Error: Invalid corner index. Must be 0 to {len(self.objp)-1}")
            return False
        
        self.current_corner_index = corner_index
        
        # Get corner position in checkerboard frame
        corner_obj = self.objp[corner_index]
        col = corner_index % self.pattern_size[0]
        row = corner_index // self.pattern_size[0]
        
        print(f"Selected corner {corner_index}:")
        print(f"  Position in pattern: Column {col}, Row {row}")
        print(f"  Position in checkerboard frame: ({corner_obj[0]*1000:.1f}mm, {corner_obj[1]*1000:.1f}mm)")
        return True
    
    def capture_gripper_pose(self, joint_positions=None):
        """
        Capture gripper pose when touching the selected corner.
        
        Args:
            joint_positions: [base, shoulder, elbow, wrist] in servo units.
                           If None, reads from arm controller.
        """
        if self.current_checkerboard_pose is None:
            print("Error: No checkerboard pose stored. Detect checkerboard first (press 'D')")
            return False
        
        if self.current_corner_index is None:
            print("Error: No corner selected. Select corner first (press 'S' then enter corner number)")
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
        
        # Calculate gripper TCP pose using forward kinematics
        try:
            T_gripper2base = self.fk.solve_from_servo_positions(np.array(joint_positions))
            R_gripper2base = T_gripper2base[0:3, 0:3]
            t_gripper2base = T_gripper2base[0:3, 3]  # Gripper TCP position in base frame
        except Exception as e:
            print(f"Error calculating forward kinematics: {e}")
            return False
        
        # Get corner position in camera frame
        corner_3d_cam = self.get_corner_3d_position(
            self.current_corner_index,
            self.current_checkerboard_pose['rvec'],
            self.current_checkerboard_pose['tvec']
        )
        
        # Create observation
        observation = {
            'timestamp': datetime.now().isoformat(),
            'joint_positions': joint_positions,
            'R_gripper2base': R_gripper2base.tolist(),
            't_gripper2base': t_gripper2base.tolist(),  # Gripper TCP in base frame
            'R_target2cam': self.current_checkerboard_pose['R_target2cam'].tolist(),
            't_target2cam': self.current_checkerboard_pose['t_target2cam'].tolist(),
            'corner_index': self.current_corner_index,
            'corner_3d_cam': corner_3d_cam.tolist() if corner_3d_cam is not None else None,
            'rvec': self.current_checkerboard_pose['rvec'].flatten().tolist(),
            'tvec': self.current_checkerboard_pose['tvec'].flatten().tolist()
        }
        
        self.observations.append(observation)
        print(f"\nObservation #{len(self.observations)} captured:")
        print(f"  Corner index: {self.current_corner_index}")
        print(f"  Joint positions: {joint_positions}")
        print(f"  Gripper TCP position (base frame): ({t_gripper2base[0]:.4f}, {t_gripper2base[1]:.4f}, {t_gripper2base[2]:.4f}) m")
        if corner_3d_cam is not None:
            print(f"  Corner position (camera frame): ({corner_3d_cam[0]:.4f}, {corner_3d_cam[1]:.4f}, {corner_3d_cam[2]:.4f}) m")
        
        # Clear for next observation
        self.current_checkerboard_pose = None
        self.current_corner_index = None
        
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
            'calibration_method': 'touch_method',
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
    parser = argparse.ArgumentParser(description='Hand-Eye Calibration - Touch Method')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--calib', type=str, default='camera_calibration.npz',
                       help='Camera calibration file')
    parser.add_argument('--pattern', type=str, default='7x6',
                       help='Checkerboard pattern as WIDTHxHEIGHT inner corners (default: 7x6)')
    parser.add_argument('--square', type=float, default=24.0,
                       help='Square size in mm (default: 24mm)')
    parser.add_argument('--output', type=str, default='hand_eye_calibration.json',
                       help='Output file')
    
    args = parser.parse_args()
    
    # Parse pattern size
    pattern_parts = args.pattern.lower().split('x')
    pattern_size = (int(pattern_parts[0]), int(pattern_parts[1]))
    
    print("=" * 70)
    print("Hand-Eye Calibration - Touch Method (Approach 2)")
    print("=" * 70)
    print("\nThis method uses gripper center (TCP) touching checkerboard corners.")
    print("The gripper center is the point between the two gripper fingers.")
    
    # Initialize
    try:
        calib = TouchMethodCalibration(args.calib, pattern_size, args.square)
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
    print("CALIBRATION WORKFLOW - TOUCH METHOD:")
    print("=" * 70)
    print("1. Place checkerboard flat on platform (keep it fixed)")
    print("2. Move gripper OUT OF VIEW")
    print("3. Press 'D' to DETECT and store checkerboard pose")
    print("4. Press 'S' to SELECT corner, then enter corner number (0 to {})".format(
        pattern_size[0] * pattern_size[1] - 1))
    print("5. Move gripper center to TOUCH the selected corner")
    print("6. Press 'C' to CAPTURE gripper pose")
    print("7. Repeat steps 2-6 for different corners/positions (5-10 total)")
    print("8. Press 'K' to CALIBRATE and save")
    print("9. Press 'Q' to quit")
    print("=" * 70)
    print("\nCorner numbering:")
    print("  - Corner 0: Top-left (first corner)")
    print("  - Corner {}: Bottom-right (last corner)".format(pattern_size[0] * pattern_size[1] - 1))
    print("  - Pattern: {} columns x {} rows".format(pattern_size[0], pattern_size[1]))
    print("=" * 70 + "\n")
    
    window_name = 'Touch Method Calibration'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect checkerboard
            success, rvec, tvec, corners, drawn_frame = calib.detect_checkerboard_pose(frame)
            
            # Status display
            status_parts = []
            status_parts.append(f"Observations: {len(calib.observations)}")
            if calib.current_checkerboard_pose:
                if calib.current_corner_index is not None:
                    status_parts.append(f"Corner {calib.current_corner_index} selected - READY TO TOUCH")
                else:
                    status_parts.append("Checkerboard stored - SELECT CORNER")
            elif success:
                status_parts.append("Checkerboard: DETECTED (press 'D' to store)")
            else:
                status_parts.append("Checkerboard: NOT FOUND")
            
            status_text = " | ".join(status_parts)
            color = (0, 255, 0) if (success or calib.current_checkerboard_pose) else (0, 0, 255)
            cv2.putText(drawn_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            instructions = "D: Detect | S: Select Corner | C: Capture | K: Calibrate | Q: Quit"
            cv2.putText(drawn_frame, instructions, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Highlight selected corner if any
            if calib.current_checkerboard_pose and calib.current_corner_index is not None:
                corner_3d_cam = calib.get_corner_3d_position(
                    calib.current_corner_index,
                    calib.current_checkerboard_pose['rvec'],
                    calib.current_checkerboard_pose['tvec']
                )
                if corner_3d_cam is not None:
                    corner_2d, _ = cv2.projectPoints(
                        corner_3d_cam.reshape(1, 1, 3),
                        calib.current_checkerboard_pose['rvec'],
                        calib.current_checkerboard_pose['tvec'],
                        calib.camera_matrix,
                        calib.dist_coeffs
                    )
                    px, py = int(corner_2d[0][0][0]), int(corner_2d[0][0][1])
                    if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:
                        cv2.circle(drawn_frame, (px, py), 15, (0, 255, 255), 3)
                        cv2.putText(drawn_frame, f"TOUCH THIS CORNER ({calib.current_corner_index})", 
                                   (px - 100, py - 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow(window_name, drawn_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):
                break
            
            elif key == ord('d') and success:  # D - Detect and store
                calib.capture_checkerboard_pose(rvec, tvec, corners)
            
            elif key == ord('s'):  # S - Select corner
                if not calib.current_checkerboard_pose:
                    print("Error: No checkerboard pose stored. Press 'D' first.")
                    continue
                
                print(f"\nEnter corner index (0 to {pattern_size[0] * pattern_size[1] - 1}):")
                print("Corner layout: {} columns x {} rows".format(pattern_size[0], pattern_size[1]))
                print("Corner 0 = top-left, Corner {} = bottom-right".format(
                    pattern_size[0] * pattern_size[1] - 1))
                user_input = input("Corner index: ").strip()
                
                try:
                    corner_idx = int(user_input)
                    calib.select_corner(corner_idx)
                except ValueError:
                    print("Error: Invalid input")
            
            elif key == ord('c'):  # C - Capture gripper pose
                if not calib.current_checkerboard_pose:
                    print("Error: No checkerboard pose stored. Press 'D' first.")
                    continue
                if calib.current_corner_index is None:
                    print("Error: No corner selected. Press 'S' first.")
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

