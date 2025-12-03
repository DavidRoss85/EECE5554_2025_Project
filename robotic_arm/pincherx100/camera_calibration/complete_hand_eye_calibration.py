#!/usr/bin/env python3
"""
Complete Hand-Eye Calibration

This script completes the hand-eye calibration using captured observations.
It requires:
1. Forward kinematics to calculate gripper poses from joint positions
2. Checkerboard poses from camera (already captured)
3. cv2.calibrateHandEye() to compute transformation

Usage:
    python complete_hand_eye_calibration.py [--observations hand_eye_observations.json]
"""

import cv2
import numpy as np
import json
import argparse
import os
import sys

# Add path for forward kinematics
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pick_place_system', 'vision'))
from forward_kinematics import PincherX100FK


def load_observations(obs_file):
    """Load observations from JSON file."""
    with open(obs_file, 'r') as f:
        data = json.load(f)
    return data


def get_joint_positions_from_user(observation_num, total):
    """
    Prompt user to enter joint positions for an observation.
    
    Returns:
        positions: [base, shoulder, elbow, wrist] in servo units, or None if skipped
    """
    print(f"\nObservation {observation_num}/{total}")
    print("Enter joint positions (base,shoulder,elbow,wrist) in servo units (0-4095)")
    print("Or press Enter to skip this observation")
    print("Example: 2048,2048,2048,2048")
    
    user_input = input("Positions: ").strip()
    
    if not user_input:
        return None
    
    try:
        parts = user_input.split(',')
        if len(parts) != 4:
            print("Error: Need 4 values")
            return None
        
        positions = [int(x.strip()) for x in parts]
        
        # Validate range
        for i, pos in enumerate(positions):
            if pos < 0 or pos > 4095:
                print(f"Warning: Position {i} ({pos}) is outside valid range [0-4095]")
        
        return positions
    except ValueError:
        print("Error: Invalid input format")
        return None


def complete_calibration(obs_file, output_file='hand_eye_calibration.json', square_size=24.0):
    """
    Complete hand-eye calibration from observations.
    
    Args:
        obs_file: Path to observations JSON file
        output_file: Path to output calibration file
        square_size: Square size in mm
    """
    print("=" * 70)
    print("Hand-Eye Calibration Completion")
    print("=" * 70)
    
    # Load observations
    print(f"\nLoading observations from {obs_file}...")
    data = load_observations(obs_file)
    
    num_obs = data.get('num_observations', len(data.get('robot_poses', [])))
    print(f"Found {num_obs} observations")
    
    if num_obs < 3:
        print("Error: Need at least 3 observations for calibration")
        return False
    
    # Check if observations have joint positions
    robot_poses = data.get('robot_poses', [])
    has_joint_positions = False
    
    for pose in robot_poses:
        if 'joint_positions' in pose and pose['joint_positions']:
            has_joint_positions = True
            break
    
    # If no joint positions, we need to get them
    if not has_joint_positions:
        print("\n" + "=" * 70)
        print("Joint positions not found in observations")
        print("=" * 70)
        print("\nYou need to provide joint positions for each observation.")
        print("You can:")
        print("1. Manually enter positions now (for each observation)")
        print("2. Or edit the JSON file to add 'joint_positions' to each observation")
        print("\nTo get joint positions:")
        print("- Use the arm controller to read current positions")
        print("- Or note them down when capturing observations")
        print("\nPress Enter to continue with manual entry, or Ctrl+C to exit")
        input()
        
        # Get joint positions for each observation
        for i, pose in enumerate(robot_poses):
            positions = get_joint_positions_from_user(i + 1, num_obs)
            if positions:
                pose['joint_positions'] = positions
            else:
                print(f"Skipping observation {i + 1}")
    
    # Initialize forward kinematics
    fk = PincherX100FK()
    
    # Load camera calibration for checkerboard detection
    camera_calib_file = 'camera_calibration.npz'
    if not os.path.exists(camera_calib_file):
        print(f"Error: Camera calibration file not found: {camera_calib_file}")
        return False
    
    calib_data = np.load(camera_calib_file)
    camera_matrix = calib_data['camera_matrix']
    dist_coeffs = calib_data['dist_coeffs']
    
    # Prepare data for cv2.calibrateHandEye
    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []
    
    valid_observations = 0
    
    print("\n" + "=" * 70)
    print("Processing observations...")
    print("=" * 70)
    
    # We need to reconstruct checkerboard poses from observations
    # For now, we'll need the user to re-run detection or provide the poses
    # Alternatively, we can load them if they were saved
    
    # Check if observations have checkerboard poses
    pattern_size = tuple(data.get('pattern_size', [6, 7]))
    square_size_m = square_size / 1000.0
    
    # Prepare object points
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_m
    
    print("\nNote: This script requires checkerboard poses (rvec, tvec) for each observation.")
    print("If these were not saved, you may need to:")
    print("1. Re-run hand_eye_calibration.py and capture with joint positions")
    print("2. Or manually add 'rvec' and 'tvec' to each observation in the JSON file")
    print("\nFor now, we'll attempt to use what's available...")
    
    for i, pose in enumerate(robot_poses):
        # Get joint positions
        if 'joint_positions' not in pose or not pose['joint_positions']:
            print(f"Skipping observation {i + 1}: No joint positions")
            continue
        
        joint_positions = np.array(pose['joint_positions'][0:4])  # base, shoulder, elbow, wrist
        
        # Calculate gripper pose using forward kinematics
        try:
            T_gripper2base = fk.solve_from_servo_positions(joint_positions)
            R_gripper2base = T_gripper2base[0:3, 0:3]
            t_gripper2base = T_gripper2base[0:3, 3]
        except Exception as e:
            print(f"Error calculating FK for observation {i + 1}: {e}")
            continue
        
        # Get checkerboard pose (if available)
        if 'rvec' in pose and 'tvec' in pose:
            rvec = np.array(pose['rvec'])
            tvec = np.array(pose['tvec'])
        elif 'R_target2cam' in pose and 't_target2cam' in pose:
            R_target2cam = np.array(pose['R_target2cam'])
            t_target2cam = np.array(pose['t_target2cam'])
        else:
            print(f"Warning: Observation {i + 1} missing checkerboard pose")
            print("  You may need to re-capture or add rvec/tvec manually")
            continue
        
        # Convert rvec to rotation matrix if needed
        if 'rvec' in pose:
            R_target2cam, _ = cv2.Rodrigues(rvec)
            t_target2cam = tvec
        else:
            R_target2cam = np.array(pose['R_target2cam'])
            t_target2cam = np.array(pose['t_target2cam']).flatten()
        
        # Store for calibration
        R_gripper2base_list.append(R_gripper2base)
        t_gripper2base_list.append(t_gripper2base)
        R_target2cam_list.append(R_target2cam)
        t_target2cam_list.append(t_target2cam)
        
        valid_observations += 1
        print(f"  Observation {i + 1}: OK")
    
    if valid_observations < 3:
        print(f"\nError: Only {valid_observations} valid observations (need at least 3)")
        return False
    
    print(f"\nUsing {valid_observations} valid observations for calibration...")
    
    # Perform hand-eye calibration
    print("\n" + "=" * 70)
    print("Performing hand-eye calibration...")
    print("=" * 70)
    
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
        
        print("\nCalibration successful!")
        print(f"\nTransformation matrix (Camera to Base):")
        print(T_cam2base)
        print(f"\nTranslation (camera position in base frame):")
        print(f"  X: {t_cam2base[0,0]:.4f} m")
        print(f"  Y: {t_cam2base[1,0]:.4f} m")
        print(f"  Z: {t_cam2base[2,0]:.4f} m")
        
        # Save calibration result
        result = {
            'calibration_date': data.get('capture_date', ''),
            'num_observations': valid_observations,
            'pattern_size': list(pattern_size),
            'square_size_mm': square_size,
            'transform_matrix': T_cam2base.tolist(),
            'rotation_matrix': R_cam2base.tolist(),
            'translation_vector': t_cam2base.flatten().tolist(),
            'observations_used': valid_observations
        }
        
        with open(output_file, 'w') as f:
            json.dump(result, f, indent=2)
        
        print(f"\nCalibration saved to: {output_file}")
        print("\n" + "=" * 70)
        print("Calibration complete!")
        print("=" * 70)
        print(f"\nYou can now use this calibration file in the pick_place_system")
        print(f"by loading it in coordinate_transform.py")
        
        return True
        
    except Exception as e:
        print(f"\nError during calibration: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(description='Complete Hand-Eye Calibration')
    parser.add_argument('--observations', type=str, default='hand_eye_observations.json',
                       help='Observations JSON file')
    parser.add_argument('--output', type=str, default='hand_eye_calibration.json',
                       help='Output calibration file')
    parser.add_argument('--square', type=float, default=24.0,
                       help='Square size in mm (default: 24mm)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.observations):
        print(f"Error: Observations file not found: {args.observations}")
        return
    
    success = complete_calibration(args.observations, args.output, args.square)
    
    if not success:
        print("\nCalibration failed. Please check:")
        print("1. Observations have joint positions")
        print("2. Observations have checkerboard poses (rvec/tvec)")
        print("3. At least 3 valid observations")


if __name__ == '__main__':
    main()

