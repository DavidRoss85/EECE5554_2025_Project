#!/usr/bin/env python3
"""
Simplified Pick System - No IK Required
Uses fixed movement sequences and only calculates base rotation

Steps:
1. Camera detects AprilTag location (x, y, z)
2. Calculate base servo to point toward AprilTag
3. Use manual/fixed movement strategy to approach (x, y from tag, z predefined)
4. Grab object
5. Pick up and move to retract location

Usage:
    python simple_pick.py pick <tag_id>    # Pick object with specified tag ID
    python simple_pick.py detect            # Detect all tags
    python simple_pick.py home              # Move to home position
    python simple_pick.py retract           # Move to retract position
"""

import sys
import os
import argparse
import cv2
import yaml
import numpy as np
import math
import time
import logging
import tempfile
from datetime import datetime

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'low-level_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))

from apriltag_detector import AprilTagDetector
from coordinate_transform import CoordinateTransformer

# Import low-level arm controller
try:
    from control_arm import ArmController, ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST, ID_GRIPPER
except ImportError:
    print("Error: Could not import control_arm. Check low-level_control path.")
    sys.exit(1)


class SimplePickSystem:
    """
    Simplified pick system using fixed movement sequences.
    No IK calculations - only base rotation and predefined movements.
    """
    
    def __init__(self, config_file=None):
        """Initialize the simplified pick system."""
        base_dir = os.path.join(os.path.dirname(__file__), '..')
        
        if config_file is None:
            config_file = os.path.join(base_dir, 'configs', 'robot_config.yaml')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger('SimplePick')
        
        # Load camera calibration
        camera_calib_path = os.path.join(
            base_dir,
            self.config['camera']['calibration_file']
        )
        
        if not os.path.exists(camera_calib_path):
            self.logger.warning(f"Camera calibration file not found: {camera_calib_path}")
            self.logger.warning("Using default calibration values")
            # Create default calibration
            self.camera_matrix = np.array([[800, 0, 640], [0, 800, 360], [0, 0, 1]], dtype=np.float32)
            self.dist_coeffs = np.zeros((5,), dtype=np.float32)
        else:
            calib_data = np.load(camera_calib_path)
            self.camera_matrix = calib_data['camera_matrix']
            self.dist_coeffs = calib_data['dist_coeffs']
        
        # Get tag size from config
        tag_size = self.config['apriltags']['tag_size']
        
        # Initialize camera and detector
        self.detector = AprilTagDetector(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            tag_size_m=tag_size
        )
        
        # Initialize coordinate transformer
        if os.path.exists(camera_calib_path):
            self.transformer = CoordinateTransformer(camera_calib_path, config_file)
        else:
            self.logger.warning("Coordinate transformer initialized without calibration file")
            # Create a temporary calibration file for CoordinateTransformer
            temp_calib = tempfile.NamedTemporaryFile(suffix='.npz', delete=False)
            np.savez(temp_calib.name, 
                     camera_matrix=self.camera_matrix,
                     dist_coeffs=self.dist_coeffs)
            temp_calib.close()
            self.transformer = CoordinateTransformer(temp_calib.name, config_file)
            # Clean up temp file after a delay (or leave it for debugging)
            # os.unlink(temp_calib.name)  # Uncomment to delete immediately
        
        # Initialize arm controller
        baudrate = self.config.get('servo', {}).get('baudrate', 1000000)
        
        self.arm = ArmController(baudrate)
        
        if not self.arm.initialize():
            raise RuntimeError("Failed to initialize arm controller")
        
        # Predefined Z heights for each tag ID (in meters, relative to platform)
        # These are the grip heights for each object
        self.tag_z_heights = {
            0: 0.092,  # Orange bottle - grip at middle (height 0.185m, so 0.185/2 = 0.0925m)
            1: 0.100,  # Apple bottle - grip at middle (height 0.200m, so 0.100m)
            2: 0.078,  # Yogurt bottle - grip at middle (height 0.155m, so 0.0775m)
        }
        
        # Fixed movement sequences (servo positions)
        # These are predefined positions for approach, grip, and lift
        self.movement_sequences = {
            'approach': {
                'shoulder': 2048,  # 90 degrees (horizontal)
                'elbow': 2048,     # 90 degrees
                'wrist': 2048,     # 90 degrees (horizontal)
            },
            'grip': {
                'shoulder': 2048,  # Keep same
                'elbow': 2500,      # Slightly more extended
                'wrist': 1800,     # Slightly down
            },
            'lift': {
                'shoulder': 1800,  # Slightly up
                'elbow': 2200,      # Slightly retracted
                'wrist': 2000,      # Horizontal
            }
        }
        
        # Movement delays
        self.movement_delay = 0.5
        self.grip_delay = 1.0
        
        self.logger.info("SimplePick system initialized")
    
    def calculate_base_angle(self, x, y):
        """
        Calculate base servo position to point toward (x, y) location.
        
        Args:
            x, y: Target position in robot base frame (meters)
            
        Returns:
            base_servo_position: Servo position (0-4095)
        """
        # Calculate angle in radians
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)
        
        # Convert to servo position
        # Servo mapping: 0° = 999, 90° = 2048, 180° = 3093
        # Formula: position = 2048 + (angle_deg - 90) * (2048 - 999) / 90
        # Or: position = 2048 + angle_deg * 11.38 (since 90° = 1049 counts)
        base_position = int(2048 + (angle_deg - 90) * (2048 - 999) / 90)
        
        # Clamp to limits
        base_min, base_max = self.config['position_limits']['base']
        base_position = max(base_min, min(base_max, base_position))
        
        self.logger.info(f"Base angle: {angle_deg:.1f}° -> servo position: {base_position}")
        return base_position
    
    def move_base_to_tag(self, tag_x, tag_y):
        """
        Move base servo to point toward tag location.
        
        Args:
            tag_x, tag_y: Tag position in robot base frame (meters)
        """
        base_position = self.calculate_base_angle(tag_x, tag_y)
        
        print(f"Moving base to point toward tag: ({tag_x*100:.1f}cm, {tag_y*100:.1f}cm)")
        print(f"  Base servo position: {base_position}")
        
        if not self.arm.set_goal_position(ID_BASE, base_position):
            print("  ✗ Failed to move base")
            return False
        
        time.sleep(2.0)  # Wait for base to rotate
        print("  ✓ Base rotated to tag direction")
        return True
    
    def move_to_approach_position(self, tag_x, tag_y, tag_z):
        """
        Move to approach position using fixed movement sequence.
        This positions the arm above the target location.
        
        Args:
            tag_x, tag_y: Tag position (meters)
            tag_z: Predefined Z height for this tag ID (meters)
        """
        print(f"\nMoving to approach position:")
        print(f"  Target: X={tag_x*100:.1f}cm, Y={tag_y*100:.1f}cm, Z={tag_z*100:.1f}cm")
        
        # Use approach sequence
        seq = self.movement_sequences['approach']
        
        # Move base first (already done, but ensure it's correct)
        # Then move shoulder, elbow, wrist to approach positions
        positions = [
            self.calculate_base_angle(tag_x, tag_y),  # Base
            seq['shoulder'],  # Shoulder
            seq['elbow'],     # Elbow
            seq['wrist']      # Wrist
        ]
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, positions[i]):
                print(f"  ✗ Failed to move servo {servo_id}")
                return False
            time.sleep(0.1)
        
        time.sleep(self.movement_delay * 2)  # Wait for movement
        print("  ✓ Reached approach position")
        return True
    
    def move_to_grip_position(self, tag_x, tag_y, tag_z):
        """
        Move to grip position - lower arm to grab object.
        
        Args:
            tag_x, tag_y: Tag position (meters)
            tag_z: Predefined Z height (meters)
        """
        print(f"\nMoving to grip position:")
        print(f"  Target: X={tag_x*100:.1f}cm, Y={tag_y*100:.1f}cm, Z={tag_z*100:.1f}cm")
        
        # Use grip sequence
        seq = self.movement_sequences['grip']
        
        positions = [
            self.calculate_base_angle(tag_x, tag_y),  # Base (keep pointing)
            seq['shoulder'],  # Shoulder
            seq['elbow'],     # Elbow (more extended)
            seq['wrist']      # Wrist (slightly down)
        ]
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, positions[i]):
                print(f"  ✗ Failed to move servo {servo_id}")
                return False
            time.sleep(0.1)
        
        time.sleep(self.movement_delay * 2)
        print("  ✓ Reached grip position")
        return True
    
    def close_gripper(self):
        """Close gripper to grab object."""
        closed_pos = self.config['gripper']['closed_position']
        print(f"\nClosing gripper...")
        
        if not self.arm.set_goal_position(ID_GRIPPER, closed_pos):
            print("  ✗ Failed to close gripper")
            return False
        
        time.sleep(self.grip_delay)
        print("  ✓ Gripper closed")
        return True
    
    def lift_object(self, tag_x, tag_y):
        """
        Lift object after grabbing.
        
        Args:
            tag_x, tag_y: Tag position (for base orientation)
        """
        print(f"\nLifting object...")
        
        # Use lift sequence
        seq = self.movement_sequences['lift']
        
        positions = [
            self.calculate_base_angle(tag_x, tag_y),  # Base (keep pointing)
            seq['shoulder'],  # Shoulder (slightly up)
            seq['elbow'],     # Elbow (slightly retracted)
            seq['wrist']      # Wrist (horizontal)
        ]
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, positions[i]):
                print(f"  ✗ Failed to move servo {servo_id}")
                return False
            time.sleep(0.1)
        
        time.sleep(self.movement_delay * 2)
        print("  ✓ Object lifted")
        return True
    
    def move_to_retract(self):
        """Move to retract position."""
        retract_pos = self.config['positions']['retract']['values']
        print(f"\nMoving to retract position...")
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, retract_pos[i]):
                print(f"  ✗ Failed to move servo {servo_id}")
                return False
            time.sleep(0.1)
        
        time.sleep(self.movement_delay * 2)
        print("  ✓ Reached retract position")
        return True
    
    def move_to_home(self):
        """Move to home position."""
        home_pos = self.config['positions']['home']['values']
        print(f"\nMoving to home position...")
        
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST]
        
        for i, servo_id in enumerate(servo_ids):
            if not self.arm.set_goal_position(servo_id, home_pos[i]):
                print(f"  ✗ Failed to move servo {servo_id}")
                return False
            time.sleep(0.1)
        
        time.sleep(self.movement_delay * 2)
        print("  ✓ Reached home position")
        return True
    
    def open_gripper(self):
        """Open gripper."""
        open_pos = self.config['gripper']['open_position']
        print(f"\nOpening gripper...")
        
        if not self.arm.set_goal_position(ID_GRIPPER, open_pos):
            print("  ✗ Failed to open gripper")
            return False
        
        time.sleep(self.grip_delay)
        print("  ✓ Gripper opened")
        return True
    
    def detect_tag(self, tag_id):
        """
        Detect AprilTag and return its position in robot base frame.
        
        Args:
            tag_id: AprilTag ID to detect
            
        Returns:
            dict with tag position or None if not found
        """
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.logger.error("Failed to open camera")
            return None
        
        print(f"\nDetecting AprilTag ID {tag_id}...")
        max_attempts = 30
        
        for attempt in range(max_attempts):
            ret, frame = cap.read()
            if not ret:
                continue
            
            detections = self.detector.detect_tags(frame)
            
            for det in detections:
                if det['tag_id'] == tag_id:
                    # Get position in camera frame
                    pos_camera = det['position_camera']
                    
                    # Transform to base frame
                    pos_base = self.transformer.camera_3d_to_base_3d(pos_camera)
                    
                    # Apply offsets
                    x_offset = self.config['coordinate_transform'].get('x_offset', 0.0)
                    y_offset = self.config['coordinate_transform'].get('y_offset', 0.0)
                    z_offset = self.config['coordinate_transform'].get('z_offset', 0.0)
                    
                    pos_base[0] += x_offset
                    pos_base[1] += y_offset
                    pos_base[2] += z_offset
                    
                    cap.release()
                    
                    print(f"  ✓ Tag {tag_id} detected:")
                    print(f"    Position (base frame): X={pos_base[0]*100:.1f}cm, Y={pos_base[1]*100:.1f}cm, Z={pos_base[2]*100:.1f}cm")
                    
                    return {
                        'id': tag_id,
                        'position': pos_base,
                        'position_camera': pos_camera
                    }
            
            if attempt % 10 == 0:
                print(f"  Attempt {attempt+1}/{max_attempts}...")
        
        cap.release()
        print(f"  ✗ Tag {tag_id} not found after {max_attempts} attempts")
        return None
    
    def detect_all_tags(self):
        """Detect all visible AprilTags."""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.logger.error("Failed to open camera")
            return []
        
        # Set camera resolution (match apriltag_base_coordinates.py)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Warm up camera - flush a few frames to let it stabilize
        for _ in range(5):
            cap.read()
        
        print("\nDetecting all AprilTags...")
        print("  (Will keep trying until tags are found, or press Ctrl+C to cancel)")
        
        # Try multiple frames to get a good detection (like apriltag_base_coordinates.py)
        # Accept ANY detection (no quality filtering) - same as apriltag_base_coordinates.py
        detections = []
        attempt = 0
        max_attempts = 1000  # Very high limit, but keep trying
        
        try:
            while attempt < max_attempts:
                ret, frame = cap.read()
                if not ret:
                    attempt += 1
                    time.sleep(0.1)  # Small delay if frame read fails
                    continue
                
                frame_detections = self.detector.detect_tags(frame)
                
                # Accept any detections (no quality filtering, same as apriltag_base_coordinates.py)
                if frame_detections:
                    detections = frame_detections
                    print(f"  Found {len(detections)} tag(s) after {attempt+1} attempts")
                    break
                
                attempt += 1
                if attempt % 30 == 0:  # Print status every 30 attempts
                    print(f"  Attempt {attempt}... (still searching)")
        
        except KeyboardInterrupt:
            print("\n  Detection cancelled by user")
            cap.release()
            return []
        
        cap.release()
        
        if not detections:
            print(f"  No tags detected after {attempt} attempts")
            print("  Make sure the tag is visible and well-lit")
            return []
        
        print(f"  Found {len(detections)} tag(s):")
        for det in detections:
            pos_camera = det['position_camera'].copy()  # Make a copy to avoid modifying original
            center_px = det['center_pixel']
            
            # Validate detection quality and pixel location
            pixel_x, pixel_y = center_px[0], center_px[1]
            decision_margin = det['decision_margin']
            hamming = det['hamming']
            
            # Debug: Print camera frame values
            print(f"    [DEBUG] Tag {det['tag_id']} - Camera Frame: X={pos_camera[0]*100:.2f}cm, Y={pos_camera[1]*100:.2f}cm, Z={pos_camera[2]*100:.2f}cm")
            print(f"    [DEBUG] Tag {det['tag_id']} - Pixel: ({pixel_x:.1f}, {pixel_y:.1f})")
            print(f"    [DEBUG] Tag {det['tag_id']} - Quality: H={hamming}, M={decision_margin:.1f}")
            
            # Warn if pixel is way off (likely bad detection)
            if pixel_x < 100 or pixel_x > 1180 or pixel_y < 100 or pixel_y > 620:
                print(f"    [WARNING] Pixel coordinates ({pixel_x:.1f}, {pixel_y:.1f}) seem unusual!")
                print(f"             Expected tag to be near center of image (640, 360)")
            
            # Warn if Z distance is unreasonable (camera is 70cm high, tag shouldn't be >100cm away)
            if pos_camera[2] > 1.0:  # More than 1 meter
                print(f"    [WARNING] Z distance {pos_camera[2]*100:.1f}cm seems too large!")
                print(f"             Camera is 70cm high, tag should be closer")
            
            pos_base = self.transformer.camera_3d_to_base_3d(pos_camera).copy()  # Make a copy
            
            # Debug: Print base frame before offsets
            print(f"    [DEBUG] Tag {det['tag_id']} - Base Frame (RAW): X={pos_base[0]*100:.2f}cm, Y={pos_base[1]*100:.2f}cm, Z={pos_base[2]*100:.2f}cm")
            
            # Apply offsets
            x_offset = self.config['coordinate_transform'].get('x_offset', 0.0)
            y_offset = self.config['coordinate_transform'].get('y_offset', 0.0)
            z_offset = self.config['coordinate_transform'].get('z_offset', 0.0)
            
            pos_base[0] += x_offset
            pos_base[1] += y_offset
            pos_base[2] += z_offset
            
            print(f"    Tag {det['tag_id']}: X={pos_base[0]*100:.1f}cm, Y={pos_base[1]*100:.1f}cm, Z={pos_base[2]*100:.1f}cm")
        
        return detections
    
    def pick_object(self, tag_id):
        """
        Complete pick operation for a tag ID.
        
        Steps:
        1. Detect tag location
        2. Move base to point toward tag
        3. Use fixed movement to approach
        4. Grab object
        5. Lift and retract
        """
        print("\n" + "="*60)
        print(f"PICK OPERATION - Tag ID {tag_id}")
        print("="*60)
        
        # Step 1: Detect tag
        tag_info = self.detect_tag(tag_id)
        if tag_info is None:
            print("  ✗ Failed to detect tag")
            return False
        
        tag_x, tag_y, tag_z_camera = tag_info['position']
        
        # Get predefined Z height for this tag ID
        if tag_id not in self.tag_z_heights:
            print(f"  ✗ No predefined Z height for tag ID {tag_id}")
            return False
        
        grip_z = self.tag_z_heights[tag_id]
        
        print(f"\nTag position: X={tag_x*100:.1f}cm, Y={tag_y*100:.1f}cm")
        print(f"Grip height: Z={grip_z*100:.1f}cm (predefined for tag {tag_id})")
        
        # Step 2: Move base to point toward tag
        if not self.move_base_to_tag(tag_x, tag_y):
            return False
        
        # Step 3: Move to approach position
        if not self.move_to_approach_position(tag_x, tag_y, grip_z):
            return False
        
        # Step 4: Move to grip position
        if not self.move_to_grip_position(tag_x, tag_y, grip_z):
            return False
        
        # Step 5: Close gripper
        if not self.close_gripper():
            return False
        
        # Step 6: Lift object
        if not self.lift_object(tag_x, tag_y):
            return False
        
        # Step 7: Move to retract
        if not self.move_to_retract():
            return False
        
        print("\n" + "="*60)
        print("PICK OPERATION COMPLETE")
        print("="*60)
        return True
    
    def shutdown(self):
        """Shutdown the system."""
        self.logger.info("Shutting down system")
        try:
            if hasattr(self, 'arm') and self.arm is not None:
                self.arm.shutdown()
        except Exception as e:
            self.logger.warning(f"Error during arm shutdown: {e}")
        
        # Clean up any open camera handles
        # (Camera should be released in detect methods, but just in case)
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        except:
            pass


def main():
    parser = argparse.ArgumentParser(description='Simplified Pick System - No IK Required')
    parser.add_argument('command', choices=['pick', 'detect', 'home', 'retract'],
                       help='Command to execute')
    parser.add_argument('tag_id', type=int, nargs='?',
                       help='Tag ID for pick command (0=Orange, 1=Apple, 2=Yogurt)')
    
    args = parser.parse_args()
    
    try:
        system = SimplePickSystem()
        
        if args.command == 'pick':
            if args.tag_id is None:
                print("Error: tag_id required for pick command")
                print("Usage: python simple_pick.py pick <tag_id>")
                print("  tag_id: 0=Orange, 1=Apple, 2=Yogurt")
                return 1
            
            success = system.pick_object(args.tag_id)
            return 0 if success else 1
        
        elif args.command == 'detect':
            system.detect_all_tags()
            return 0
        
        elif args.command == 'home':
            success = system.move_to_home()
            return 0 if success else 1
        
        elif args.command == 'retract':
            success = system.move_to_retract()
            return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if 'system' in locals():
            system.shutdown()


if __name__ == '__main__':
    sys.exit(main())

