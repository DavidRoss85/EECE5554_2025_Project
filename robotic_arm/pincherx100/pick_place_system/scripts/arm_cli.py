#!/usr/bin/env python3
"""
Command-Line Interface for Pick and Place Operations

Usage:
    python arm_cli.py pick orange
    python arm_cli.py pick yogurt
    python arm_cli.py drop
    python arm_cli.py status
    python arm_cli.py home
"""

import sys
import os
import argparse
import cv2
import yaml

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from object_detector import ObjectDetector
from checkerboard_detector import CheckerboardDetector
from coordinate_transform import CoordinateTransformer
from arm_controller_wrapper import PickPlaceController
from inverse_kinematics import PincherX100IK


class PickPlaceCLI:
    """
    Command-line interface for pick and place operations.
    """
    
    def __init__(self, config_file=None):
        """
        Initialize CLI.
        
        Args:
            config_file: Path to robot config YAML
        """
        # Load configuration
        if config_file is None:
            config_file = os.path.join(os.path.dirname(__file__), '..', 'configs', 'robot_config.yaml')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Initialize components
        camera_calib = os.path.join(os.path.dirname(__file__), '..', '..', 'camera_calibration', 'camera_calibration.npz')
        self.transformer = CoordinateTransformer(camera_calib, config_file)
        
        # Load hand-eye calibration if available
        # NOTE: Hand-eye calibration may be incorrect. Set USE_HAND_EYE_CALIBRATION=False to disable.
        USE_HAND_EYE_CALIBRATION = False  # Set to True to use hand-eye calibration
        hand_eye_file = os.path.join(os.path.dirname(__file__), '..', 'calibration', 'hand_eye_calibration.json')
        if USE_HAND_EYE_CALIBRATION and os.path.exists(hand_eye_file):
            self.transformer.load_hand_eye_calibration(hand_eye_file)
            print("  Using hand-eye calibration transform")
        else:
            print("  Using approximate coordinate transformation (hand-eye calibration disabled)")
        
        # Initialize detectors
        # Use lower confidence threshold for better detection
        model_path = self.config.get('detection', {}).get('yolo_model', 'yolov8n.pt')
        confidence = self.config.get('detection', {}).get('confidence_threshold', 0.3)
        use_cap_detector = self.config.get('detection', {}).get('use_cap_detector', True)  # Default to True for top-down
        
        self.object_detector = ObjectDetector(
            model_path=model_path,
            confidence_threshold=confidence,
            use_cap_detector=use_cap_detector
        )
        
        # Initialize cap detector with camera calibration if using top-down view
        if use_cap_detector:
            square_size_mm = self.config.get('checkerboard', {}).get('square_size', 24.0) * 1000
            self.object_detector.set_cap_detector(
                camera_matrix=self.transformer.camera_matrix,
                dist_coeffs=self.transformer.dist_coeffs,
                square_size_mm=square_size_mm
            )
        self.checkerboard_detector = CheckerboardDetector(
            pattern_size=tuple(self.config['checkerboard']['pattern_size']),
            square_size=self.config['checkerboard']['square_size'] * 1000  # Convert to mm
        )
        
        # Initialize arm controller
        self.arm_controller = PickPlaceController()
        
        # Camera
        self.camera_index = 0
        self.cap = None
        
        print("Pick and Place CLI initialized")
    
    def open_camera(self):
        """Open camera."""
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            print(f"Error: Failed to open camera {self.camera_index}")
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return True
    
    def close_camera(self):
        """Close camera."""
        if self.cap:
            self.cap.release()
            self.cap = None
    
    def capture_frame(self):
        """Capture frame from camera."""
        if not self.cap:
            if not self.open_camera():
                return None
        
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            return None
        
        return frame
    
    def detect_object(self, object_type, show_debug=False):
        """
        Detect object in camera frame.
        
        Args:
            object_type: 'orange_bottle' or 'yogurt_bottle'
            show_debug: If True, show camera feed with detections
        
        Returns:
            (success, position_base, detection_info)
        """
        print(f"Detecting {object_type}...")
        
        frame = self.capture_frame()
        if frame is None:
            return False, None, None
        
        # Detect objects
        detections = self.object_detector.detect_bottles(frame)
        
        # Debug: Show all detections
        print(f"  Total detections: {len(detections)}")
        for i, det in enumerate(detections):
            print(f"    Detection {i}: {det['type']} (confidence: {det['confidence']:.2f}, "
                  f"color_match: {det['color_match']}, center: {det['center']})")
        
        # Filter by type
        matching_detections = [d for d in detections if d['type'] == object_type]
        
        if len(matching_detections) == 0:
            print(f"  No {object_type} detected")
            if show_debug:
                # Show debug window with all detections
                annotated = self.object_detector.draw_detections(frame, detections)
                cv2.putText(annotated, f"Looking for: {object_type}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(annotated, f"Found: {len(detections)} bottles", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(annotated, "Press any key to continue...", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow('Debug: Object Detection', annotated)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            return False, None, None
        
        # Use best detection (highest confidence)
        detection = max(matching_detections, key=lambda d: d['confidence'])
        print(f"  Found {object_type} at pixel: {detection['center']}")
        print(f"  Confidence: {detection['confidence']:.2f}")
        print(f"  Color match: {detection['color_match']}")
        
        # Show debug window if requested
        if show_debug:
            annotated = self.object_detector.draw_detections(frame, detections)
            # Highlight the selected detection
            bbox = detection['bbox']
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cx, cy = map(int, detection['center'])
            cv2.circle(annotated, (cx, cy), 8, (0, 255, 0), -1)
            cv2.putText(annotated, f"SELECTED: {object_type}", (x1, y1 - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, "Press any key to continue...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow('Debug: Object Detection', annotated)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        # Convert to robot base coordinates
        pixel_x, pixel_y = detection['center']
        
        # Get object height from config
        if object_type == 'orange_bottle':
            object_height = self.config['objects']['orange_bottle']['height']
        else:
            object_height = self.config['objects']['yogurt_bottle']['height']
        
        # The cap is detected at the top of the bottle
        # For coordinate transformation, we need to account for the cap height
        # Cap is at bottle_height above platform, so use that for Z distance calculation
        # This gives us the X,Y position at the base of the bottle
        position_base = self.transformer.pixel_to_base_3d(pixel_x, pixel_y, object_height=object_height)
        
        # The Z coordinate from transformation is at cap level, but we want base of bottle
        # So adjust Z to platform level (0.0)
        position_base[2] = 0.0  # Base of bottle is at platform level
        
        print(f"  Cap detected at pixel: ({pixel_x:.1f}, {pixel_y:.1f})")
        print(f"  Bottle base position in base frame: ({position_base[0]:.3f}, {position_base[1]:.3f}, {position_base[2]:.3f})")
        print(f"  Bottle height: {object_height*100:.1f}cm")
        
        detection_info = {
            'detection': detection,
            'pixel': [pixel_x, pixel_y],
            'base': position_base.tolist(),
            'object_height': object_height
        }
        
        return True, position_base, detection_info
    
    def detect_drop_location(self, show_debug=False):
        """
        Detect drop location (checkerboard center).
        
        Args:
            show_debug: If True, show camera feed with detection overlay
        
        Returns:
            (success, position_base, detection_info)
        """
        print("Detecting drop location...")
        
        frame = self.capture_frame()
        if frame is None:
            return False, None, None
        
        # Detect checkerboard
        success, center_pixel, center_3d, pose_data = self.checkerboard_detector.detect(
            frame,
            self.transformer.camera_matrix,
            self.transformer.dist_coeffs
        )
        
        if not success:
            print("  Checkerboard not detected")
            if show_debug:
                cv2.putText(frame, "Checkerboard not detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, "Press any key to continue...", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow('Debug: Checkerboard Detection', frame)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            return False, None, None
        
        if show_debug and pose_data and 'drawn_image' in pose_data:
            cv2.putText(pose_data['drawn_image'], "Press any key to continue...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow('Debug: Checkerboard Detection', pose_data['drawn_image'])
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        print(f"  Found checkerboard center at pixel: {center_pixel}")
        
        # Convert to robot base coordinates
        pixel_x, pixel_y = center_pixel
        position_base = self.transformer.pixel_to_base_3d(pixel_x, pixel_y, object_height=0.0)
        print(f"  Position in base frame: ({position_base[0]:.3f}, {position_base[1]:.3f}, {position_base[2]:.3f})")
        
        detection_info = {
            'pixel': center_pixel.tolist(),
            'base': position_base.tolist(),
            'pose_data': pose_data
        }
        
        return True, position_base, detection_info
    
    def pick(self, object_type, show_debug=False):
        """
        Pick up object.
        
        Args:
            object_type: 'orange_bottle' or 'yogurt_bottle'
            show_debug: If True, show camera feed with detections
        """
        if not self.arm_controller.initialized:
            print("Initializing arm...")
            if not self.arm_controller.initialize():
                print("Error: Failed to initialize arm")
                return False
        
        # Detect object
        success, position_base, detection_info = self.detect_object(object_type, show_debug=show_debug)
        if not success:
            print(f"Error: Could not detect {object_type}")
            return False
        
        # Get object height
        object_height = detection_info['object_height']
        grip_height_ratio = self.config['objects'][object_type]['grip_height_ratio']
        grip_height = object_height * grip_height_ratio
        
        # Get pick parameters from config
        pick_params = self.config['pick_place']
        
        # Perform pick
        success = self.arm_controller.pick_object(
            position_base,
            grip_height,
            approach_height=pick_params['approach_height'],
            lift_height=pick_params['lift_height']
        )
        
        if success:
            print(f"Successfully picked {object_type}")
        else:
            print(f"Failed to pick {object_type}")
        
        return success
    
    def drop(self, show_debug=False):
        """
        Drop object at drop location.
        
        Args:
            show_debug: If True, show camera feed with detection overlay
        """
        if not self.arm_controller.initialized:
            print("Error: Arm not initialized")
            return False
        
        # Detect drop location
        success, position_base, detection_info = self.detect_drop_location(show_debug=show_debug)
        if not success:
            print("Error: Could not detect drop location")
            return False
        
        # Get drop parameters from config
        drop_height = self.config['pick_place']['drop_height']
        
        # Perform place
        success = self.arm_controller.place_object(position_base, drop_height=drop_height)
        
        if success:
            print("Successfully placed object")
        else:
            print("Failed to place object")
        
        return success
    
    def status(self):
        """Show system status."""
        print("\n=== System Status ===")
        
        # Arm status
        if self.arm_controller.initialized:
            positions = self.arm_controller.get_current_joint_positions()
            if positions:
                print(f"Arm: Initialized")
                print(f"  Joint positions: {positions}")
            else:
                print("Arm: Initialized (cannot read positions)")
        else:
            print("Arm: Not initialized")
        
        # Camera status
        if self.cap and self.cap.isOpened():
            print("Camera: Open")
        else:
            print("Camera: Closed")
        
        # Calibration status
        if self.transformer.hand_eye_loaded:
            print("Hand-eye calibration: Loaded")
        else:
            print("Hand-eye calibration: Not loaded (using approximate transform)")
        
        print("====================\n")
    
    def home(self):
        """Move arm to home position."""
        if not self.arm_controller.initialized:
            print("Initializing arm...")
            if not self.arm_controller.initialize():
                print("Error: Failed to initialize arm")
                return False
        
        print("Moving to home position...")
        success = self.arm_controller.move_to_home()
        if success:
            print("Arm moved to home position")
        else:
            print("Failed to move to home position")
        
        return success
    
    def shutdown(self):
        """Shutdown all components."""
        self.close_camera()
        if self.arm_controller.initialized:
            self.arm_controller.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Pick and Place CLI for PincherX100')
    parser.add_argument('command', choices=['pick', 'drop', 'status', 'home', 'test_detection'],
                       help='Command to execute')
    parser.add_argument('object_type', nargs='?', choices=['orange', 'yogurt'],
                       help='Object type for pick command')
    parser.add_argument('--config', type=str, default=None,
                       help='Path to robot config file')
    parser.add_argument('--debug', action='store_true',
                       help='Show camera feed with detections (debug mode)')
    parser.add_argument('--model', type=str, default='yolov8n.pt',
                       help='YOLO model path (default: yolov8n.pt, try yolov8s.pt or yolov8m.pt for better accuracy)')
    
    args = parser.parse_args()
    
    # Initialize CLI
    cli = PickPlaceCLI(config_file=args.config)
    
    # Override YOLO model if specified
    if args.model != 'yolov8n.pt':
        print(f"Using YOLO model: {args.model}")
        cli.object_detector = ObjectDetector(model_path=args.model, confidence_threshold=0.5)
    
    try:
        # Execute command
        if args.command == 'pick':
            if args.object_type is None:
                print("Error: Object type required for pick command")
                print("Usage: python arm_cli.py pick [orange|yogurt] [--debug] [--model MODEL]")
                return
            
            object_type = f"{args.object_type}_bottle"
            cli.pick(object_type, show_debug=args.debug)
        
        elif args.command == 'drop':
            cli.drop(show_debug=args.debug)
        
        elif args.command == 'status':
            cli.status()
        
        elif args.command == 'home':
            cli.home()
        
        elif args.command == 'test_detection':
            # Test detection without moving arm
            print("Testing object detection (camera feed will show)...")
            print("Press 'Q' to quit, 'S' to save current frame")
            
            if not cli.open_camera():
                print("Error: Failed to open camera")
                return
            
            frame_count = 0
            while True:
                ret, frame = cli.cap.read()
                if not ret:
                    break
                
                # Detect objects
                detections = cli.object_detector.detect_bottles(frame)
                annotated = cli.object_detector.draw_detections(frame, detections)
                
                # Show detection count
                cv2.putText(annotated, f"Detections: {len(detections)}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated, "Press 'Q' to quit, 'S' to save", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show detection details
                y_offset = 90
                for i, det in enumerate(detections[:5]):  # Show first 5
                    text = f"{i+1}. {det['type']}: {det['confidence']:.2f}"
                    if det['color_match']:
                        text += " [color match]"
                    cv2.putText(annotated, text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    y_offset += 25
                
                cv2.imshow('Object Detection Test', annotated)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f'detection_test_{frame_count}.jpg'
                    cv2.imwrite(filename, annotated)
                    print(f"Saved frame to {filename}")
                    frame_count += 1
            
            cli.close_camera()
            cv2.destroyAllWindows()
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        cli.shutdown()


if __name__ == '__main__':
    main()

