#!/usr/bin/env python3
"""
Three Bottle Pick and Place System
AprilTag-Based with 1-inch tags

Bottles: Orange, Apple, Yogurt
AprilTag Size: 1.0 inch (0.0254 meters)

Usage:
    python pick_place.py detect          # Detect all bottles
    python pick_place.py pick orange     # Pick orange bottle (tag 0)
    python pick_place.py pick apple      # Pick apple bottle (tag 1)
    python pick_place.py pick yogurt     # Pick yogurt bottle (tag 2)
    python pick_place.py place orange    # Place at orange drop zone (tag 10)
    python pick_place.py move orange apple  # Move orange to apple zone
    python pick_place.py status          # Show system status
    python pick_place.py home            # Move to home position
    python pick_place.py pos home        # Move to home position (center)
    python pick_place.py pos retract     # Move to retract position (fully retracted)
"""

import sys
import os
import argparse
import cv2
import yaml
import numpy as np
import logging
from datetime import datetime

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'low-level_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))

from apriltag_detector import AprilTagDetector
from coordinate_transform import CoordinateTransformer
from arm_controller_wrapper import PickPlaceController


class ThreeBottlePickPlace:
    """
    Pick and place system for three bottles using AprilTags.
    
    Bottles:
        - Orange (Tag ID 0, Drop Zone Tag ID 10)
        - Apple (Tag ID 1, Drop Zone Tag ID 11)
        - Yogurt (Tag ID 2, Drop Zone Tag ID 12)
    
    AprilTag Size: 1.0 inch (25.4mm)
    """
    
    # Bottle name to tag ID mapping
    BOTTLE_TAGS = {
        'orange': 0,
        'apple': 1,
        'yogurt': 2
    }
    
    # Drop zone mapping
    DROP_ZONE_TAGS = {
        'orange': 10,
        'apple': 11,
        'yogurt': 12
    }
    
    def __init__(self, config_file=None):
        """
        Initialize the three-bottle pick and place system.
        
        Args:
            config_file: Path to robot configuration YAML file
        """
        # Load configuration
        if config_file is None:
            config_file = os.path.join(
                os.path.dirname(__file__), '..', 'configs', 'robot_config.yaml'
            )
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Setup logging
        self._setup_logging()
        self.logger.info("Initializing Three Bottle Pick and Place System")
        self.logger.info("Bottles: Orange, Apple, Yogurt")
        self.logger.info("AprilTag Size: 1.0 inch (25.4mm)")
        
        # Load camera calibration
        camera_calib_path = os.path.join(
            os.path.dirname(__file__), '..', 
            self.config['camera']['calibration_file']
        )
        
        # Check if calibration file exists
        if not os.path.exists(camera_calib_path):
            self.logger.warning(f"Camera calibration file not found: {camera_calib_path}")
            self.logger.info("Using default camera parameters. Please calibrate camera for accurate results.")
            # Create default calibration
            self.camera_matrix = np.array([[800, 0, 640], [0, 800, 360], [0, 0, 1]], dtype=np.float32)
            self.dist_coeffs = np.zeros((5,), dtype=np.float32)
        else:
            calib_data = np.load(camera_calib_path)
            self.camera_matrix = calib_data['camera_matrix']
            self.dist_coeffs = calib_data['dist_coeffs']
        
        # Initialize AprilTag detector with 1-inch tags
        tag_size = self.config['apriltags']['tag_size']
        self.logger.info(f"Initializing AprilTag detector with tag size: {tag_size}m ({tag_size*39.37:.1f} inches)")
        
        self.tag_detector = AprilTagDetector(
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            tag_size_m=tag_size
        )
        
        # Initialize coordinate transformer
        if os.path.exists(camera_calib_path):
            self.transformer = CoordinateTransformer(camera_calib_path, config_file)
        else:
            self.logger.warning("Coordinate transformer initialized without calibration")
            self.transformer = None
        
        # Initialize arm controller
        self.arm_controller = PickPlaceController()
        
        # Camera
        self.camera_index = 0
        self.cap = None
        
        # Tag mapping
        self.tag_mapping = self.config['apriltags']['tag_mapping']
        
        self.logger.info("System initialized successfully")
    
    def _setup_logging(self):
        """Setup logging configuration."""
        log_config = self.config.get('logging', {})
        
        if log_config.get('enabled', True):
            log_dir = os.path.join(os.path.dirname(__file__), '..', log_config.get('log_directory', 'logs'))
            os.makedirs(log_dir, exist_ok=True)
            
            log_level = getattr(logging, log_config.get('log_level', 'INFO'))
            
            self.logger = logging.getLogger('ThreeBottlePickPlace')
            self.logger.setLevel(log_level)
            
            # File handler
            log_file = os.path.join(
                log_dir, 
                f'system_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
            )
            fh = logging.FileHandler(log_file)
            fh.setLevel(log_level)
            
            # Console handler
            ch = logging.StreamHandler()
            ch.setLevel(log_level)
            
            # Formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            
            self.logger.addHandler(fh)
            self.logger.addHandler(ch)
        else:
            self.logger = logging.getLogger('ThreeBottlePickPlace')
            self.logger.addHandler(logging.NullHandler())
    
    def open_camera(self):
        """Open camera for image capture."""
        if self.cap and self.cap.isOpened():
            return True
            
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.logger.error(f"Failed to open camera {self.camera_index}")
            return False
        
        width = self.config['camera']['image']['width']
        height = self.config['camera']['image']['height']
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        self.logger.debug(f"Camera opened: {width}x{height}")
        return True
    
    def close_camera(self):
        """Close camera."""
        if self.cap:
            self.cap.release()
            self.cap = None
            self.logger.debug("Camera closed")
    
    def capture_frame(self):
        """
        Capture frame from camera.
        
        Returns:
            numpy.ndarray: Captured frame, or None if failed
        """
        if not self.open_camera():
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            self.logger.error("Failed to capture frame")
            return None
        
        return frame
    
    def detect_bottle(self, bottle_name, max_attempts=10, show_visualization=False):
        """
        Detect bottle by name.
        
        Args:
            bottle_name: 'orange', 'apple', or 'yogurt'
            max_attempts: Maximum detection attempts
            show_visualization: Show detection visualization
        
        Returns:
            dict: Detection data or None if not found
        """
        if bottle_name not in self.BOTTLE_TAGS:
            self.logger.error(f"Invalid bottle name: {bottle_name}. Must be 'orange', 'apple', or 'yogurt'")
            return None
        
        tag_id = self.BOTTLE_TAGS[bottle_name]
        return self.detect_tag(tag_id, max_attempts, show_visualization)
    
    def detect_tag(self, tag_id, max_attempts=10, show_visualization=False):
        """
        Detect specific AprilTag.
        
        Args:
            tag_id: AprilTag ID to detect
            max_attempts: Maximum detection attempts
            show_visualization: Show detection visualization
        
        Returns:
            dict: Detection data or None if not found
        """
        self.logger.info(f"Detecting AprilTag ID {tag_id}")
        
        min_margin = self.config['apriltags']['min_decision_margin']
        max_hamming = self.config['apriltags']['max_hamming_error']
        
        for attempt in range(max_attempts):
            frame = self.capture_frame()
            if frame is None:
                continue
            
            # Detect all tags
            detections = self.tag_detector.detect_tags(frame)
            
            # Find target tag
            target_detection = None
            for det in detections:
                if det['tag_id'] == tag_id:
                    target_detection = det
                    break
            
            if target_detection is None:
                if attempt < max_attempts - 1:
                    self.logger.debug(
                        f"Tag {tag_id} not found, attempt {attempt+1}/{max_attempts}"
                    )
                    continue
                else:
                    self.logger.warning(
                        f"Tag {tag_id} not detected after {max_attempts} attempts"
                    )
                    return None
            
            # Check detection quality
            if target_detection['hamming'] > max_hamming:
                self.logger.warning(
                    f"Tag {tag_id} Hamming error: {target_detection['hamming']} "
                    f"(max: {max_hamming})"
                )
            
            if target_detection['decision_margin'] < min_margin:
                self.logger.warning(
                    f"Tag {tag_id} decision margin: "
                    f"{target_detection['decision_margin']:.1f} (min: {min_margin})"
                )
            
            # Convert camera frame position to base frame
            pos_camera = target_detection['position_camera']
            if self.transformer:
                pos_base = self.transformer.camera_3d_to_base_3d(pos_camera)
                
                # Apply coordinate offsets if configured
                coord_config = self.config.get('coordinate_transform', {})
                x_offset = coord_config.get('x_offset', 0.0)
                y_offset = coord_config.get('y_offset', 0.0)
                z_offset = coord_config.get('z_offset', 0.0)
                
                if x_offset != 0.0 or y_offset != 0.0 or z_offset != 0.0:
                    pos_base[0] += x_offset
                    pos_base[1] += y_offset
                    pos_base[2] += z_offset
                    self.logger.debug(f"Applied offsets - X:{x_offset}m, Y:{y_offset}m, Z:{z_offset}m")
            else:
                # If no transformer, use camera position as-is (not accurate!)
                pos_base = pos_camera
                self.logger.warning("No coordinate transformer available. Using camera coordinates.")
            
            # Get object name if mapped
            object_name = self.tag_mapping.get(tag_id, f"tag_{tag_id}")
            
            detection_data = {
                'tag_id': tag_id,
                'object_name': object_name,
                'position_camera': pos_camera,
                'position_base': pos_base,
                'rotation_matrix': target_detection['rotation_matrix'],
                'center_pixel': target_detection['center_pixel'],
                'quality': {
                    'hamming': target_detection['hamming'],
                    'decision_margin': target_detection['decision_margin'],
                    'pose_error': target_detection['pose_err']
                }
            }
            
            self.logger.info(
                f"Tag {tag_id} ({object_name}) detected at base: "
                f"({pos_base[0]:.3f}, {pos_base[1]:.3f}, {pos_base[2]:.3f}) m"
            )
            
            # Show visualization if requested
            if show_visualization:
                self._show_detection_visualization(frame, detections, tag_id)
            
            return detection_data
        
        return None
    
    def _show_detection_visualization(self, frame, detections, highlight_tag_id=None):
        """Show detection visualization."""
        annotated = self.tag_detector.draw_detections(frame, detections)
        
        # Add title
        cv2.putText(
            annotated, 
            f"Three Bottle System - Target: {highlight_tag_id}", 
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        
        # Highlight target tag
        if highlight_tag_id is not None:
            for det in detections:
                if det['tag_id'] == highlight_tag_id:
                    corners = det['corners'].astype(int)
                    cv2.polylines(
                        annotated, [corners], True, (0, 255, 0), 4
                    )
        
        cv2.putText(
            annotated, 
            "Press any key to continue...", 
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )
        
        cv2.imshow('AprilTag Detection', annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def pick_bottle(self, bottle_name, show_visualization=False, debug_step=None):
        """
        Pick bottle by name.
        
        Args:
            bottle_name: 'orange', 'apple', or 'yogurt'
            show_visualization: Show detection visualization
            debug_step: Debug step number (1-5) to stop at
        
        Returns:
            bool: True if pick successful
        """
        if bottle_name not in self.BOTTLE_TAGS:
            self.logger.error(f"Invalid bottle name: {bottle_name}")
            return False
        
        tag_id = self.BOTTLE_TAGS[bottle_name]
        return self.pick_by_tag(tag_id, show_visualization, debug_step)
    
    def pick_by_tag(self, tag_id, show_visualization=False, debug_step=None):
        """
        Pick object identified by AprilTag.
        
        Args:
            tag_id: AprilTag ID mounted on object
            show_visualization: Show detection visualization
            debug_step: Debug step number (1-5) to stop at that step
        
        Returns:
            bool: True if pick successful
        """
        self.logger.info(f"Pick operation: tag {tag_id}")
        if debug_step:
            print(f"\n[DEBUG MODE] Will stop after step {debug_step}")
        
        # Initialize arm if needed
        if not self.arm_controller.initialized:
            self.logger.info("Initializing arm controller")
            if not self.arm_controller.initialize():
                self.logger.error("Failed to initialize arm")
                return False
        
        # STEP 1: Detect tag and calculate positions
        print("\n" + "="*60)
        print("STEP 1: DETECTION AND POSITION CALCULATION")
        print("="*60)
        
        detection = self.detect_tag(tag_id, show_visualization=show_visualization)
        if detection is None:
            self.logger.error(f"Cannot pick: tag {tag_id} not detected")
            return False
        
        # Get object configuration
        object_name = detection['object_name']
        object_config = self.config['objects'].get(object_name, {})
        
        # Get object height
        object_height = object_config.get('height', 0.15)
        grip_ratio = object_config.get('grip_height_ratio', 0.5)
        tag_z_offset = object_config.get('tag_offset_z', 0.0)
        
        # Print debug info for detection
        print(f"\nDetected Tag ID: {tag_id}")
        print(f"Object Name: {object_name}")
        print(f"Object Height: {object_height*100:.1f}cm")
        print(f"Grip Ratio: {grip_ratio} (grip at {grip_ratio*100:.0f}% of height)")
        print(f"\nTag Position (camera frame):")
        print(f"  X: {detection['position_camera'][0]:.4f}m")
        print(f"  Y: {detection['position_camera'][1]:.4f}m")
        print(f"  Z: {detection['position_camera'][2]:.4f}m")
        print(f"\nTag Position (base frame - RAW):")
        print(f"  X: {detection['position_base'][0]:.4f}m")
        print(f"  Y: {detection['position_base'][1]:.4f}m")
        print(f"  Z: {detection['position_base'][2]:.4f}m")
        
        # Calculate grip point
        position_base = detection['position_base'].copy()
        
        # Tag is on top of object, grip at middle
        grip_height = object_height * grip_ratio
        position_base[2] = position_base[2] - (object_height - grip_height) - tag_z_offset
        
        print(f"\nCalculated Grip Point:")
        print(f"  Grip Height: {grip_height*100:.1f}cm above platform")
        print(f"  Grip Position (base frame):")
        print(f"    X: {position_base[0]:.4f}m")
        print(f"    Y: {position_base[1]:.4f}m")
        print(f"    Z: {position_base[2]:.4f}m")
        
        self.logger.info(
            f"Grip point: ({position_base[0]:.3f}, {position_base[1]:.3f}, "
            f"{position_base[2]:.3f}) m"
        )
        
        if debug_step == 1:
            print("\n[DEBUG] Stopped after Step 1 - Detection and Position Calculation")
            print("="*60)
            return True
        
        # STEP 2: Move to approach position and open gripper
        print("\n" + "="*60)
        print("STEP 2: APPROACH POSITION AND OPEN GRIPPER")
        print("="*60)
        
        pick_params = self.config['pick_place']
        approach_pos = [position_base[0], position_base[1], grip_height + pick_params['approach_height']]
        
        print(f"\nApproach Position:")
        print(f"  X: {approach_pos[0]:.4f}m")
        print(f"  Y: {approach_pos[1]:.4f}m")
        print(f"  Z: {approach_pos[2]:.4f}m (grip_height + {pick_params['approach_height']*100:.1f}cm)")
        
        print(f"\nMoving to approach position (horizontal wrist approach)...")
        if not self.arm_controller.move_to_position(approach_pos, orientation='horizontal', elbow='up', use_horizontal_wrist=True):
            print("  ✗ Failed to move to approach position")
            return False
        print("  ✓ Reached approach position")
        
        import time
        time.sleep(3.0)  # Increased wait time for slower movements
        
        print(f"\nOpening gripper...")
        self.arm_controller.open_gripper()
        time.sleep(1.0)  # Increased for reliability
        print("  ✓ Gripper opened")
        
        if debug_step == 2:
            print("\n[DEBUG] Stopped after Step 2 - Approach and Open Gripper")
            print("="*60)
            return True
        
        # STEP 3: Descend to grip position
        print("\n" + "="*60)
        print("STEP 3: DESCEND TO GRIP POSITION")
        print("="*60)
        
        grip_pos = [position_base[0], position_base[1], grip_height]
        print(f"\nGrip Position:")
        print(f"  X: {grip_pos[0]:.4f}m")
        print(f"  Y: {grip_pos[1]:.4f}m")
        print(f"  Z: {grip_pos[2]:.4f}m")
        
        print(f"\nDescending to grip position (horizontal wrist)...")
        if not self.arm_controller.move_to_position(grip_pos, orientation='horizontal', elbow='up', use_horizontal_wrist=True):
            print("  ✗ Failed to move to grip position")
            return False
        print("  ✓ Reached grip position")
        
        time.sleep(3.0)  # Increased wait time for slower movements
        
        if debug_step == 3:
            print("\n[DEBUG] Stopped after Step 3 - Descend to Grip")
            print("="*60)
            return True
        
        # STEP 4: Close gripper
        print("\n" + "="*60)
        print("STEP 4: CLOSE GRIPPER")
        print("="*60)
        
        print(f"\nClosing gripper...")
        self.arm_controller.close_gripper()
        time.sleep(2.0)  # Increased for reliable grip
        print("  ✓ Gripper closed")
        
        if debug_step == 4:
            print("\n[DEBUG] Stopped after Step 4 - Close Gripper")
            print("="*60)
            return True
        
        # STEP 5: Lift object
        print("\n" + "="*60)
        print("STEP 5: LIFT OBJECT")
        print("="*60)
        
        lift_pos = [position_base[0], position_base[1], pick_params['lift_height']]
        print(f"\nLift Position:")
        print(f"  X: {lift_pos[0]:.4f}m")
        print(f"  Y: {lift_pos[1]:.4f}m")
        print(f"  Z: {lift_pos[2]:.4f}m ({pick_params['lift_height']*100:.1f}cm above platform)")
        
        print(f"\nLifting object (horizontal wrist)...")
        if not self.arm_controller.move_to_position(lift_pos, orientation='horizontal', elbow='up', use_horizontal_wrist=True):
            print("  ✗ Failed to lift object")
            return False
        print("  ✓ Object lifted")
        
        time.sleep(3.0)  # Increased wait time for slower movements
        
        if debug_step == 5:
            print("\n[DEBUG] Stopped after Step 5 - Lift Object")
            print("="*60)
            return True
        
        print("\n" + "="*60)
        print("✓ PICK OPERATION COMPLETE")
        print("="*60)
        
        if True:
            self.logger.info(f"Successfully picked tag {tag_id} ({object_name})")
        else:
            self.logger.error(f"Failed to pick tag {tag_id} ({object_name})")
        
        return True
    
    def place_at_zone(self, zone_name, show_visualization=False):
        """
        Place object at drop zone by name.
        
        Args:
            zone_name: 'orange', 'apple', or 'yogurt'
            show_visualization: Show detection visualization
        
        Returns:
            bool: True if place successful
        """
        if zone_name not in self.DROP_ZONE_TAGS:
            self.logger.error(f"Invalid zone name: {zone_name}")
            return False
        
        tag_id = self.DROP_ZONE_TAGS[zone_name]
        return self.place_at_tag(tag_id, show_visualization)
    
    def place_at_tag(self, tag_id, show_visualization=False):
        """
        Place object at location marked by AprilTag.
        
        Args:
            tag_id: AprilTag ID marking drop location
            show_visualization: Show detection visualization
        
        Returns:
            bool: True if place successful
        """
        self.logger.info(f"Place operation: at tag {tag_id}")
        
        if not self.arm_controller.initialized:
            self.logger.error("Arm not initialized")
            return False
        
        # Detect target tag
        detection = self.detect_tag(tag_id, show_visualization=show_visualization)
        if detection is None:
            self.logger.error(f"Cannot place: tag {tag_id} not detected")
            return False
        
        # Get place parameters
        place_params = self.config['pick_place']
        drop_height = place_params['drop_height']
        
        # Use tag position as drop location
        position_base = detection['position_base']
        
        # Execute place
        success = self.arm_controller.place_object(
            position_base, 
            drop_height=drop_height
        )
        
        if success:
            self.logger.info(f"Successfully placed at tag {tag_id}")
        else:
            self.logger.error(f"Failed to place at tag {tag_id}")
        
        return success
    
    def move_bottle(self, source_bottle, target_zone, show_visualization=False):
        """
        Move bottle from source to target zone.
        
        Args:
            source_bottle: 'orange', 'apple', or 'yogurt'
            target_zone: 'orange', 'apple', or 'yogurt'
            show_visualization: Show detection visualization
        
        Returns:
            bool: True if move successful
        """
        self.logger.info(f"Move operation: {source_bottle} -> {target_zone} zone")
        
        # Pick from source
        if not self.pick_bottle(source_bottle, show_visualization=show_visualization):
            return False
        
        # Place at target
        if not self.place_at_zone(target_zone, show_visualization=show_visualization):
            return False
        
        self.logger.info("Move operation completed successfully")
        return True
    
    def detect_all_bottles(self, show_visualization=True):
        """
        Detect all three bottles.
        
        Args:
            show_visualization: Show detection visualization
        
        Returns:
            dict: Detection results for each bottle
        """
        print("\n" + "="*60)
        print("DETECTING ALL BOTTLES")
        print("="*60)
        
        frame = self.capture_frame()
        if frame is None:
            print("Failed to capture frame")
            return {}
        
        # Detect all tags
        detections = self.tag_detector.detect_tags(frame)
        
        results = {}
        for bottle_name, tag_id in self.BOTTLE_TAGS.items():
            for det in detections:
                if det['tag_id'] == tag_id:
                    pos_camera = det['position_camera']
                    if self.transformer:
                        pos_base = self.transformer.camera_3d_to_base_3d(pos_camera)
                    else:
                        pos_base = pos_camera
                    
                    results[bottle_name] = {
                        'tag_id': tag_id,
                        'position_base': pos_base,
                        'position_camera': pos_camera,
                        'quality': {
                            'hamming': det['hamming'],
                            'decision_margin': det['decision_margin']
                        }
                    }
                    
                    print(f"\n{bottle_name.upper()} Bottle (Tag {tag_id}):")
                    print(f"  Position: ({pos_base[0]:.3f}, {pos_base[1]:.3f}, {pos_base[2]:.3f}) m")
                    print(f"  Quality: H={det['hamming']}, M={det['decision_margin']:.1f}")
        
        # Show visualization
        if show_visualization and len(detections) > 0:
            annotated = self.tag_detector.draw_detections(frame, detections)
            cv2.putText(
                annotated, 
                "Three Bottle System - All Detections", 
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
            cv2.imshow('Bottle Detection', annotated)
            print("\nPress any key to close visualization...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        print("="*60)
        print(f"Detected {len(results)}/3 bottles")
        print("="*60 + "\n")
        
        return results
    
    def show_status(self):
        """Display system status."""
        print("\n" + "=" * 60)
        print("THREE BOTTLE PICK AND PLACE SYSTEM STATUS")
        print("=" * 60)
        
        # System info
        print("\nBottle Configuration:")
        print("  Orange Bottle - Tag ID: 0, Drop Zone: Tag ID 10")
        print("  Apple Bottle  - Tag ID: 1, Drop Zone: Tag ID 11")
        print("  Yogurt Bottle - Tag ID: 2, Drop Zone: Tag ID 12")
        print(f"\nAprilTag Size: {self.config['apriltags']['tag_size']}m (1.0 inch)")
        
        # Arm status
        if self.arm_controller.initialized:
            positions = self.arm_controller.get_current_joint_positions()
            if positions:
                print(f"\nArm: Initialized")
                print(f"  Joint positions: {positions}")
            else:
                print("\nArm: Initialized (cannot read positions)")
        else:
            print("\nArm: Not initialized")
        
        # Camera status
        if self.cap and self.cap.isOpened():
            print("Camera: Open")
        else:
            print("Camera: Closed")
        
        print("=" * 60)
    
    def move_home(self):
        """Move arm to home position."""
        self.logger.info("Moving to home position")
        
        if not self.arm_controller.initialized:
            self.logger.info("Initializing arm controller")
            if not self.arm_controller.initialize():
                self.logger.error("Failed to initialize arm")
                return False
        
        success = self.arm_controller.move_to_home()
        if success:
            self.logger.info("Moved to home position")
            print("Arm moved to home position")
        else:
            self.logger.error("Failed to move to home position")
            print("Failed to move to home position")
        
        return success
    
    def move_to_position(self, position_name):
        """
        Move arm to named position.
        
        Positions are loaded from configs/robot_config.yaml under 'positions' section.
        
        Args:
            position_name: Position name (e.g., 'home', 'retract')
        
        Returns:
            bool: True if successful
        """
        self.logger.info(f"Moving to {position_name} position")
        
        if not self.arm_controller.initialized:
            self.logger.info("Initializing arm controller")
            if not self.arm_controller.initialize():
                self.logger.error("Failed to initialize arm")
                return False
        
        # Get positions from config file
        positions_config = self.config.get('positions', {})
        
        if not positions_config:
            self.logger.error("No positions defined in config file")
            print("Error: No positions defined in configs/robot_config.yaml")
            print("Please add a 'positions' section to the config file.")
            return False
        
        if position_name not in positions_config:
            self.logger.error(f"Unknown position: {position_name}")
            print(f"Error: Unknown position '{position_name}'")
            print(f"Available positions: {', '.join(positions_config.keys())}")
            print(f"Define positions in configs/robot_config.yaml")
            return False
        
        pos_config = positions_config[position_name]
        pos_values = pos_config.get('values')
        pos_description = pos_config.get('description', 'No description')
        
        if not pos_values or len(pos_values) != 4:
            self.logger.error(f"Invalid position values for '{position_name}'")
            print(f"Error: Position '{position_name}' must have 4 values [base, shoulder, elbow, wrist]")
            return False
        
        print(f"Moving to {position_name} position...")
        print(f"  Description: {pos_description}")
        print(f"  Servo values: base={pos_values[0]}, shoulder={pos_values[1]}, "
              f"elbow={pos_values[2]}, wrist={pos_values[3]}")
        
        success = self.arm_controller.move_to_joint_positions(pos_values)
        
        if success:
            self.logger.info(f"Moved to {position_name} position")
            print(f"✓ Arm moved to {position_name} position")
        else:
            self.logger.error(f"Failed to move to {position_name} position")
            print(f"✗ Failed to move to {position_name} position")
        
        return success
    
    def shutdown(self):
        """Shutdown system and release resources."""
        self.logger.info("Shutting down system")
        self.close_camera()
        if self.arm_controller.initialized:
            self.arm_controller.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Three Bottle Pick and Place System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python pick_place.py detect
  python pick_place.py pick orange
  python pick_place.py pick apple
  python pick_place.py pick yogurt
  python pick_place.py place orange
  python pick_place.py move orange apple
  python pick_place.py status
  python pick_place.py home
  python pick_place.py pos home
  python pick_place.py pos retract
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Detect command
    detect_parser = subparsers.add_parser('detect', help='Detect all bottles')
    detect_parser.add_argument(
        '--no-display', action='store_true', help='Disable visualization'
    )
    
    # Pick command
    pick_parser = subparsers.add_parser('pick', help='Pick bottle')
    pick_parser.add_argument(
        'bottle', choices=['orange', 'apple', 'yogurt'], help='Bottle to pick'
    )
    pick_parser.add_argument(
        '--show', action='store_true', help='Show detection visualization'
    )
    pick_parser.add_argument(
        '--debug', type=int, choices=[1, 2, 3, 4, 5], metavar='STEP',
        help='Debug mode: stop after step (1=detect, 2=approach, 3=descend, 4=grip, 5=lift)'
    )
    
    # Place command
    place_parser = subparsers.add_parser('place', help='Place at drop zone')
    place_parser.add_argument(
        'zone', choices=['orange', 'apple', 'yogurt'], help='Drop zone'
    )
    place_parser.add_argument(
        '--show', action='store_true', help='Show detection visualization'
    )
    
    # Move command
    move_parser = subparsers.add_parser('move', help='Move bottle from source to target')
    move_parser.add_argument(
        'source', choices=['orange', 'apple', 'yogurt'], help='Source bottle'
    )
    move_parser.add_argument(
        'target', choices=['orange', 'apple', 'yogurt'], help='Target zone'
    )
    move_parser.add_argument(
        '--show', action='store_true', help='Show detection visualization'
    )
    
    # Status command
    subparsers.add_parser('status', help='Show system status')
    
    # Home command
    subparsers.add_parser('home', help='Move to home position')
    
    # Position command
    pos_parser = subparsers.add_parser('pos', help='Move to named position')
    pos_parser.add_argument(
        'position', choices=['home', 'retract'], 
        help='Position name (home=center, retract=fully retracted)'
    )
    
    # Config file argument
    parser.add_argument(
        '--config', type=str, default=None, help='Path to configuration file'
    )
    
    args = parser.parse_args()
    
    if args.command is None:
        parser.print_help()
        return 1
    
    # Initialize system
    try:
        system = ThreeBottlePickPlace(config_file=args.config)
    except Exception as e:
        print(f"Error initializing system: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    try:
        # Execute command
        if args.command == 'detect':
            success = system.detect_all_bottles(show_visualization=(not args.no_display))
            return 0 if success else 1
        
        elif args.command == 'pick':
            debug_step = args.debug if hasattr(args, 'debug') else None
            success = system.pick_bottle(
                args.bottle, 
                show_visualization=args.show,
                debug_step=debug_step
            )
            return 0 if success else 1
        
        elif args.command == 'place':
            success = system.place_at_zone(args.zone, show_visualization=args.show)
            return 0 if success else 1
        
        elif args.command == 'move':
            success = system.move_bottle(args.source, args.target, show_visualization=args.show)
            return 0 if success else 1
        
        elif args.command == 'status':
            system.show_status()
            return 0
        
        elif args.command == 'home':
            success = system.move_home()
            return 0 if success else 1
        
        elif args.command == 'pos':
            success = system.move_to_position(args.position)
            return 0 if success else 1
    
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
        return 1
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        system.shutdown()


if __name__ == '__main__':
    sys.exit(main())

