#!/usr/bin/env python3
"""
AprilTag Detection for Object Localization

Uses AprilTags attached to bottles for robust 6DOF pose estimation.
Much more accurate than color-based detection.
"""

import numpy as np
import cv2
from dt_apriltags import Detector


class AprilTagDetector:
    """Detect AprilTags and estimate 6DOF pose."""
    
    def __init__(self, camera_matrix, dist_coeffs, tag_size_m=0.05):
        """
        Initialize AprilTag detector.
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Camera distortion coefficients
            tag_size_m: Physical size of AprilTag in meters (default: 5cm)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size_m
        
        # Initialize detector (36h11 family is most common)
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        # Camera parameters for pose estimation
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        self.camera_params = [fx, fy, cx, cy]
    
    def detect_tags(self, image):
        """
        Detect AprilTags in image.
        
        Args:
            image: Input image (color or grayscale)
        
        Returns:
            List of detections with pose information
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect tags
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )
        
        results = []
        for detection in detections:
            # Extract pose
            rotation_matrix = detection.pose_R
            translation = detection.pose_t.flatten()  # [x, y, z] in camera frame
            
            # Get tag center in pixel coordinates
            center = detection.center
            
            # Get tag corners
            corners = detection.corners
            
            result = {
                'tag_id': detection.tag_id,
                'center_pixel': center,  # (u, v)
                'corners': corners,  # 4x2 array of corner pixels
                'position_camera': translation,  # [x, y, z] in meters
                'rotation_matrix': rotation_matrix,  # 3x3 rotation matrix
                'hamming': detection.hamming,  # Error metric (0 is perfect)
                'decision_margin': detection.decision_margin,  # Detection quality
                'pose_err': detection.pose_err  # Pose estimation error
            }
            results.append(result)
        
        return results
    
    def detect_object_by_tag(self, image, tag_id):
        """
        Detect specific object by its AprilTag ID.
        
        Args:
            image: Input image
            tag_id: AprilTag ID to look for
        
        Returns:
            detection dict or None if not found
        """
        detections = self.detect_tags(image)
        
        for det in detections:
            if det['tag_id'] == tag_id:
                return det
        
        return None
    
    def draw_detections(self, image, detections, color=(0, 255, 0), thickness=2):
        """
        Draw detected AprilTags on image.
        
        Args:
            image: Input image
            detections: List of detections from detect_tags()
            color: Color for drawing (default: green)
            thickness: Line thickness
        
        Returns:
            Annotated image
        """
        annotated = image.copy()
        
        for det in detections:
            # Draw corners
            corners = det['corners'].astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(annotated, pt1, pt2, color, thickness)
            
            # Draw center
            center = det['center_pixel'].astype(int)
            cv2.circle(annotated, tuple(center), 5, (0, 0, 255), -1)
            
            # Draw tag ID and position
            tag_id = det['tag_id']
            pos = det['position_camera']
            text = f"ID:{tag_id} ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
            
            # Position text above tag
            text_pos = (center[0] - 50, center[1] - 20)
            cv2.putText(annotated, text, text_pos,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)
            
            # Show detection quality
            quality_text = f"H:{det['hamming']} M:{det['decision_margin']:.1f}"
            quality_pos = (center[0] - 50, center[1] + 20)
            cv2.putText(annotated, quality_text, quality_pos,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return annotated
    
    def get_bottle_position(self, image, tag_id, bottle_height=0.0):
        """
        Get bottle position from AprilTag detection.
        
        The AprilTag is assumed to be attached to the top of the bottle.
        This function returns the position of the bottle base on the platform.
        
        Args:
            image: Input image
            tag_id: AprilTag ID for the bottle
            bottle_height: Height of bottle in meters (used to calculate base position)
        
        Returns:
            (success, position_camera, detection_info)
            position_camera: [x, y, z] position of bottle BASE in camera frame
        """
        detection = self.detect_object_by_tag(image, tag_id)
        
        if detection is None:
            return False, None, None
        
        # Get tag position (at top of bottle)
        tag_pos_camera = detection['position_camera']
        
        # Calculate bottle base position
        # Tag is at top, so bottle base is bottle_height below in Z
        bottle_base_camera = tag_pos_camera.copy()
        bottle_base_camera[2] += bottle_height  # Move down in camera frame (Z is negative down)
        
        detection_info = {
            'tag_detection': detection,
            'tag_position': tag_pos_camera.tolist(),
            'bottle_base_position': bottle_base_camera.tolist(),
            'bottle_height': bottle_height
        }
        
        return True, bottle_base_camera, detection_info


def test_apriltag_detector():
    """Test AprilTag detector with webcam."""
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--calibration', type=str, 
                       default='../calibration/camera_calibration.npz',
                       help='Camera calibration file')
    parser.add_argument('--tag-size', type=float, default=0.05,
                       help='AprilTag size in meters (default: 0.05 = 5cm)')
    args = parser.parse_args()
    
    # Load camera calibration
    calib_data = np.load(args.calibration)
    camera_matrix = calib_data['camera_matrix']
    dist_coeffs = calib_data['dist_coeffs']
    
    # Initialize detector
    detector = AprilTagDetector(camera_matrix, dist_coeffs, tag_size_m=args.tag_size)
    
    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        return
    
    print("AprilTag Detector Test")
    print("=" * 60)
    print(f"Tag size: {args.tag_size}m")
    print(f"Camera matrix:\n{camera_matrix}")
    print("Press 'Q' to quit")
    print("=" * 60)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect tags
        detections = detector.detect_tags(frame)
        
        # Draw detections
        annotated = detector.draw_detections(frame, detections)
        
        # Show info
        cv2.putText(annotated, f"Tags detected: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, "Press 'Q' to quit", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Print detection info
        for det in detections:
            pos = det['position_camera']
            print(f"Tag {det['tag_id']}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), "
                  f"hamming={det['hamming']}, margin={det['decision_margin']:.1f}")
        
        cv2.imshow('AprilTag Detection', annotated)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    test_apriltag_detector()

