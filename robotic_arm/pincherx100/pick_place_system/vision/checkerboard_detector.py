#!/usr/bin/env python3
"""
Checkerboard Detection for Drop Location

Detects the 6x7 checkerboard pattern to identify drop location.
"""

import cv2
import numpy as np


class CheckerboardDetector:
    """
    Detects checkerboard pattern for drop location identification.
    """
    
    def __init__(self, pattern_size=(6, 7), square_size=24.0):
        """
        Initialize checkerboard detector.
        
        Args:
            pattern_size: (width, height) of inner corners
            square_size: Size of each square in mm (default: 24mm for large pattern)
        """
        self.pattern_size = pattern_size
        self.square_size = square_size / 1000.0  # Convert to meters
        
        # Prepare object points (3D points in checkerboard frame)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        print(f"Checkerboard detector initialized")
        print(f"  Pattern: {pattern_size[0]}x{pattern_size[1]} corners")
        print(f"  Square size: {square_size}mm")
    
    def detect(self, image, camera_matrix, dist_coeffs):
        """
        Detect checkerboard in image.
        
        Args:
            image: BGR image
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Distortion coefficients
        
        Returns:
            (success, center_pixel, center_3d, pose_data)
            - success: True if checkerboard detected
            - center_pixel: [cx, cy] pixel coordinates of center
            - center_3d: [X, Y, Z] 3D coordinates in camera frame (meters)
            - pose_data: Dictionary with pose information
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(
            gray,
            self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        if not ret:
            return False, None, None, None
        
        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # Calculate pose
        success, rvec, tvec = cv2.solvePnP(
            self.objp,
            corners_refined,
            camera_matrix,
            dist_coeffs
        )
        
        if not success:
            return False, None, None, None
        
        # Calculate center of checkerboard in pixel coordinates
        # Center is at pattern_size[0]/2, pattern_size[1]/2 in checkerboard frame
        center_obj = np.array([
            (self.pattern_size[0] - 1) / 2 * self.square_size,
            (self.pattern_size[1] - 1) / 2 * self.square_size,
            0.0
        ])
        
        # Project center to image
        center_2d, _ = cv2.projectPoints(
            center_obj.reshape(1, 1, 3),
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs
        )
        center_pixel = center_2d[0][0]
        
        # Get 3D center in camera frame
        R, _ = cv2.Rodrigues(rvec)
        center_3d_cam = R @ center_obj + tvec.flatten()
        
        pose_data = {
            'rvec': rvec,
            'tvec': tvec,
            'R': R,
            'corners': corners_refined
        }
        
        return True, center_pixel, center_3d_cam, pose_data
    
    def draw_detection(self, image, center_pixel, pose_data):
        """
        Draw checkerboard detection on image.
        
        Args:
            image: BGR image
            center_pixel: [cx, cy] center pixel coordinates
            pose_data: Pose data from detect()
        
        Returns:
            annotated_image: Image with detection drawn
        """
        annotated = image.copy()
        
        if pose_data is None:
            return annotated
        
        # Draw checkerboard corners
        cv2.drawChessboardCorners(
            annotated,
            self.pattern_size,
            pose_data['corners'],
            True
        )
        
        # Draw center point
        if center_pixel is not None:
            cx, cy = map(int, center_pixel)
            cv2.circle(annotated, (cx, cy), 10, (0, 255, 0), -1)
            cv2.circle(annotated, (cx, cy), 15, (0, 255, 0), 2)
            
            # Draw label
            cv2.putText(annotated, "Drop Location", (cx - 50, cy - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw axis
        camera_matrix = np.eye(3)  # Will be provided by caller
        dist_coeffs = np.zeros(4)
        axis_length = self.square_size * 3 * 1000  # Convert to mm
        
        # Note: camera_matrix and dist_coeffs should be passed as parameters
        # For now, this is a placeholder
        
        return annotated


if __name__ == '__main__':
    # Test checkerboard detector
    from coordinate_transform import CoordinateTransformer
    
    transformer = CoordinateTransformer("../camera_calibration/camera_calibration.npz")
    detector = CheckerboardDetector(pattern_size=(6, 7), square_size=24.0)
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Testing checkerboard detection. Press 'Q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        success, center_pixel, center_3d, pose_data = detector.detect(
            frame,
            transformer.camera_matrix,
            transformer.dist_coeffs
        )
        
        if success:
            annotated = detector.draw_detection(frame, center_pixel, pose_data)
            cv2.putText(annotated, "Checkerboard: DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            annotated = frame
            cv2.putText(annotated, "Checkerboard: NOT FOUND", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow('Checkerboard Detection Test', annotated)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

