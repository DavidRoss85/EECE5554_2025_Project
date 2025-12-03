#!/usr/bin/env python3
"""
Object Detection Module using YOLO

Detects orange juice bottles and yogurt bottles using YOLOv8.
Uses color filtering to improve detection accuracy.
"""

import cv2
import numpy as np
from ultralytics import YOLO
import os

# Import cap detector for top-down views
try:
    from cap_detector import CapDetector
    CAP_DETECTOR_AVAILABLE = True
except ImportError:
    CAP_DETECTOR_AVAILABLE = False


class ObjectDetector:
    """
    Object detector for bottles using YOLO and color filtering.
    For top-down views, consider using CapDetector instead.
    """
    
    def __init__(self, model_path='yolov8n.pt', confidence_threshold=0.3, use_cap_detector=False):
        """
        Initialize object detector.
        
        Args:
            model_path: Path to YOLO model file (or model name like 'yolov8n.pt')
                       Options: 'yolov8n.pt' (nano, fastest), 'yolov8s.pt' (small, better accuracy),
                               'yolov8m.pt' (medium), 'yolov8l.pt' (large), 'yolov8x.pt' (extra large)
            confidence_threshold: Minimum confidence for detections (default: 0.3 for better recall)
        """
        # Load YOLO model
        try:
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
            else:
                # Try to download/use pretrained model
                print(f"Model {model_path} not found locally, downloading...")
                self.model = YOLO(model_path)  # Will download if not found
        except Exception as e:
            print(f"Error loading YOLO model {model_path}: {e}")
            print("Falling back to yolov8n.pt")
            self.model = YOLO('yolov8n.pt')
        
        self.confidence_threshold = confidence_threshold
        self.use_cap_detector = use_cap_detector
        
        # Initialize cap detector if requested (for top-down views)
        self.cap_detector = None
        if use_cap_detector and CAP_DETECTOR_AVAILABLE:
            # Cap detector will be initialized with camera calibration later
            self.cap_detector = None  # Will be set when camera calibration is available
        
        # Object classes we're interested in
        # YOLO COCO classes: bottle=39, cup=41
        self.target_classes = [39]  # bottle class
        
        # Color ranges for filtering (HSV)
        self.color_ranges = {
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            'pink': {
                'lower': np.array([140, 50, 50]),
                'upper': np.array([170, 255, 255])
            },
            'orange': {
                'lower': np.array([5, 50, 50]),
                'upper': np.array([25, 255, 255])
            }
        }
        
        print(f"Object detector initialized with model: {model_path}")
        print(f"  Confidence threshold: {confidence_threshold}")
    
    def detect_objects(self, image):
        """
        Detect objects in image using YOLO.
        
        Args:
            image: BGR image (numpy array)
        
        Returns:
            results: YOLO results object
        """
        results = self.model(image, verbose=False, conf=self.confidence_threshold)
        return results
    
    def filter_by_color(self, image, bbox, color_name):
        """
        Filter detection by color in bounding box region.
        
        Args:
            image: BGR image
            bbox: Bounding box [x1, y1, x2, y2]
            color_name: 'green', 'pink', or 'orange'
        
        Returns:
            has_color: True if color is present in bbox
            color_ratio: Ratio of pixels matching color
        """
        if color_name not in self.color_ranges:
            return False, 0.0
        
        # Extract region of interest
        x1, y1, x2, y2 = map(int, bbox)
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(image.shape[1], x2)
        y2 = min(image.shape[0], y2)
        
        if x2 <= x1 or y2 <= y1:
            return False, 0.0
        
        roi = image[y1:y2, x1:x2]
        
        if roi.size == 0:
            return False, 0.0
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create mask for color
        color_range = self.color_ranges[color_name]
        mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # Calculate ratio
        color_ratio = np.sum(mask > 0) / mask.size
        
        # Lower threshold for better detection (5% of pixels)
        return color_ratio > 0.05, color_ratio
    
    def set_cap_detector(self, camera_matrix=None, dist_coeffs=None, square_size_mm=24.0):
        """
        Initialize cap detector with camera calibration.
        Use this for top-down views where only caps are visible.
        
        Args:
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Camera distortion coefficients
            square_size_mm: Checkerboard square size in mm (for scale reference)
        """
        if CAP_DETECTOR_AVAILABLE and self.use_cap_detector:
            from cap_detector import CapDetector
            self.cap_detector = CapDetector(
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                square_size_mm=square_size_mm
            )
            print("Cap detector initialized for top-down view")
    
    def detect_bottles(self, image):
        """
        Detect orange juice and yogurt bottles in image.
        
        For top-down views, uses cap detection if enabled.
        Otherwise uses YOLO detection.
        
        Args:
            image: BGR image
        
        Returns:
            detections: List of detection dictionaries with:
                - 'type': 'orange_bottle' or 'yogurt_bottle'
                - 'bbox': [x1, y1, x2, y2] (for YOLO) or None (for cap detector)
                - 'center': [cx, cy] (pixel coordinates)
                - 'confidence': detection confidence
                - 'color_match': True if cap color matches
        """
        # Use cap detector for top-down views if available
        if self.cap_detector is not None:
            cap_detections = self.cap_detector.detect_caps(image, debug=False)
            # Convert cap detections to same format as YOLO detections
            detections = []
            for cap_det in cap_detections:
                center = cap_det['center']
                radius = cap_det['radius']
                # Create bounding box from circle
                bbox = [
                    center[0] - radius,
                    center[1] - radius,
                    center[0] + radius,
                    center[1] + radius
                ]
                detections.append({
                    'type': cap_det['type'],
                    'bbox': bbox,
                    'center': center,
                    'confidence': cap_det['confidence'],
                    'color_match': cap_det['color_match'],
                    'size': {
                        'width': radius * 2,
                        'height': radius * 2
                    },
                    'detection_method': 'cap_detector'
                })
            return detections
        
        # Run YOLO detection (for side views or when cap detector not available)
        results = self.detect_objects(image)
        
        detections = []
        
        if results[0].boxes is not None:
            boxes = results[0].boxes
            
            for i in range(len(boxes)):
                box = boxes[i]
                
                # Check if it's a bottle class
                cls = int(box.cls[0])
                if cls not in self.target_classes:
                    continue
                
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold:
                    continue
                
                # Get bounding box
                bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                
                # Calculate center
                cx = (bbox[0] + bbox[2]) / 2
                cy = (bbox[1] + bbox[3]) / 2
                
                # Check for orange bottle (green cap)
                has_green, green_ratio = self.filter_by_color(image, bbox, 'green')
                
                # Check for yogurt bottle (pink cap)
                has_pink, pink_ratio = self.filter_by_color(image, bbox, 'pink')
                
                # Classify based on color
                bottle_type = None
                color_match = False
                
                # Use stronger color preference (2x ratio difference to avoid false positives)
                if has_green and green_ratio > (pink_ratio * 2):
                    bottle_type = 'orange_bottle'
                    color_match = True
                elif has_pink and pink_ratio > (green_ratio * 2):
                    bottle_type = 'yogurt_bottle'
                    color_match = True
                elif has_green and green_ratio > pink_ratio:
                    # Green detected but not strong enough
                    bottle_type = 'orange_bottle'
                    color_match = False
                elif has_pink and pink_ratio > green_ratio:
                    # Pink detected but not strong enough
                    bottle_type = 'yogurt_bottle'
                    color_match = False
                else:
                    # No clear color match, use size/shape heuristics
                    width = bbox[2] - bbox[0]
                    height = bbox[3] - bbox[1]
                    aspect_ratio = width / height if height > 0 else 0
                    
                    # Orange bottle is more square, yogurt is more round
                    if aspect_ratio > 0.85:  # More square-like (tuned threshold)
                        bottle_type = 'orange_bottle'
                    else:
                        bottle_type = 'yogurt_bottle'
                
                detection = {
                    'type': bottle_type,
                    'bbox': bbox.tolist(),
                    'center': [float(cx), float(cy)],
                    'confidence': confidence,
                    'color_match': color_match,
                    'size': {
                        'width': float(bbox[2] - bbox[0]),
                        'height': float(bbox[3] - bbox[1])
                    },
                    'detection_method': 'yolo'
                }
                
                detections.append(detection)
        
        return detections
    
    def draw_detections(self, image, detections):
        """
        Draw detection boxes and labels on image.
        
        Args:
            image: BGR image
            detections: List of detection dictionaries
        
        Returns:
            annotated_image: Image with detections drawn
        """
        annotated = image.copy()
        
        for det in detections:
            # Choose color based on type
            if det['type'] == 'orange_bottle':
                color = (0, 165, 255)  # Orange
            else:
                color = (203, 192, 255)  # Pink
            
            # Draw based on detection method
            if det.get('detection_method') == 'cap_detector':
                # Draw circle for cap detection
                center = tuple(map(int, det['center']))
                radius = int(det.get('radius', 20))
                cv2.circle(annotated, center, radius, color, 2)
                cv2.circle(annotated, center, 3, color, -1)
                
                # Draw label
                label = f"{det['type']} ({det['confidence']:.2f})"
                if det.get('size_mm'):
                    label += f" {det['size_mm']:.1f}mm"
                cv2.putText(annotated, label, (center[0] - 50, center[1] - radius - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                # Draw bounding box for YOLO detection
                bbox = det['bbox']
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                
                # Draw center point
                cx, cy = map(int, det['center'])
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                
                # Draw label
                label = f"{det['type']} ({det['confidence']:.2f})"
                if det['color_match']:
                    label += " [color match]"
                
                cv2.putText(annotated, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated


if __name__ == '__main__':
    # Test object detector
    detector = ObjectDetector()
    
    # Test with camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Testing object detection. Press 'Q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        detections = detector.detect_bottles(frame)
        annotated = detector.draw_detections(frame, detections)
        
        # Show count
        cv2.putText(annotated, f"Detections: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('Object Detection Test', annotated)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

