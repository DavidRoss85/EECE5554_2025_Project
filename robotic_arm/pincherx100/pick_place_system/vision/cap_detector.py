#!/usr/bin/env python3
"""
Top-Down Cap Detection Module

Detects bottle caps from top-down camera view using color and shape detection.
More reliable than YOLO for top-down views where only caps are visible.

Orange bottle: Green circular cap (radius ~2.5cm)
Yogurt bottle: Pink circular cap (radius ~2.0cm)
"""

import cv2
import numpy as np


class CapDetector:
    """
    Detects bottle caps from top-down view using color and circular shape detection.
    """
    
    def __init__(self, camera_matrix=None, dist_coeffs=None, square_size_mm=24.0):
        """
        Initialize cap detector.
        
        Args:
            camera_matrix: Camera intrinsic matrix (for size estimation)
            dist_coeffs: Camera distortion coefficients
            square_size_mm: Size of checkerboard square in mm (for scale reference)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.square_size_mm = square_size_mm
        
        # Cap dimensions (in meters)
        self.cap_sizes = {
            'orange_bottle': {
                'radius_mm': 25.0,  # 2.5cm radius
                'radius_range': (20.0, 30.0)  # Acceptable range in mm
            },
            'yogurt_bottle': {
                'radius_mm': 20.0,  # 2.0cm radius
                'radius_range': (15.0, 25.0)  # Acceptable range in mm
            }
        }
        
        # Color ranges in HSV (tuned for top-down lighting)
        # Expanded green range, pink back to single range
        self.color_ranges = {
            'green': {
                'lower': np.array([35, 30, 30]),  # Expanded: lower H, lower S/V for darker greens
                'upper': np.array([85, 255, 255])  # Expanded: higher H
            },
            'pink': {
                'lower': np.array([168, 185, 120]),  # Back to original range
                'upper': np.array([179, 226, 152])
            }
        }
        
        # Detection parameters (balanced to reduce false positives)
        # Note: Caps appear larger from top-down view due to bottle height/perspective
        self.min_circle_area = 50  # Minimum pixel area for a cap (reduced to catch smaller caps)
        self.max_circle_area = 50000  # Maximum pixel area (increased significantly for perspective/height effects)
        self.circularity_threshold = 0.25  # How circular the shape must be (0-1) (very low for partial/imperfect circles)
        self.min_color_ratio = 0.05  # Minimum ratio of pixels in bbox that match color (5%, very low)
        self.max_aspect_ratio = 3.0  # Maximum aspect ratio (width/height) for valid circle (very permissive)
        self.min_mask_ratio = 0.3  # Minimum ratio of mask pixels in detected region (30% of region should be colored)
        
        # Bottle heights for perspective calculation (in meters)
        # Orange bottle: 18.5cm, Yogurt bottle: 15.5cm
        # Camera is 50cm above platform, so caps are at:
        # Orange: 50cm - 18.5cm = 31.5cm from camera
        # Yogurt: 50cm - 15.5cm = 34.5cm from camera
        # Perspective makes caps appear larger than if they were flat on platform
        self.bottle_heights = {
            'orange_bottle': 0.185,  # 18.5cm
            'yogurt_bottle': 0.155   # 15.5cm
        }
        self.camera_height = 0.50  # 50cm above platform
        
        print("Cap detector initialized (top-down view)")
        print(f"  Min area: {self.min_circle_area}, Max area: {self.max_circle_area} (accounts for bottle height/perspective)")
        print(f"  Circularity threshold: {self.circularity_threshold} (very lenient for partial circles)")
        print(f"  Min color ratio: {self.min_color_ratio}")
        print(f"  Min mask ratio: {self.min_mask_ratio} (for partial circle detection)")
        print(f"  Max aspect ratio: {self.max_aspect_ratio}")
        print(f"  Camera height: {self.camera_height*100:.1f}cm, Bottle heights: Orange={self.bottle_heights['orange_bottle']*100:.1f}cm, Yogurt={self.bottle_heights['yogurt_bottle']*100:.1f}cm")
    
    def detect_circles(self, image):
        """
        Detect circular objects in image using HoughCircles.
        
        Args:
            image: BGR image
        
        Returns:
            circles: List of detected circles [(x, y, radius), ...]
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,  # Minimum distance between circle centers
            param1=50,   # Upper threshold for edge detection
            param2=30,   # Accumulator threshold (lower = more circles)
            minRadius=10,  # Minimum circle radius in pixels
            maxRadius=100  # Maximum circle radius in pixels
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            return [(c[0], c[1], c[2]) for c in circles]
        return []
    
    def detect_circles_by_contour(self, image, color_name, debug=False):
        """
        Detect circular objects by color and contour analysis.
        More reliable than HoughCircles for colored caps.
        
        Args:
            image: BGR image
            color_name: 'green' or 'pink'
            debug: If True, return debug info
        
        Returns:
            circles: List of detected circles with metadata
            debug_info: (if debug=True) Dictionary with mask and contour info
        """
        if color_name not in ['green', 'pink']:
            return [] if not debug else ([], {})
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for color
        color_range = self.color_ranges[color_name]
        mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if debug:
            print(f"\n{color_name.upper()} mask: Found {len(contours)} contours")
        
        circles = []
        debug_info = {
            'mask': mask,
            'contours_all': contours,
            'contours_filtered': [],
            'rejected': {
                'area_too_small': 0,
                'area_too_large': 0,
                'circularity_too_low': 0,
                'aspect_ratio_too_high': 0,
                'color_ratio_too_low': 0,
                'mask_ratio_too_low': 0
            }
        }
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            # Filter by area
            if area < self.min_circle_area:
                debug_info['rejected']['area_too_small'] += 1
                if debug:
                    print(f"  [DEBUG] Contour rejected: area {area:.0f} < min {self.min_circle_area}")
                continue
            if area > self.max_circle_area:
                debug_info['rejected']['area_too_large'] += 1
                if debug:
                    print(f"  [DEBUG] Contour rejected: area {area:.0f} > max {self.max_circle_area}")
                continue
            
            # Get bounding circle first (we'll use this even if not perfectly circular)
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            
            # Check if the bounding circle region has enough color pixels
            # This works better for partial circles
            x1 = max(0, int(x - radius))
            y1 = max(0, int(y - radius))
            x2 = min(image.shape[1], int(x + radius))
            y2 = min(image.shape[0], int(y + radius))
            
            if x2 > x1 and y2 > y1:
                # Check mask ratio in bounding circle region
                circle_region_mask = mask[y1:y2, x1:x2]
                circle_region_area = (x2 - x1) * (y2 - y1)
                if circle_region_area > 0:
                    mask_pixels = np.sum(circle_region_mask > 0)
                    mask_ratio = mask_pixels / circle_region_area
                    if debug:
                        print(f"    radius={int(radius)}, mask_ratio={mask_ratio:.3f} (need >={self.min_mask_ratio})")
                    if mask_ratio < self.min_mask_ratio:
                        debug_info['rejected']['mask_ratio_too_low'] += 1
                        if debug:
                            print(f"    ✗ REJECTED: mask_ratio {mask_ratio:.3f} < min {self.min_mask_ratio}")
                        continue
                    else:
                        if debug:
                            print(f"    ✓ mask_ratio OK")
                else:
                    if debug:
                        print(f"    ✗ REJECTED: invalid circle region")
                    continue
            else:
                if debug:
                    print(f"    ✗ REJECTED: invalid bounding box")
                continue
            
            # Check circularity (but be very lenient - don't reject for low circularity alone)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            # Track low circularity but don't reject - partial circles are expected
            if circularity < self.circularity_threshold:
                debug_info['rejected']['circularity_too_low'] += 1
                # Still allow through if mask ratio is good (partial circle detection)
            
            # Get bounding rect to check aspect ratio (very lenient)
            rect = cv2.boundingRect(contour)
            rect_width, rect_height = rect[2], rect[3]
            if rect_height > 0:
                aspect_ratio = rect_width / rect_height
                if debug:
                    print(f"  Contour {i}: aspect_ratio={aspect_ratio:.2f} (max={self.max_aspect_ratio})")
                if aspect_ratio > self.max_aspect_ratio or aspect_ratio < (1.0 / self.max_aspect_ratio):
                    debug_info['rejected']['aspect_ratio_too_high'] += 1
                    if debug:
                        print(f"  Contour {i}: REJECTED - aspect ratio out of range")
                    continue
            else:
                if debug:
                    print(f"  Contour {i}: REJECTED - zero height")
                continue
            
            # Check color ratio in actual contour area (more accurate than bounding box)
            # Create a mask for just this contour
            contour_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            
            # Count how many pixels in the contour match the color
            color_in_contour = cv2.bitwise_and(mask, contour_mask)
            color_pixels = np.sum(color_in_contour > 0)
            contour_pixels = np.sum(contour_mask > 0)
            
            if contour_pixels > 0:
                color_ratio = color_pixels / contour_pixels
                if debug:
                    print(f"  Contour {i}: color_ratio={color_ratio:.3f} (min={self.min_color_ratio})")
                if color_ratio < self.min_color_ratio:
                    debug_info['rejected']['color_ratio_too_low'] += 1
                    # If mask ratio in circle region is good, still allow it (for partial circles)
                    if mask_ratio < self.min_mask_ratio:
                        if debug:
                            print(f"  ✗ Contour {i}: REJECTED - color ratio too low AND mask ratio also low")
                        continue
                    else:
                        if debug:
                            print(f"  Contour {i}: Color ratio low but mask ratio good, allowing")
                else:
                    if debug:
                        print(f"  Contour {i}: Color ratio OK")
            else:
                if debug:
                    print(f"  ✗ Contour {i}: REJECTED - empty contour")
                continue
            
            # Calculate actual size if camera calibration available
            size_mm = None
            if self.camera_matrix is not None:
                # Estimate size based on pixel radius and camera parameters
                # This is approximate - assumes cap is at platform height
                focal_length = self.camera_matrix[0, 0]
                # Rough estimate: size_mm = (radius_pixels * sensor_size_mm) / focal_length_pixels
                # For now, use a simple heuristic based on known checkerboard size
                size_mm = radius * 2 * (self.square_size_mm / 50.0)  # Rough estimate
            
            circle_info = {
                'center': center,
                'radius': radius,
                'area': area,
                'circularity': circularity,
                'size_mm': size_mm,
                'contour': contour
            }
            circles.append(circle_info)
            debug_info['contours_filtered'].append(contour)
            
            if debug:
                print(f"    ✓✓✓ CONTOUR {i} ACCEPTED! ✓✓✓")
                print(f"      Final: area={int(area)}, radius={int(radius)}, circularity={circularity:.3f}")
                print(f"      mask_ratio={mask_ratio:.3f}, color_ratio={color_ratio:.3f}")
        
        if debug:
            return circles, debug_info
        return circles
    
    def classify_cap(self, circle_info, color_name):
        """
        Classify cap as orange_bottle or yogurt_bottle based on size and color.
        
        Args:
            circle_info: Circle detection info dict
            color_name: 'green' or 'pink'
        
        Returns:
            bottle_type: 'orange_bottle' or 'yogurt_bottle' or None
            confidence: Confidence score (0-1)
        """
        radius = circle_info['radius']
        size_mm = circle_info.get('size_mm')
        
        # If we have size estimate, use it
        if size_mm is not None:
            if color_name == 'green':
                # Green cap should be ~25mm radius (orange bottle)
                expected_radius = self.cap_sizes['orange_bottle']['radius_mm']
                radius_range = self.cap_sizes['orange_bottle']['radius_range']
                
                if radius_range[0] <= size_mm <= radius_range[1]:
                    return 'orange_bottle', 0.9
                else:
                    # Size doesn't match, but color does
                    return 'orange_bottle', 0.6
            
            elif color_name == 'pink':
                # Pink cap should be ~20mm radius (yogurt bottle)
                expected_radius = self.cap_sizes['yogurt_bottle']['radius_mm']
                radius_range = self.cap_sizes['yogurt_bottle']['radius_range']
                
                if radius_range[0] <= size_mm <= radius_range[1]:
                    return 'yogurt_bottle', 0.9
                else:
                    # Size doesn't match, but color does
                    return 'yogurt_bottle', 0.6
        
        # Fallback: classify by color only
        if color_name == 'green':
            return 'orange_bottle', 0.7
        elif color_name == 'pink':
            return 'yogurt_bottle', 0.7
        
        return None, 0.0
    
    def detect_caps(self, image, debug=False):
        """
        Detect bottle caps from top-down view.
        
        Args:
            image: BGR image from top-down camera
            debug: If True, return debug information and print detailed logs
        
        Returns:
            detections: List of detection dictionaries with:
                - 'type': 'orange_bottle' or 'yogurt_bottle'
                - 'center': [cx, cy] (pixel coordinates)
                - 'radius': radius in pixels
                - 'confidence': detection confidence (0-1)
                - 'color_match': True
                - 'size_mm': estimated size in mm (if available)
            debug_info: (if debug=True) Dictionary with masks and rejection stats
        """
        detections = []
        debug_info = {}
        
        # Detect green caps (orange bottles)
        if debug:
            print("\n" + "=" * 60)
            print("Detecting GREEN caps (orange bottles)...")
            print("=" * 60)
            green_circles, green_debug = self.detect_circles_by_contour(image, 'green', debug=True)
            debug_info['green'] = green_debug
            print(f"Found {len(green_circles)} green circles after filtering")
        else:
            green_circles = self.detect_circles_by_contour(image, 'green', debug=False)
        
        for circle_info in green_circles:
            bottle_type, confidence = self.classify_cap(circle_info, 'green')
            if bottle_type:
                detections.append({
                    'type': bottle_type,
                    'center': list(circle_info['center']),
                    'radius': circle_info['radius'],
                    'confidence': confidence,
                    'color_match': True,
                    'size_mm': circle_info.get('size_mm'),
                    'circularity': circle_info['circularity']
                })
        
        # Detect pink caps (yogurt bottles)
        if debug:
            print("\n" + "=" * 60)
            print("Detecting PINK caps (yogurt bottles)...")
            print("=" * 60)
            pink_circles, pink_debug = self.detect_circles_by_contour(image, 'pink', debug=True)
            debug_info['pink'] = pink_debug
            print(f"Found {len(pink_circles)} pink circles after filtering")
            print("=" * 60 + "\n")
        else:
            pink_circles = self.detect_circles_by_contour(image, 'pink', debug=False)
        
        for circle_info in pink_circles:
            bottle_type, confidence = self.classify_cap(circle_info, 'pink')
            if bottle_type:
                detections.append({
                    'type': bottle_type,
                    'center': list(circle_info['center']),
                    'radius': circle_info['radius'],
                    'confidence': confidence,
                    'color_match': True,
                    'size_mm': circle_info.get('size_mm'),
                    'circularity': circle_info['circularity']
                })
        
        if debug:
            return detections, debug_info
        return detections
    
    def draw_detections(self, image, detections):
        """
        Draw detection circles and labels on image.
        
        Args:
            image: BGR image
            detections: List of detection dictionaries
        
        Returns:
            annotated_image: Image with detections drawn
        """
        annotated = image.copy()
        
        for det in detections:
            center = tuple(map(int, det['center']))
            radius = det['radius']
            
            # Choose color based on type
            if det['type'] == 'orange_bottle':
                color = (0, 165, 255)  # Orange
            else:
                color = (203, 192, 255)  # Pink
            
            # Draw circle
            cv2.circle(annotated, center, radius, color, 2)
            cv2.circle(annotated, center, 3, color, -1)
            
            # Draw label
            label = f"{det['type']} ({det['confidence']:.2f})"
            if det.get('size_mm'):
                label += f" {det['size_mm']:.1f}mm"
            
            cv2.putText(annotated, label, (center[0] - 50, center[1] - radius - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated


if __name__ == '__main__':
    # Test cap detector with debug mode
    import os
    import sys
    
    # Add paths
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    
    # Load camera calibration if available
    camera_calib_file = os.path.join(os.path.dirname(__file__), '..', '..', 'camera_calibration', 'camera_calibration.npz')
    camera_matrix = None
    dist_coeffs = None
    
    if os.path.exists(camera_calib_file):
        calib_data = np.load(camera_calib_file)
        camera_matrix = calib_data['camera_matrix']
        dist_coeffs = calib_data['dist_coeffs']
        print("Loaded camera calibration")
    
    detector = CapDetector(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    
    # Test with camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Testing cap detection. Press 'Q' to quit, 'D' to toggle debug view.")
    show_debug = True
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        detections, debug_info = detector.detect_caps(frame, debug=True)
        annotated = detector.draw_detections(frame, detections)
        
        # Show count
        cv2.putText(annotated, f"Detections: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Show debug info
        if show_debug:
            y_pos = 60
            for color_name in ['green', 'pink']:
                if color_name in debug_info:
                    info = debug_info[color_name]
                    rejected = info['rejected']
                    total_contours = len(info['contours_all'])
                    filtered_contours = len(info['contours_filtered'])
                    
                    # Show detailed rejection reasons
                    text1 = f"{color_name}: {filtered_contours}/{total_contours} contours passed"
                    cv2.putText(annotated, text1, (10, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    y_pos += 18
                    
                    text2 = f"  Rejected: area_s={rejected['area_too_small']}, area_l={rejected['area_too_large']}, circ={rejected['circularity_too_low']}"
                    cv2.putText(annotated, text2, (10, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
                    y_pos += 18
                    
                    text3 = f"  aspect={rejected['aspect_ratio_too_high']}, color={rejected['color_ratio_too_low']}, mask={rejected['mask_ratio_too_low']}"
                    cv2.putText(annotated, text3, (10, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
                    y_pos += 25
                    
                    # Show details of filtered contours
                    if filtered_contours > 0:
                        for i, contour in enumerate(info['contours_filtered'][:3]):  # Show first 3
                            area = cv2.contourArea(contour)
                            perimeter = cv2.arcLength(contour, True)
                            if perimeter > 0:
                                circularity = 4 * np.pi * area / (perimeter * perimeter)
                                (x, y), radius = cv2.minEnclosingCircle(contour)
                                text = f"  Contour {i+1}: area={int(area)}, circ={circularity:.2f}, r={int(radius)}"
                                cv2.putText(annotated, text, (10, y_pos),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 255, 150), 1)
                                y_pos += 15
        
        cv2.imshow('Cap Detection Test', annotated)
        
        # Show debug masks
        if show_debug:
            # Create debug view with masks
            h, w = frame.shape[:2]
            debug_view = np.zeros((h, w * 3, 3), dtype=np.uint8)
            debug_view[:, :w] = frame
            debug_view[:, w:2*w] = cv2.cvtColor(debug_info.get('green', {}).get('mask', np.zeros((h, w), dtype=np.uint8)), cv2.COLOR_GRAY2BGR)
            debug_view[:, 2*w:3*w] = cv2.cvtColor(debug_info.get('pink', {}).get('mask', np.zeros((h, w), dtype=np.uint8)), cv2.COLOR_GRAY2BGR)
            
            # Add labels
            cv2.putText(debug_view, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_view, "Green Mask", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_view, "Pink Mask", (2*w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (203, 192, 255), 2)
            
            cv2.imshow('Debug: Color Masks', debug_view)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):
            show_debug = not show_debug
            if not show_debug:
                cv2.destroyWindow('Debug: Color Masks')
    
    cap.release()
    cv2.destroyAllWindows()

