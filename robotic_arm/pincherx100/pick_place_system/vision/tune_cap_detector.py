#!/usr/bin/env python3
"""
Interactive tool to tune cap detector color ranges and parameters.

This tool helps you adjust the HSV color ranges and detection parameters
to match your specific lighting conditions and cap colors.
"""

import cv2
import numpy as np
import os
import sys

# Add paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from cap_detector import CapDetector


class CapDetectorTuner:
    """Interactive tuner for cap detector parameters."""
    
    def __init__(self):
        self.color_ranges = {
            'green': {
                'lower': np.array([35, 30, 30]),
                'upper': np.array([85, 255, 255])
            },
            'pink': {
                'lower': np.array([168, 185, 120]),
                'upper': np.array([179, 226, 152])
            }
        }
        
        self.current_color = 'green'
        self.current_param = 'H'  # H, S, or V
        self.current_bound = 'lower'  # lower or upper
        
        # Load camera calibration if available
        camera_calib_file = os.path.join(os.path.dirname(__file__), '..', '..', 'camera_calibration', 'camera_calibration.npz')
        camera_matrix = None
        dist_coeffs = None
        
        if os.path.exists(camera_calib_file):
            calib_data = np.load(camera_calib_file)
            camera_matrix = calib_data['camera_matrix']
            dist_coeffs = calib_data['dist_coeffs']
        
        self.detector = CapDetector(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        # Override color ranges
        self.detector.color_ranges = self.color_ranges
        
        print("Cap Detector Tuner")
        print("=" * 50)
        print("Controls:")
        print("  'g' - Switch to green cap tuning")
        print("  'p' - Switch to pink cap tuning")
        print("  'h'/'s'/'v' - Select H/S/V parameter")
        print("  'l'/'u' - Select lower/upper bound")
        print("  Arrow keys - Adjust value")
        print("  'r' - Reset to defaults")
        print("  'q' - Quit")
        print("=" * 50)
    
    def create_mask_view(self, image):
        """Create visualization of color masks."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks
        green_mask = cv2.inRange(hsv, self.color_ranges['green']['lower'], self.color_ranges['green']['upper'])
        pink_mask = cv2.inRange(hsv, self.color_ranges['pink']['lower'], self.color_ranges['pink']['upper'])
        
        # Combine masks
        combined = cv2.bitwise_or(
            cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(pink_mask, cv2.COLOR_GRAY2BGR)
        )
        
        # Apply masks to original
        green_result = cv2.bitwise_and(image, image, mask=green_mask)
        pink_result = cv2.bitwise_and(image, image, mask=pink_mask)
        
        return green_mask, pink_mask, green_result, pink_result
    
    def run(self):
        """Run the tuning interface."""
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect caps with current settings
            detections = self.detector.detect_caps(frame)
            annotated = self.detector.draw_detections(frame, detections)
            
            # Create mask visualization
            green_mask, pink_mask, green_result, pink_result = self.create_mask_view(frame)
            
            # Show current settings
            color_range = self.color_ranges[self.current_color]
            lower = color_range['lower']
            upper = color_range['upper']
            
            param_idx = {'H': 0, 'S': 1, 'V': 2}[self.current_param]
            bound_name = self.current_bound
            current_value = lower[param_idx] if bound_name == 'lower' else upper[param_idx]
            
            info_text = [
                f"Tuning: {self.current_color.upper()} cap",
                f"Parameter: {self.current_param} ({bound_name})",
                f"Current value: {current_value}",
                f"Lower: [{lower[0]}, {lower[1]}, {lower[2]}]",
                f"Upper: [{upper[0]}, {upper[1]}, {upper[2]}]",
                f"Detections: {len(detections)}"
            ]
            
            for i, text in enumerate(info_text):
                cv2.putText(annotated, text, (10, 30 + i * 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show masks side by side
            h, w = frame.shape[:2]
            mask_view = np.zeros((h, w * 2, 3), dtype=np.uint8)
            mask_view[:, :w] = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
            mask_view[:, w:2*w] = cv2.cvtColor(pink_mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_view, "Green Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(mask_view, "Pink Mask", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (203, 192, 255), 2)
            
            cv2.imshow('Cap Detection (Tuning)', annotated)
            cv2.imshow('Color Masks', mask_view)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                self.current_color = 'green'
                print(f"Switched to green cap tuning")
            elif key == ord('p'):
                self.current_color = 'pink'
                print(f"Switched to pink cap tuning")
            elif key == ord('h'):
                self.current_param = 'H'
                print(f"Selected H (Hue) parameter")
            elif key == ord('s'):
                self.current_param = 'S'
                print(f"Selected S (Saturation) parameter")
            elif key == ord('v'):
                self.current_param = 'V'
                print(f"Selected V (Value/Brightness) parameter")
            elif key == ord('l'):
                self.current_bound = 'lower'
                print(f"Selected lower bound")
            elif key == ord('u'):
                self.current_bound = 'upper'
                print(f"Selected upper bound")
            elif key == ord('r'):
                # Reset to defaults
                self.color_ranges = {
                    'green': {
                        'lower': np.array([35, 30, 30]),
                        'upper': np.array([85, 255, 255])
                    },
                    'pink': {
                        'lower': np.array([140, 50, 50]),
                        'upper': np.array([170, 255, 255])
                    }
                }
                self.detector.color_ranges = self.color_ranges
                print("Reset to default values")
            elif key == 82 or key == 0:  # Up arrow
                # Increase value
                param_idx = {'H': 0, 'S': 1, 'V': 2}[self.current_param]
                bound = self.color_ranges[self.current_color][self.current_bound]
                
                if self.current_param == 'H':
                    bound[param_idx] = min(179, bound[param_idx] + 1)
                else:
                    bound[param_idx] = min(255, bound[param_idx] + 1)
                self.detector.color_ranges = self.color_ranges
                print(f"Increased {self.current_param} ({self.current_bound}): {bound[param_idx]}")
            elif key == 84 or key == 1:  # Down arrow
                # Decrease value
                param_idx = {'H': 0, 'S': 1, 'V': 2}[self.current_param]
                bound = self.color_ranges[self.current_color][self.current_bound]
                
                bound[param_idx] = max(0, bound[param_idx] - 1)
                self.detector.color_ranges = self.color_ranges
                print(f"Decreased {self.current_param} ({self.current_bound}): {bound[param_idx]}")
        
        cap.release()
        cv2.destroyAllWindows()
        
        # Print final values
        print("\n" + "=" * 50)
        print("Final color ranges:")
        print(f"Green: lower={self.color_ranges['green']['lower']}, upper={self.color_ranges['green']['upper']}")
        print(f"Pink: lower={self.color_ranges['pink']['lower']}, upper={self.color_ranges['pink']['upper']}")
        print("\nUpdate cap_detector.py with these values if they work better!")


if __name__ == '__main__':
    tuner = CapDetectorTuner()
    tuner.run()

