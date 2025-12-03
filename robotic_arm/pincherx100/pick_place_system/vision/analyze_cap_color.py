#!/usr/bin/env python3
"""
Cap Color Analyzer

Takes a picture and analyzes the color of bottle caps to determine optimal HSV ranges.
This helps tune the color detection for your specific lighting conditions.

Usage:
    python analyze_cap_color.py [--image path/to/image.jpg]
    # If no image provided, will capture from camera
"""

import cv2
import numpy as np
import argparse
import os
import sys


class CapColorAnalyzer:
    """Analyze bottle cap colors from images."""
    
    def __init__(self):
        self.selected_region = None
        self.drawing = False
        self.start_point = None
        self.end_point = None
        self.current_image = None
        self.hsv_image = None
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for region selection."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            self.end_point = (x, y)
        
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.end_point = (x, y)
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end_point = (x, y)
            # Ensure start_point is top-left and end_point is bottom-right
            x1, y1 = self.start_point
            x2, y2 = self.end_point
            self.selected_region = (
                min(x1, x2), min(y1, y2),
                max(x1, x2), max(y1, y2)
            )
    
    def analyze_region(self, image, region):
        """
        Analyze color in selected region.
        
        Args:
            image: BGR image
            region: (x1, y1, x2, y2) bounding box
        
        Returns:
            analysis: Dictionary with color statistics
        """
        x1, y1, x2, y2 = region
        
        # Extract region
        roi = image[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        
        # Convert to HSV
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Calculate statistics
        h_channel = hsv_roi[:, :, 0].flatten()
        s_channel = hsv_roi[:, :, 1].flatten()
        v_channel = hsv_roi[:, :, 2].flatten()
        
        # Remove zeros (masked pixels) if any
        h_channel = h_channel[h_channel > 0]
        s_channel = s_channel[s_channel > 0]
        v_channel = v_channel[v_channel > 0]
        
        if len(h_channel) == 0:
            return None
        
        analysis = {
            'h': {
                'min': int(np.min(h_channel)),
                'max': int(np.max(h_channel)),
                'mean': int(np.mean(h_channel)),
                'std': int(np.std(h_channel)),
                'median': int(np.median(h_channel))
            },
            's': {
                'min': int(np.min(s_channel)),
                'max': int(np.max(s_channel)),
                'mean': int(np.mean(s_channel)),
                'std': int(np.std(s_channel)),
                'median': int(np.median(s_channel))
            },
            'v': {
                'min': int(np.min(v_channel)),
                'max': int(np.max(v_channel)),
                'mean': int(np.mean(v_channel)),
                'std': int(np.std(v_channel)),
                'median': int(np.median(v_channel))
            },
            'pixel_count': len(h_channel)
        }
        
        return analysis
    
    def suggest_color_range(self, analysis, margin_factor=0.3):
        """
        Suggest color range based on analysis.
        
        Args:
            analysis: Color analysis dictionary
            margin_factor: How much margin to add (0.3 = 30% margin)
        
        Returns:
            suggested_range: (lower, upper) HSV arrays
        """
        h = analysis['h']
        s = analysis['s']
        v = analysis['v']
        
        # Calculate ranges with margin
        h_range = h['max'] - h['min']
        s_range = s['max'] - s['min']
        v_range = v['max'] - v['min']
        
        h_margin = max(5, int(h_range * margin_factor))
        s_margin = max(10, int(s_range * margin_factor))
        v_margin = max(10, int(v_range * margin_factor))
        
        lower = np.array([
            max(0, h['min'] - h_margin),
            max(0, s['min'] - s_margin),
            max(0, v['min'] - v_margin)
        ])
        
        upper = np.array([
            min(179, h['max'] + h_margin),  # H max is 179 in OpenCV
            min(255, s['max'] + s_margin),
            min(255, v['max'] + v_margin)
        ])
        
        return lower, upper
    
    def draw_analysis_overlay(self, image, region, analysis, suggested_range):
        """Draw analysis results on image."""
        annotated = image.copy()
        
        if region:
            x1, y1, x2, y2 = region
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw current selection rectangle
        if self.drawing and self.start_point and self.end_point:
            cv2.rectangle(annotated, self.start_point, self.end_point, (255, 0, 0), 2)
        
        # Add text overlay
        y_offset = 30
        line_height = 25
        
        if analysis:
            text_lines = [
                "Color Analysis:",
                f"H: min={analysis['h']['min']}, max={analysis['h']['max']}, mean={analysis['h']['mean']:.0f}",
                f"S: min={analysis['s']['min']}, max={analysis['s']['max']}, mean={analysis['s']['mean']:.0f}",
                f"V: min={analysis['v']['min']}, max={analysis['v']['max']}, mean={analysis['v']['mean']:.0f}",
                "",
                "Suggested Range:",
                f"Lower: [{suggested_range[0][0]}, {suggested_range[0][1]}, {suggested_range[0][2]}]",
                f"Upper: [{suggested_range[1][0]}, {suggested_range[1][1]}, {suggested_range[1][2]}]",
                "",
                "Instructions:",
                "1. Click and drag to select cap region",
                "2. Press 'A' to analyze selected region",
                "3. Press 'S' to save suggested range",
                "4. Press 'C' to clear selection",
                "5. Press 'Q' to quit"
            ]
        else:
            text_lines = [
                "Instructions:",
                "1. Click and drag to select cap region",
                "2. Press 'A' to analyze selected region",
                "3. Press 'Q' to quit"
            ]
        
        for line in text_lines:
            cv2.putText(annotated, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += line_height
        
        return annotated
    
    def run(self, image_path=None):
        """Run the color analyzer."""
        # Load or capture image
        if image_path and os.path.exists(image_path):
            self.current_image = cv2.imread(image_path)
            print(f"Loaded image: {image_path}")
        else:
            # Capture from camera
            print("Capturing image from camera...")
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            print("Press SPACE to capture, Q to quit")
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                cv2.putText(frame, "Press SPACE to capture, Q to quit", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow('Camera - Press SPACE to capture', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    self.current_image = frame.copy()
                    break
                elif key == ord('q'):
                    cap.release()
                    cv2.destroyAllWindows()
                    return
            
            cap.release()
            cv2.destroyAllWindows()
        
        if self.current_image is None:
            print("Error: No image available")
            return
        
        # Convert to HSV
        self.hsv_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        
        # Create window and set mouse callback
        window_name = 'Cap Color Analyzer'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        analysis = None
        suggested_range = None
        
        print("\n" + "=" * 60)
        print("Cap Color Analyzer")
        print("=" * 60)
        print("Instructions:")
        print("  1. Click and drag to select the cap region")
        print("  2. Press 'A' to analyze the selected region")
        print("  3. Press 'S' to save the suggested range to file")
        print("  4. Press 'C' to clear selection")
        print("  5. Press 'Q' to quit")
        print("=" * 60 + "\n")
        
        while True:
            display_image = self.draw_analysis_overlay(
                self.current_image,
                self.selected_region,
                analysis,
                suggested_range
            )
            
            cv2.imshow(window_name, display_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            
            elif key == ord('a') or key == ord('A'):
                if self.selected_region:
                    print("\nAnalyzing region...")
                    analysis = self.analyze_region(self.current_image, self.selected_region)
                    if analysis:
                        suggested_range = self.suggest_color_range(analysis)
                        
                        print("\n" + "-" * 60)
                        print("Color Analysis Results:")
                        print("-" * 60)
                        print(f"Hue (H):")
                        print(f"  Min: {analysis['h']['min']}, Max: {analysis['h']['max']}")
                        print(f"  Mean: {analysis['h']['mean']:.1f}, Std: {analysis['h']['std']:.1f}")
                        print(f"  Median: {analysis['h']['median']}")
                        print(f"\nSaturation (S):")
                        print(f"  Min: {analysis['s']['min']}, Max: {analysis['s']['max']}")
                        print(f"  Mean: {analysis['s']['mean']:.1f}, Std: {analysis['s']['std']:.1f}")
                        print(f"  Median: {analysis['s']['median']}")
                        print(f"\nValue/Brightness (V):")
                        print(f"  Min: {analysis['v']['min']}, Max: {analysis['v']['max']}")
                        print(f"  Mean: {analysis['v']['mean']:.1f}, Std: {analysis['v']['std']:.1f}")
                        print(f"  Median: {analysis['v']['median']}")
                        print(f"\nPixel count: {analysis['pixel_count']}")
                        print("\n" + "-" * 60)
                        print("Suggested HSV Range:")
                        print(f"  Lower: [{suggested_range[0][0]}, {suggested_range[0][1]}, {suggested_range[0][2]}]")
                        print(f"  Upper: [{suggested_range[1][0]}, {suggested_range[1][1]}, {suggested_range[1][2]}]")
                        print("-" * 60 + "\n")
                    else:
                        print("Error: Could not analyze region")
                else:
                    print("Please select a region first (click and drag)")
            
            elif key == ord('s') or key == ord('S'):
                if suggested_range:
                    # Save to file
                    output_file = 'suggested_color_range.txt'
                    with open(output_file, 'w') as f:
                        f.write("Suggested HSV Color Range\n")
                        f.write("=" * 40 + "\n\n")
                        f.write(f"Lower: [{suggested_range[0][0]}, {suggested_range[0][1]}, {suggested_range[0][2]}]\n")
                        f.write(f"Upper: [{suggested_range[1][0]}, {suggested_range[1][1]}, {suggested_range[1][2]}]\n\n")
                        f.write("Use in cap_detector.py:\n")
                        f.write(f"  'lower': np.array([{suggested_range[0][0]}, {suggested_range[0][1]}, {suggested_range[0][2]}]),\n")
                        f.write(f"  'upper': np.array([{suggested_range[1][0]}, {suggested_range[1][1]}, {suggested_range[1][2]}])\n")
                    print(f"\nSaved suggested range to {output_file}")
                else:
                    print("Please analyze a region first (press 'A')")
            
            elif key == ord('c') or key == ord('C'):
                self.selected_region = None
                analysis = None
                suggested_range = None
                print("Selection cleared")
        
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='Analyze bottle cap colors from images')
    parser.add_argument('--image', type=str, default=None,
                       help='Path to image file (if not provided, will capture from camera)')
    
    args = parser.parse_args()
    
    analyzer = CapColorAnalyzer()
    analyzer.run(image_path=args.image)


if __name__ == '__main__':
    main()

