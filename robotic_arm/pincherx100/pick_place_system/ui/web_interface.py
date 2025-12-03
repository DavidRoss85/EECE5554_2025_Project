#!/usr/bin/env python3
"""
Web Interface for Pick and Place System

Provides camera feed, detection overlays, and telemetry data.
"""

import cv2
import numpy as np
import threading
import time
import json
import os
import sys

from flask import Flask, render_template, Response, jsonify

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vision'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from object_detector import ObjectDetector
from checkerboard_detector import CheckerboardDetector
from coordinate_transform import CoordinateTransformer
from arm_controller_wrapper import PickPlaceController


app = Flask(__name__)

# Global state
camera = None
object_detector = None
checkerboard_detector = None
transformer = None
arm_controller = None

frame_lock = threading.Lock()
current_frame = None
detections_data = {
    'objects': [],
    'drop_location': None,
    'arm_status': {}
}


def initialize_components():
    """Initialize all system components."""
    global camera, object_detector, checkerboard_detector, transformer, arm_controller
    
    # Load configuration
    config_file = os.path.join(os.path.dirname(__file__), '..', 'configs', 'robot_config.yaml')
    import yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Initialize camera
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # Initialize transformer
    camera_calib = os.path.join(os.path.dirname(__file__), '..', '..', 'camera_calibration', 'camera_calibration.npz')
    transformer = CoordinateTransformer(camera_calib, config_file)
    
    # Load hand-eye calibration if available
    hand_eye_file = os.path.join(os.path.dirname(__file__), '..', 'calibration', 'hand_eye_calibration.json')
    if os.path.exists(hand_eye_file):
        transformer.load_hand_eye_calibration(hand_eye_file)
    
    # Initialize detectors
    object_detector = ObjectDetector()
    checkerboard_detector = CheckerboardDetector(
        pattern_size=tuple(config['checkerboard']['pattern_size']),
        square_size=config['checkerboard']['square_size'] * 1000
    )
    
    # Initialize arm controller (don't connect yet)
    arm_controller = PickPlaceController()
    
    print("Web interface components initialized")


def capture_loop():
    """Continuous capture and detection loop."""
    global current_frame, detections_data
    
    while True:
        ret, frame = camera.read()
        if not ret:
            time.sleep(0.1)
            continue
        
        # Detect objects
        object_detections = object_detector.detect_bottles(frame)
        
        # Detect checkerboard
        success, center_pixel, center_3d, pose_data = checkerboard_detector.detect(
            frame,
            transformer.camera_matrix,
            transformer.dist_coeffs
        )
        
        # Draw detections
        annotated = object_detector.draw_detections(frame, object_detections)
        
        if success:
            annotated = checkerboard_detector.draw_detection(annotated, center_pixel, pose_data)
        
        # Update frame
        with frame_lock:
            current_frame = annotated.copy()
        
        # Update detection data
        detections_data['objects'] = []
        for det in object_detections:
            pixel_x, pixel_y = det['center']
            position_base = transformer.pixel_to_base_3d(pixel_x, pixel_y, object_height=0.0)
            detections_data['objects'].append({
                'type': det['type'],
                'pixel': det['center'],
                'base': position_base.tolist(),
                'confidence': det['confidence']
            })
        
        if success:
            pixel_x, pixel_y = center_pixel
            position_base = transformer.pixel_to_base_3d(pixel_x, pixel_y, object_height=0.0)
            detections_data['drop_location'] = {
                'pixel': center_pixel.tolist(),
                'base': position_base.tolist()
            }
        else:
            detections_data['drop_location'] = None
        
        # Update arm status
        if arm_controller.initialized:
            positions = arm_controller.get_current_joint_positions()
            if positions:
                detections_data['arm_status'] = {
                    'initialized': True,
                    'joint_positions': positions
                }
            else:
                detections_data['arm_status'] = {
                    'initialized': True,
                    'joint_positions': None
                }
        else:
            detections_data['arm_status'] = {
                'initialized': False
            }
        
        time.sleep(0.033)  # ~30 FPS


def generate_frames():
    """Generate video frames for streaming."""
    while True:
        with frame_lock:
            if current_frame is None:
                time.sleep(0.1)
                continue
            frame = current_frame.copy()
        
        # Encode frame
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/')
def index():
    """Main page."""
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/detections')
def api_detections():
    """API endpoint for detection data."""
    return jsonify(detections_data)


@app.route('/api/arm/status')
def api_arm_status():
    """API endpoint for arm status."""
    return jsonify(detections_data['arm_status'])


if __name__ == '__main__':
    # Initialize components
    initialize_components()
    
    # Start capture thread
    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()
    
    # Run Flask app
    print("Starting web interface on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

