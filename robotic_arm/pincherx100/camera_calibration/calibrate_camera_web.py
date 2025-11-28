#!/usr/bin/env python3
"""
Web-based Camera Calibration for PincherX100
Works through browser - no X11 needed!

Usage:
    python calibrate_camera_web.py --pattern 6x7 --square 12
    Then open: http://<raspberry_pi_ip>:5001
"""

import cv2
import numpy as np
import os
import json
import argparse
from datetime import datetime
from flask import Flask, render_template_string, Response, request, jsonify
import threading

app = Flask(__name__)

# Global variables
camera = None
calibration = None
pattern_size = (6, 7)
square_size = 12.0

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Camera Calibration</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #1e1e1e;
            color: #ffffff;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        h1 { color: #4CAF50; }
        .container { max-width: 1400px; margin: 0 auto; }
        #videoFeed {
            max-width: 100%;
            border: 3px solid #4CAF50;
            border-radius: 10px;
            margin: 20px 0;
        }
        .controls {
            margin: 20px 0;
            padding: 20px;
            background-color: #2d2d2d;
            border-radius: 10px;
        }
        button {
            padding: 15px 30px;
            margin: 10px;
            font-size: 18px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-weight: bold;
        }
        .capture-btn {
            background-color: #4CAF50;
            color: white;
        }
        .capture-btn:hover { background-color: #45a049; }
        .capture-btn:disabled {
            background-color: #666;
            cursor: not-allowed;
        }
        .calibrate-btn {
            background-color: #2196F3;
            color: white;
        }
        .calibrate-btn:hover { background-color: #0b7dda; }
        .status {
            padding: 10px;
            margin: 10px 0;
            border-radius: 5px;
            font-size: 16px;
        }
        .status-ready { background-color: #4CAF50; }
        .status-notready { background-color: #f44336; }
        .status-info { background-color: #2196F3; }
        .info-box {
            text-align: left;
            padding: 15px;
            background-color: #2d2d2d;
            border-radius: 5px;
            margin: 20px 0;
        }
        .progress {
            margin: 20px 0;
        }
        .progress-bar {
            background-color: #444;
            height: 30px;
            border-radius: 15px;
            overflow: hidden;
        }
        .progress-fill {
            background-color: #4CAF50;
            height: 100%;
            transition: width 0.3s;
            text-align: center;
            line-height: 30px;
        }
        .results {
            text-align: left;
            padding: 20px;
            background-color: #2d2d2d;
            border-radius: 10px;
            margin: 20px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Camera Calibration (Zhang's Method)</h1>
        
        <div class="info-box">
            <h3>Configuration:</h3>
            <p>Pattern: {{ pattern[0] }}x{{ pattern[1] }} inner corners</p>
            <p>Square Size: {{ square }}mm</p>
            <p>WARNING: Make sure your printed checkerboard matches this size!</p>
        </div>
        
        <div id="status" class="status status-notready">
            Waiting for pattern detection...
        </div>
        
        <img id="videoFeed" src="{{ url_for('video_feed') }}" alt="Camera Feed">
        
        <div class="controls">
            <h3>Controls:</h3>
            <button id="captureBtn" class="capture-btn" onclick="captureImage()" disabled>
                CAPTURE (SPACE)
            </button>
            <button class="calibrate-btn" onclick="calibrate()">
                CALIBRATE (C)
            </button>
        </div>
        
        <div class="progress">
            <h3>Progress: <span id="imageCount">0</span> / 15-20 images</h3>
            <div class="progress-bar">
                <div id="progressFill" class="progress-fill" style="width: 0%">0%</div>
            </div>
        </div>
        
        <div class="info-box">
            <h3>Instructions:</h3>
            <ol>
                <li>Hold checkerboard in front of camera</li>
                <li>When pattern detected (green overlay), press CAPTURE</li>
                <li>Move to different positions and angles:
                    <ul>
                        <li>Different distances (close, medium, far)</li>
                        <li>Different angles (tilt left/right, up/down)</li>
                        <li>Different positions (corners, edges, center)</li>
                    </ul>
                </li>
                <li>Capture 15-20 good images</li>
                <li>Press CALIBRATE when done</li>
            </ol>
        </div>
        
        <div id="results" style="display:none;" class="results">
            <h3>Calibration Results</h3>
            <div id="resultsContent"></div>
        </div>
    </div>
    
    <script>
        // Update status
        setInterval(function() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    const statusDiv = document.getElementById('status');
                    const captureBtn = document.getElementById('captureBtn');
                    
                    if (data.pattern_found) {
                        statusDiv.textContent = 'Pattern detected! Ready to capture';
                        statusDiv.className = 'status status-ready';
                        captureBtn.disabled = false;
                    } else {
                        statusDiv.textContent = 'Pattern not detected - adjust position';
                        statusDiv.className = 'status status-notready';
                        captureBtn.disabled = true;
                    }
                    
                    // Update progress
                    const count = data.captured_count;
                    document.getElementById('imageCount').textContent = count;
                    const percent = Math.min((count / 20) * 100, 100);
                    document.getElementById('progressFill').style.width = percent + '%';
                    document.getElementById('progressFill').textContent = Math.round(percent) + '%';
                });
        }, 200);
        
        // Capture image
        function captureImage() {
            fetch('/capture', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Image ' + data.count + ' captured!');
                    } else {
                        alert('Failed to capture: ' + data.message);
                    }
                });
        }
        
        // Calibrate
        function calibrate() {
            if (confirm('Start calibration? This may take a few seconds...')) {
                document.getElementById('status').textContent = 'Calibrating...';
                document.getElementById('status').className = 'status status-info';
                
                fetch('/calibrate', {method: 'POST'})
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            document.getElementById('results').style.display = 'block';
                            document.getElementById('resultsContent').innerHTML = data.results_html;
                            document.getElementById('status').textContent = 'Calibration complete!';
                            document.getElementById('status').className = 'status status-ready';
                            alert('Calibration successful!\\nError: ' + data.error.toFixed(4) + ' pixels');
                        } else {
                            alert('Calibration failed: ' + data.message);
                            document.getElementById('status').textContent = 'Calibration failed';
                            document.getElementById('status').className = 'status status-notready';
                        }
                    });
            }
        }
        
        // Keyboard shortcuts
        document.addEventListener('keydown', function(event) {
            if (event.code === 'Space') {
                event.preventDefault();
                const btn = document.getElementById('captureBtn');
                if (!btn.disabled) {
                    captureImage();
                }
            } else if (event.code === 'KeyC') {
                calibrate();
            }
        });
    </script>
</body>
</html>
"""

class CameraCalibration:
    def __init__(self, pattern_size, square_size):
        self.pattern_size = pattern_size
        self.square_size = square_size
        
        # Prepare object points
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        self.objpoints = []
        self.imgpoints = []
        self.captured_count = 0
        self.last_frame = None
        self.last_corners = None
        self.pattern_found = False
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = None
    
    def process_frame(self, frame):
        """Process frame and detect checkerboard"""
        self.last_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard
        ret, corners = cv2.findChessboardCorners(
            gray, self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        drawn_frame = frame.copy()
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(drawn_frame, self.pattern_size, corners_refined, ret)
            self.last_corners = corners_refined
            self.pattern_found = True
        else:
            self.last_corners = None
            self.pattern_found = False
        
        # Add status text
        status = "READY" if ret else "NOT DETECTED"
        color = (0, 255, 0) if ret else (0, 0, 255)
        cv2.putText(drawn_frame, f"Pattern: {status} | Captured: {self.captured_count}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return drawn_frame
    
    def capture(self):
        """Capture current frame for calibration"""
        if self.last_frame is None or self.last_corners is None:
            return False, "No pattern detected"
        
        self.objpoints.append(self.objp)
        self.imgpoints.append(self.last_corners)
        self.captured_count += 1
        
        # Save image
        os.makedirs('calibration_images', exist_ok=True)
        filename = f'calibration_images/calib_{self.captured_count:03d}.jpg'
        cv2.imwrite(filename, self.last_frame)
        
        return True, f"Captured image {self.captured_count}"
    
    def calibrate(self):
        """Perform calibration"""
        if len(self.objpoints) < 10:
            return False, f"Need at least 10 images (have {len(self.objpoints)})"
        
        image_size = (self.last_frame.shape[1], self.last_frame.shape[0])
        
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None
        )
        
        if not ret:
            return False, "Calibration failed"
        
        # Calculate error
        total_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                self.objpoints[i], rvecs[i], tvecs[i], 
                self.camera_matrix, self.dist_coeffs
            )
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        
        self.calibration_error = total_error / len(self.objpoints)
        
        # Save calibration
        np.savez(
            'camera_calibration.npz',
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            image_size=image_size,
            calibration_error=self.calibration_error,
            pattern_size=self.pattern_size,
            square_size=self.square_size
        )
        
        calib_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'image_size': list(image_size),
            'calibration_error': float(self.calibration_error),
            'pattern_size': list(self.pattern_size),
            'square_size': float(self.square_size),
            'calibration_date': datetime.now().isoformat(),
            'num_images': len(self.objpoints)
        }
        
        with open('camera_calibration.json', 'w') as f:
            json.dump(calib_data, f, indent=2)
        
        return True, "Calibration successful"

def generate_frames():
    """Generate frames for video feed"""
    global camera, calibration
    
    while True:
        if camera is None or not camera.isOpened():
            break
        
        success, frame = camera.read()
        if not success:
            break
        
        # Process frame
        processed_frame = calibration.process_frame(frame)
        
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', processed_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, 
                                 pattern=pattern_size, 
                                 square=square_size)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    return jsonify({
        'pattern_found': calibration.pattern_found,
        'captured_count': calibration.captured_count
    })

@app.route('/capture', methods=['POST'])
def capture():
    success, message = calibration.capture()
    return jsonify({
        'success': success,
        'message': message,
        'count': calibration.captured_count
    })

@app.route('/calibrate', methods=['POST'])
def calibrate_route():
    success, message = calibration.calibrate()
    
    if success:
        results_html = f"""
        <p><strong>Reprojection Error:</strong> {calibration.calibration_error:.4f} pixels</p>
        <p><strong>Number of Images:</strong> {len(calibration.objpoints)}</p>
        <p><strong>Camera Matrix:</strong></p>
        <pre>{calibration.camera_matrix}</pre>
        <p><strong>Distortion Coefficients:</strong></p>
        <pre>{calibration.dist_coeffs}</pre>
        <p><strong>Files Saved:</strong></p>
        <ul>
            <li>camera_calibration.npz</li>
            <li>camera_calibration.json</li>
            <li>calibration_images/ folder</li>
        </ul>
        """
        return jsonify({
            'success': True,
            'error': float(calibration.calibration_error),
            'results_html': results_html
        })
    else:
        return jsonify({
            'success': False,
            'message': message
        })

def main():
    global camera, calibration, pattern_size, square_size
    
    parser = argparse.ArgumentParser(description='Web-based Camera Calibration')
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--pattern', type=str, default='6x7', help='Pattern size (e.g., 6x7)')
    parser.add_argument('--square', type=float, default=12.0, help='Square size in mm')
    parser.add_argument('--port', type=int, default=5001, help='Web server port')
    args = parser.parse_args()
    
    # Parse pattern
    pattern_parts = args.pattern.split('x')
    pattern_size = (int(pattern_parts[0]), int(pattern_parts[1]))
    square_size = args.square
    
    print("=" * 70)
    print("Web-Based Camera Calibration")
    print("=" * 70)
    print(f"\nPattern: {pattern_size[0]}x{pattern_size[1]} inner corners")
    print(f"Square size: {square_size}mm")
    
    # Initialize camera
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        print(f"Failed to open camera {args.camera}")
        return
    
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Camera opened")
    
    # Initialize calibration
    calibration = CameraCalibration(pattern_size, square_size)
    
    # Get IP
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except:
        ip = "localhost"
    finally:
        s.close()
    
    print("\n" + "=" * 70)
    print("Open in your browser:")
    print(f"   http://{ip}:{args.port}")
    print("=" * 70)
    print("\nPress Ctrl+C to stop")
    
    try:
        app.run(host='0.0.0.0', port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        if camera is not None:
            camera.release()

if __name__ == '__main__':
    main()

