#!/usr/bin/env python3
"""
UVC Camera Stream Server for Raspberry Pi
Streams camera feed over HTTP (MJPEG) so it can be viewed on Mac

Usage:
    python camera_stream_server.py [--camera 0] [--port 5000] [--host 0.0.0.0]
    
Access from Mac:
    Open browser: http://<raspberry_pi_ip>:5000
    Or use the client viewer: python camera_stream_client.py --ip <raspberry_pi_ip>
"""

import cv2
import numpy as np
from flask import Flask, render_template_string, Response
import argparse
import socket
import time

app = Flask(__name__)

# Global camera object
camera = None
camera_index = 0

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>PincherX100 Camera Stream</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #1e1e1e;
            color: #ffffff;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        h1 {
            color: #4CAF50;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        #videoFeed {
            max-width: 100%;
            border: 3px solid #4CAF50;
            border-radius: 10px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.3);
        }
        .info {
            margin-top: 20px;
            padding: 15px;
            background-color: #2d2d2d;
            border-radius: 5px;
            text-align: left;
        }
        .info h3 {
            color: #4CAF50;
            margin-top: 0;
        }
        .status {
            display: inline-block;
            padding: 5px 15px;
            background-color: #4CAF50;
            border-radius: 3px;
            margin: 10px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>PincherX100 Camera Stream</h1>
        <div class="status">LIVE</div>
        <div>
            <img id="videoFeed" src="{{ url_for('video_feed') }}" alt="Camera Feed">
        </div>
        <div class="info">
            <h3>Camera Information</h3>
            <p><strong>Camera Index:</strong> {{ camera_index }}</p>
            <p><strong>Server:</strong> {{ server_ip }}</p>
            <p><strong>Stream URL:</strong> http://{{ server_ip }}:{{ port }}/video_feed</p>
            <h3>Calibration Instructions</h3>
            <ol style="text-align: left;">
                <li>Print the checkerboard pattern (checkerboard_8x6.svg)</li>
                <li>Mount it on a rigid flat surface</li>
                <li>Run the calibration script: <code>python calibrate_camera.py</code></li>
                <li>Hold the checkerboard in different positions and orientations</li>
                <li>Press SPACE to capture calibration images</li>
                <li>Capture 15-20 images from various angles</li>
            </ol>
        </div>
    </div>
    <script>
        // Auto-reload page if stream dies
        var img = document.getElementById('videoFeed');
        img.onerror = function() {
            setTimeout(function() {
                location.reload();
            }, 3000);
        };
    </script>
</body>
</html>
"""

def get_ip_address():
    """Get the local IP address of the Raspberry Pi"""
    try:
        # Create a socket to get the local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"

def init_camera(cam_index):
    """Initialize the camera"""
    global camera
    
    print(f"Initializing camera {cam_index}...")
    camera = cv2.VideoCapture(cam_index)
    
    if not camera.isOpened():
        print(f"Failed to open camera {cam_index}")
        return False
    
    # Set camera properties for better quality
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FPS, 30)
    
    # Verify camera is working
    ret, frame = camera.read()
    if not ret:
        print("Failed to read from camera")
        return False
    
    actual_width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = camera.get(cv2.CAP_PROP_FPS)
    
    print(f"Camera initialized successfully")
    print(f"  Resolution: {int(actual_width)}x{int(actual_height)}")
    print(f"  FPS: {int(actual_fps)}")
    
    return True

def generate_frames():
    """Generate frames for MJPEG stream"""
    global camera
    
    while True:
        if camera is None or not camera.isOpened():
            # Try to reinitialize
            time.sleep(1)
            if not init_camera(camera_index):
                continue
        
        success, frame = camera.read()
        
        if not success:
            print("Failed to read frame")
            time.sleep(0.1)
            continue
        
        # Add timestamp and info overlay
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, timestamp, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        
        # Yield frame in MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Main page with video feed"""
    server_ip = get_ip_address()
    return render_template_string(HTML_TEMPLATE, 
                                 camera_index=camera_index,
                                 server_ip=server_ip,
                                 port=args.port)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    global camera_index, args
    
    parser = argparse.ArgumentParser(description='UVC Camera Stream Server for Raspberry Pi')
    parser.add_argument('--camera', type=int, default=0, 
                       help='Camera index (default: 0)')
    parser.add_argument('--port', type=int, default=5000,
                       help='Server port (default: 5000)')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='Server host (default: 0.0.0.0 for all interfaces)')
    args = parser.parse_args()
    
    camera_index = args.camera
    
    print("=" * 70)
    print("PincherX100 Camera Stream Server")
    print("=" * 70)
    
    # List available cameras
    print("\nChecking for available cameras...")
    available_cameras = []
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    
    if available_cameras:
        print(f"Found cameras at indexes: {available_cameras}")
    else:
        print("No cameras found!")
        print("Make sure UVC camera is connected.")
        return
    
    # Initialize camera
    if not init_camera(camera_index):
        print("\nFailed to initialize camera. Exiting.")
        return
    
    # Get and display server info
    server_ip = get_ip_address()
    
    print("\n" + "=" * 70)
    print("Server starting...")
    print("=" * 70)
    print(f"\nAccess camera stream from your Mac:")
    print(f"   Browser:  http://{server_ip}:{args.port}")
    print(f"   Direct:   http://{server_ip}:{args.port}/video_feed")
    print(f"\nOn your Mac, you can also use:")
    print(f"   python camera_stream_client.py --ip {server_ip} --port {args.port}")
    print("\nPress Ctrl+C to stop server")
    print("=" * 70 + "\n")
    
    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nShutting down server...")
    finally:
        if camera is not None:
            camera.release()
        print("Camera released. Goodbye!")

if __name__ == '__main__':
    main()

