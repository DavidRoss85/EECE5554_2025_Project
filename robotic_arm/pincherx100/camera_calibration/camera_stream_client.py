#!/usr/bin/env python3
"""
Camera Stream Client for Mac
Displays the camera stream from Raspberry Pi in an OpenCV window

Usage:
    python camera_stream_client.py --ip <raspberry_pi_ip> [--port 5000]
"""

import cv2
import numpy as np
import argparse
import urllib.request
import time

def main():
    parser = argparse.ArgumentParser(description='Camera Stream Client for Mac')
    parser.add_argument('--ip', type=str, required=True,
                       help='Raspberry Pi IP address')
    parser.add_argument('--port', type=int, default=5000,
                       help='Server port (default: 5000)')
    args = parser.parse_args()
    
    stream_url = f"http://{args.ip}:{args.port}/video_feed"
    
    print("=" * 70)
    print("PincherX100 Camera Stream Client")
    print("=" * 70)
    print(f"\nConnecting to: {stream_url}")
    print("\nControls:")
    print("  ESC or Q - Quit")
    print("  S - Save snapshot")
    print("=" * 70 + "\n")
    
    # Open stream
    try:
        cap = cv2.VideoCapture(stream_url)
        
        if not cap.isOpened():
            print(f"Failed to connect to stream at {stream_url}")
            print("\nTroubleshooting:")
            print("1. Make sure the server is running on Raspberry Pi")
            print("2. Check that you can ping the Raspberry Pi")
            print("3. Verify the IP address and port")
            print("4. Check firewall settings")
            return
        
        print("Connected successfully!")
        print("\nPress ESC or Q to quit, S to save snapshot\n")
        
        snapshot_count = 0
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Lost connection to stream")
                break
            
            # Display the frame
            cv2.imshow('PincherX100 Camera Stream', frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):  # ESC or Q
                print("Quitting...")
                break
            elif key == ord('s'):  # Save snapshot
                snapshot_count += 1
                filename = f"snapshot_{snapshot_count:03d}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved {filename}")
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()
        print("\nClient closed. Goodbye!")

if __name__ == '__main__':
    main()

