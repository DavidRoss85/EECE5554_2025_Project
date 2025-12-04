#!/usr/bin/env python3
"""
AprilTag Distance Measurement Tool

Live camera tool that detects AprilTags and shows their distance from
the camera using the calibrated intrinsics.

Usage:
    python apriltag_distance.py --size 0.0254

Options:
    --size       Tag size in meters (e.g., 0.0254 for 1 inch, 0.05 for 5cm)
    --camera     Camera index (default: 0)
    --calib      Path to camera_calibration.npz (default: ../calibration/camera_calibration.npz)
    --family     AprilTag family (default: tag36h11)

Displayed for each detected tag:
    - Tag ID
    - Z distance (forward from camera, meters)
    - 3D distance (Euclidean distance from camera, meters)
    - Pixel position of tag center
"""

import sys
import os
import argparse
import cv2
import numpy as np
from dt_apriltags import Detector


def load_calibration(calib_path: str):
    if not os.path.exists(calib_path):
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")
    data = np.load(calib_path)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    return camera_matrix, dist_coeffs


def create_detector(camera_matrix, family: str, tag_size_m: float):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    camera_params = [fx, fy, cx, cy]

    detector = Detector(
        families=family,
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    return detector, camera_params


def draw_info_panel(frame, tag_size_m: float):
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Background for text
    cv2.rectangle(overlay, (10, 10), (420, 120), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)

    cv2.putText(frame, "AprilTag Distance Tool", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.putText(frame, f"Tag size: {tag_size_m*1000:.1f} mm", (20, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, "Q: quit, S: save frame", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return frame


def draw_tag_info(frame, center_px, distance_z, distance_3d, tag_id):
    x, y = int(center_px[0]), int(center_px[1])

    # Draw center point
    cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    # Text with distances
    text1 = f"ID {tag_id} Z={distance_z:.3f}m D={distance_3d:.3f}m"
    text2 = f"px=({x},{y})"

    y_text = y - 25 if y - 25 > 10 else y + 25
    cv2.putText(frame, text1, (x - 80, y_text),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(frame, text2, (x - 80, y_text + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)

    return frame


def main():
    parser = argparse.ArgumentParser(
        description="AprilTag distance measurement tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  python apriltag_distance.py --size 0.0254
  python apriltag_distance.py --size 0.05 --camera 1
  python apriltag_distance.py --size 0.0254 --calib ../calibration/camera_calibration.npz

Notes:
  - Distance Z is along the camera's forward axis (in meters)
  - Distance D is full 3D distance from camera to tag center
  - Accuracy depends heavily on correct tag_size and calibration
""",
    )

    parser.add_argument("--size", type=float, required=True,
                        help="AprilTag size in meters (e.g., 0.0254 for 1 inch)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument("--calib", type=str, default=None,
                        help="Path to camera_calibration.npz (default: ../calibration/camera_calibration.npz)")
    parser.add_argument("--family", type=str, default="tag36h11",
                        help="AprilTag family (default: tag36h11)")

    args = parser.parse_args()

    # Resolve calibration path
    if args.calib is None:
        calib_path = os.path.join(os.path.dirname(__file__), "..", "calibration", "camera_calibration.npz")
    else:
        calib_path = args.calib

    calib_path = os.path.abspath(calib_path)

    print("Loading calibration from:", calib_path)
    camera_matrix, dist_coeffs = load_calibration(calib_path)

    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:", dist_coeffs.ravel())

    detector, camera_params = create_detector(camera_matrix, args.family, args.size)

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Failed to open camera {args.camera}")
        return 1

    # Try to set a reasonable resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("\nStarting live AprilTag distance view...")
    print("Press Q to quit, S to save a frame")

    frame_idx = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=args.size,
            )

            # Draw detections
            for det in detections:
                # Pose (translation) is in meters in camera frame
                t = det.pose_t.flatten()  # [x, y, z]
                x_c, y_c, z_c = t[0], t[1], t[2]
                distance_z = z_c
                distance_3d = float(np.linalg.norm(t))

                center = det.center  # [u, v]

                # Draw bounding box
                corners = det.corners.astype(int)
                cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

                frame = draw_tag_info(frame, center, distance_z, distance_3d, det.tag_id)

            frame = draw_info_panel(frame, args.size)

            cv2.imshow("AprilTag Distance", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == ord("Q"):
                break
            elif key == ord("s") or key == ord("S"):
                out_name = f"apriltag_distance_{frame_idx:04d}.png"
                cv2.imwrite(out_name, frame)
                print(f"Saved frame to {out_name}")

            frame_idx += 1

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
