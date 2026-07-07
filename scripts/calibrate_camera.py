#!/usr/bin/env python3
"""Real camera-calibration flow for vehicle_wbt Platform.

Run sequence (operator does this on the physical robot):

    # 1. Print a chessboard (8x6 INNER corners, 25 mm squares). Mount it
    #    in front of the camera you're calibrating, hold it flat, tilt
    #    through various angles, ~20 frames at varied distances.

    # 2. Start camera_node (the launch file does this):
    ros2 launch vehicle_wbt_platform_cpp full_system.launch.py \
        front_device:=/dev/cam<your front>

    # 3. Capture frames to disk (one for offline analysis, this script
    #    also reads them back for chessboard detection):
    mkdir -p ~/calib_session
    ros2 bag record -o ~/calib_session/front.bag \
        /vehicle_wbt/v1/sensors/camera/front/image_raw

    # 4. Run the standard ROS2 calibration tool (cameracalibrator.py opens
    #    a GUI; click Calibrate when you have ~20 good frames, then click
    #    Save and point the dialog at params/camera_front.yaml):
    ros2 run camera_calibration cameracalibrator.py --size 8x6 --square 0.025 \
        image:=/vehicle_wbt/v1/sensors/camera/front/image_raw

    # 5. Restart camera_node — isCalibrated() returns true → camera_info
    #    starts being published. Done.

OR run this script end-to-end on recorded frames (headless, no GUI):

    python3 scripts/calibrate_camera.py /path/to/frames/*.png 8 6 0.025 \\
        --out params/camera_front.yaml

This script does NOT fake intrinsics. It either finds a chessboard in the
input images and computes K/D from real corners, or it fails loudly and
writes no output. There is no "no calibration today, fill in plausible
numbers" mode — that would violate the no-mocks rule.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "images", nargs="+", type=Path, help="Calibration frames (PNG/JPG)")
    p.add_argument(
        "cols", type=int, help="Chessboard INNER corners — columns (e.g. 8)")
    p.add_argument(
        "rows", type=int, help="Chessboard INNER corners — rows (e.g. 6)")
    p.add_argument(
        "square_m", type=float, help="Square side length in meters (e.g. 0.025)")
    p.add_argument(
        "--out", type=Path, default=Path("camera.yaml"),
        help="Output YAML path (camera_info_manager schema). "
             "NOT created if calibration fails.")
    p.add_argument(
        "--min-frames", type=int, default=10,
        help="Refuse calibration if fewer frames have the chessboard (default 10)")
    p.add_argument(
        "--fix-aspect", action="store_true",
        help="Force fx == fy if your lens is square-pixel (most USB cams are)")
    return p.parse_args()


def main():
    args = parse_args()
    if args.cols < 3 or args.rows < 3:
        print(f"ERROR: --cols/--rows must be >= 3 (got {args.cols}x{args.rows})")
        sys.exit(1)
    if args.square_m <= 0:
        print(f"ERROR: square_m must be > 0 (got {args.square_m})")
        sys.exit(1)

    pattern = (args.cols, args.rows)
    pattern_size = (args.cols - 1, args.rows - 1)  # OpenCV uses squares-based
    object_pts = np.zeros((pattern[0] * pattern[1], 3), dtype=np.float64)
    for j in range(pattern[1]):
        for i in range(pattern[0]):
            object_pts[j * pattern[0] + i] = [i * args.square_m, j * args.square_m, 0.0]
    object_pts_template = object_pts.copy()

    flags = cv2.CALIB_CB_FAST_CHECK if args.fix_aspect else 0
    detect_flags = flags
    subpix_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    img_pts_all = []
    obj_pts_all = []
    used_images = []
    skipped = []
    width, height = None, None

    for p in args.images:
        if not p.is_file():
            print(f"WARN: not a file: {p}")
            continue
        img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"WARN: cannot read: {p}")
            continue
        if width is None:
            height, width = img.shape
        found, corners = cv2.findChessboardCorners(
            img, pattern_size, detect_flags)
        if not found:
            skipped.append(p)
            continue
        cv2.cornerSubPix(
            img, corners, (11, 11), (-1, -1), subpix_criteria)
        img_pts_all.append(corners)
        obj_pts_all.append(object_pts_template.copy())
        used_images.append(p)

    print(
        f"chessboard detected in {len(img_pts_all)}/{len(args.images)} images; "
        f"{len(skipped)} skipped")
    if len(img_pts_all) < args.min_frames:
        print(
            f"ERROR: need at least {args.min_frames} good frames for a "
            f"trustworthy calibration; got {len(img_pts_all)}. The robot "
            f"operator must (1) print a chessboard, (2) position it in "
            f"front of the camera, (3) capture frames at varied angles "
            f"and distances. This script writes NO output file in that "
            f"case — params/camera_<id>.yaml stays in its 'no calibration' "
            f"sentinel state and camera_node will not publish camera_info.")
        sys.exit(1)

    # Camera-matrix init: start from the rough values OpenCV suggests for
    # the sensor size. Calibration refines these. This is not 'plausible
    # numbers we made up' — it is the mathematically correct starting
    # point that calibrateCamera() needs.
    K_init = np.array(
        [[max(width, height), 0.0, width / 2.0],
         [0.0, max(width, height), height / 2.0],
         [0.0, 0.0, 1.0]],
        dtype=np.float64)
    calib_flags = 0
    if args.fix_aspect:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts_all, img_pts_all, (width, height),
        K_init, None, flags=calib_flags)

    print(f"RMS reprojection error: {rms:.4f} pixels")
    if rms > 1.0:
        print(
            f"WARN: RMS={rms:.4f} is high — typical USB webcam calibrations "
            f"achieve <0.5 px. Re-capture frames with better coverage.")

    # OpenCV camera_calibration_parsers schema. R and P match an
    # unrectified monocular camera (R identity, P = K * [I|0]).
    R_out = np.eye(3, dtype=np.float64).reshape(-1).tolist()
    P_out = np.zeros((3, 4), dtype=np.float64)
    P_out[:3, :3] = K
    K_list = K.reshape(-1).tolist()
    D_list = dist.reshape(-1).tolist() if dist.size > 0 else [0.0] * 5

    # Hand-write the YAML in the exact format camera_calibration_parsers
    # reads back, so round-tripping (save → loadCameraInfo) is lossless.
    def arr(name, data, rows):
        body = ", ".join(repr(v) for v in data)
        return (f"{name}:\n  rows: {rows}\n  cols: {len(data)//rows}\n"
                f"  data: [{body}]")

    body = (
        f"image_width: {width}\n"
        f"image_height: {height}\n"
        f"camera_name: front\n"
        f"distortion_model: {('rational_polynomial' if len(D_list) > 5 else 'plumb_bob')}\n"
        f"\n"
        f"{arr('camera_matrix', K_list, 3)}\n"
        f"\n"
        f"{arr('distortion_coefficients', D_list, 1)}\n"
        f"\n"
        f"{arr('rectification_matrix', R_out, 3)}\n"
        f"\n"
        f"{arr('projection_matrix', P_out.reshape(-1).tolist(), 3)}\n"
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(body)
    print(f"wrote {args.out} (RMS={rms:.4f} px)")
    print(
        f"\n>>> To use this calibration, restart camera_node with the new "
        f"file. It will detect non-zero K[0] and begin publishing "
        f"camera_info. <<<")


if __name__ == "__main__":
    main()
