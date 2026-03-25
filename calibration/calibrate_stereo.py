#!/usr/bin/env python3
"""
Stereo camera calibration script.
Processes captured image pairs and outputs calibration values
ready to paste into the OV2SLAM YAML config file.

Usage:
    python3 calibrate_stereo.py [--images-dir PATH]

Accepts two image directory layouts:
  Subdirectory layout (default):
    <dir>/left/left_XXXX.png
    <dir>/right/right_XXXX.png

  Flat layout (e.g. initial_calibration/):
    <dir>/left-XXXX.png
    <dir>/right-XXXX.png

Images are paired by sorted order, so left-0000.png pairs with right-0000.png, etc.
"""

import argparse
import os
import sys
import glob
import gc
import numpy as np
import cv2

DEFAULT_IMAGES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_images")

# Checkerboard dimensions - number of INNER corners (squares - 1)
# For an 12x8 printed board this is 11x7
BOARD_COLS = 11
BOARD_ROWS = 7
SQUARE_SIZE = 0.025  # meters - match your printed square size


def find_corners(image_paths, board_size):
    """Find checkerboard corners in a list of images."""
    obj_points = []  # 3D points in real world
    img_points = []  # 2D points in image plane
    valid_pairs = []

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in image_paths:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        del img
        ret, corners = cv2.findChessboardCornersSB(
            gray, board_size,
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        )
        del gray
        gc.collect()
        if ret:
            obj_points.append(objp)
            img_points.append(corners)
            valid_pairs.append(path)
        else:
            print(f"  WARNING: No corners found in {os.path.basename(path)} - skipping")

    return obj_points, img_points, valid_pairs


def format_matrix(name, mat):
    """Format a matrix as OpenCV YAML format for OV2SLAM config."""
    rows, cols = mat.shape
    data = mat.flatten().tolist()
    data_str = ", ".join(f"{v:.10f}" for v in data)
    return (
        f"{name}: !!opencv-matrix\n"
        f"   rows: {rows}\n"
        f"   cols: {cols}\n"
        f"   dt: d\n"
        f"   data: [{data_str}]"
    )


def find_images(images_dir):
    """Find left/right image pairs, supporting both subdirectory and flat layouts."""
    # Try subdirectory layout first: <dir>/left/*.png, <dir>/right/*.png
    left_images = sorted(glob.glob(os.path.join(images_dir, "left", "*.png")))
    right_images = sorted(glob.glob(os.path.join(images_dir, "right", "*.png")))

    if left_images and right_images:
        print(f"Using subdirectory layout: {images_dir}/left/ and {images_dir}/right/")
        return left_images, right_images

    # Try flat layout: <dir>/left-*.png, <dir>/right-*.png
    left_images = sorted(glob.glob(os.path.join(images_dir, "left-*.png")))
    right_images = sorted(glob.glob(os.path.join(images_dir, "right-*.png")))

    if left_images and right_images:
        print(f"Using flat layout: {images_dir}/left-*.png and {images_dir}/right-*.png")
        return left_images, right_images

    return [], []


def main():
    parser = argparse.ArgumentParser(description="Stereo camera calibration for OV2SLAM")
    parser.add_argument(
        "--images-dir",
        default=DEFAULT_IMAGES_DIR,
        help="Directory containing left/right image pairs (default: %(default)s)",
    )
    args = parser.parse_args()
    images_dir = os.path.abspath(args.images_dir)

    board_size = (BOARD_COLS, BOARD_ROWS)

    left_images, right_images = find_images(images_dir)

    if not left_images or not right_images:
        print(f"ERROR: No images found in {images_dir}")
        print("Expected either:")
        print(f"  {images_dir}/left/left_XXXX.png  and  {images_dir}/right/right_XXXX.png")
        print(f"  {images_dir}/left-XXXX.png        and  {images_dir}/right-XXXX.png")
        sys.exit(1)

    if len(left_images) != len(right_images):
        print(f"ERROR: Mismatched image counts: {len(left_images)} left, {len(right_images)} right")
        sys.exit(1)

    print(f"Found {len(left_images)} image pairs")
    print(f"Board size: {BOARD_COLS}x{BOARD_ROWS} inner corners, {SQUARE_SIZE*100:.1f}cm squares\n")

    # Find corners in each camera separately
    print("Finding corners in left images...")
    obj_pts_l, img_pts_l, valid_l = find_corners(left_images, board_size)

    print("Finding corners in right images...")
    obj_pts_r, img_pts_r, valid_r = find_corners(right_images, board_size)

    # Keep only pairs where both images had valid corners.
    # Images are paired by sorted position (left-0000 pairs with right-0000, etc.),
    # so we match by index rather than filename to support any naming convention.
    valid_l_set = set(valid_l)
    valid_r_set = set(valid_r)

    obj_pts = []
    img_pts_left = []
    img_pts_right = []

    for lpath, rpath in zip(left_images, right_images):
        if lpath in valid_l_set and rpath in valid_r_set:
            idx_l = valid_l.index(lpath)
            idx_r = valid_r.index(rpath)
            obj_pts.append(obj_pts_l[idx_l])
            img_pts_left.append(img_pts_l[idx_l])
            img_pts_right.append(img_pts_r[idx_r])

    print(f"\nValid pairs for calibration: {len(obj_pts)} / {len(left_images)}")
    if len(obj_pts) < 10:
        print("WARNING: fewer than 10 valid pairs - results may be unreliable")
    if len(obj_pts) == 0:
        print("ERROR: No valid pairs found - check your checkerboard size settings")
        sys.exit(1)

    # Get image size
    sample = cv2.imread(left_images[0])
    image_size = (sample.shape[1], sample.shape[0])
    print(f"Image size: {image_size[0]}x{image_size[1]}\n")

    # Calibrate each camera individually
    print("Calibrating left camera...")
    ret_l, K_l, D_l, _, _ = cv2.calibrateCamera(obj_pts, img_pts_left, image_size, None, None)
    print(f"  Left reprojection error: {ret_l:.4f}px")

    print("Calibrating right camera...")
    ret_r, K_r, D_r, _, _ = cv2.calibrateCamera(obj_pts, img_pts_right, image_size, None, None)
    print(f"  Right reprojection error: {ret_r:.4f}px")

    # Stereo calibration
    # CALIB_USE_INTRINSIC_GUESS uses mono calibration as starting point but jointly
    # refines intrinsics + extrinsics together, giving much lower stereo error than
    # CALIB_FIX_INTRINSIC which locks intrinsics and can't compensate for their errors.
    print("Running stereo calibration...")
    flags = cv2.CALIB_USE_INTRINSIC_GUESS
    ret_stereo, K_l, D_l, K_r, D_r, R, T, E, F = cv2.stereoCalibrate(
        obj_pts, img_pts_left, img_pts_right,
        K_l, D_l, K_r, D_r,
        image_size,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    )
    print(f"  Stereo reprojection error: {ret_stereo:.4f}px\n")

    if ret_stereo > 1.0:
        print("WARNING: reprojection error > 1px - consider recapturing with better coverage")

    # Build body_T_cam matrices (4x4 transform, identity for left, R|T for right)
    body_T_cam0 = np.eye(4)

    body_T_cam1 = np.eye(4)
    body_T_cam1[:3, :3] = R
    body_T_cam1[:3, 3] = T.flatten()

    # Extract values
    fxl, fyl = K_l[0, 0], K_l[1, 1]
    cxl, cyl = K_l[0, 2], K_l[1, 2]
    k1l, k2l, p1l, p2l = D_l[0, 0], D_l[0, 1], D_l[0, 2], D_l[0, 3]

    fxr, fyr = K_r[0, 0], K_r[1, 1]
    cxr, cyr = K_r[0, 2], K_r[1, 2]
    k1r, k2r, p1r, p2r = D_r[0, 0], D_r[0, 1], D_r[0, 2], D_r[0, 3]

    baseline = np.linalg.norm(T)

    # Print results
    print("=" * 60)
    print("CALIBRATION RESULTS - paste into your OV2SLAM YAML file")
    print("=" * 60)
    print()
    print(f"Camera.topic_left: /left/image_raw")
    print(f"Camera.topic_right: /right/image_raw")
    print()
    print(f"Camera.model_left: pinhole")
    print(f"Camera.model_right: pinhole")
    print()
    print(f"Camera.left_nwidth: {image_size[0]}")
    print(f"Camera.left_nheight: {image_size[1]}")
    print(f"Camera.right_nwidth: {image_size[0]}")
    print(f"Camera.right_nheight: {image_size[1]}")
    print()
    print(f"Camera.fxl: {fxl:.6f}")
    print(f"Camera.fyl: {fyl:.6f}")
    print(f"Camera.cxl: {cxl:.6f}")
    print(f"Camera.cyl: {cyl:.6f}")
    print()
    print(f"Camera.k1l: {k1l:.10f}")
    print(f"Camera.k2l: {k2l:.10f}")
    print(f"Camera.p1l: {p1l:.10f}")
    print(f"Camera.p2l: {p2l:.10f}")
    print()
    print(f"Camera.fxr: {fxr:.6f}")
    print(f"Camera.fyr: {fyr:.6f}")
    print(f"Camera.cxr: {cxr:.6f}")
    print(f"Camera.cyr: {cyr:.6f}")
    print()
    print(f"Camera.k1r: {k1r:.10f}")
    print(f"Camera.k2r: {k2r:.10f}")
    print(f"Camera.p1r: {p1r:.10f}")
    print(f"Camera.p2r: {p2r:.10f}")
    print()
    print(format_matrix("body_T_cam0", body_T_cam0))
    print()
    print(format_matrix("body_T_cam1", body_T_cam1))
    print()
    print(f"# Baseline: {baseline*100:.2f}cm")
    print(f"# Stereo reprojection error: {ret_stereo:.4f}px")
    print("=" * 60)

    # Also save to file
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_result.txt")
    with open(out_path, "w") as f:
        f.write(f"Camera.topic_left: /left/image_raw\n")
        f.write(f"Camera.topic_right: /right/image_raw\n\n")
        f.write(f"Camera.model_left: pinhole\n")
        f.write(f"Camera.model_right: pinhole\n\n")
        f.write(f"Camera.left_nwidth: {image_size[0]}\n")
        f.write(f"Camera.left_nheight: {image_size[1]}\n")
        f.write(f"Camera.right_nwidth: {image_size[0]}\n")
        f.write(f"Camera.right_nheight: {image_size[1]}\n\n")
        f.write(f"Camera.fxl: {fxl:.6f}\n")
        f.write(f"Camera.fyl: {fyl:.6f}\n")
        f.write(f"Camera.cxl: {cxl:.6f}\n")
        f.write(f"Camera.cyl: {cyl:.6f}\n\n")
        f.write(f"Camera.k1l: {k1l:.10f}\n")
        f.write(f"Camera.k2l: {k2l:.10f}\n")
        f.write(f"Camera.p1l: {p1l:.10f}\n")
        f.write(f"Camera.p2l: {p2l:.10f}\n\n")
        f.write(f"Camera.fxr: {fxr:.6f}\n")
        f.write(f"Camera.fyr: {fyr:.6f}\n")
        f.write(f"Camera.cxr: {cxr:.6f}\n")
        f.write(f"Camera.cyr: {cyr:.6f}\n\n")
        f.write(f"Camera.k1r: {k1r:.10f}\n")
        f.write(f"Camera.k2r: {k2r:.10f}\n")
        f.write(f"Camera.p1r: {p1r:.10f}\n")
        f.write(f"Camera.p2r: {p2r:.10f}\n\n")
        f.write(format_matrix("body_T_cam0", body_T_cam0) + "\n\n")
        f.write(format_matrix("body_T_cam1", body_T_cam1) + "\n\n")
        f.write(f"# Baseline: {baseline*100:.2f}cm\n")
        f.write(f"# Stereo reprojection error: {ret_stereo:.4f}px\n")

    print(f"\nResults also saved to {out_path}")


if __name__ == "__main__":
    main()
