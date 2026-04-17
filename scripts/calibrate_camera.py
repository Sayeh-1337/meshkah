#!/usr/bin/env python3
import argparse
import glob
import json

import cv2
import numpy as np


def calibrate(image_glob: str, cols: int, rows: int, square_size: float) -> dict:
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size
    obj_points = []
    img_points = []
    image_size = None

    for path in sorted(glob.glob(image_glob)):
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            continue
        obj_points.append(objp)
        img_points.append(corners)
        image_size = gray.shape[::-1]

    if not obj_points or image_size is None:
        raise RuntimeError("No valid chessboard detections found.")

    ret, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)
    if not ret:
        raise RuntimeError("Calibration failed.")
    return {
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
        "dist": dist.reshape(-1).tolist(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True, help="Glob to chessboard images")
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.024)
    parser.add_argument("--out", default="camera_intrinsics.yaml")
    args = parser.parse_args()

    params = calibrate(args.images, args.cols, args.rows, args.square_size)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(
            "camera:\n"
            f"  fx: {params['fx']}\n"
            f"  fy: {params['fy']}\n"
            f"  cx: {params['cx']}\n"
            f"  cy: {params['cy']}\n"
            f"  dist: {params['dist']}\n"
        )
    print(f"Wrote calibration to {args.out}")


if __name__ == "__main__":
    main()
