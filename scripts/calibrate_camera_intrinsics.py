#!/usr/bin/env python3
"""从 XT-NetRC ChArUco 照片计算相机内参和畸变系数。"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", type=Path, required=True,
                        help="包含 jpg/png 标定照片的目录")
    parser.add_argument("--output", type=Path,
                        default=Path("main/camera_calibration/output/camera_intrinsics.json"))
    parser.add_argument("--review-dir", type=Path,
                        default=Path("main/camera_calibration/output/intrinsics_review"))
    parser.add_argument("--min-frames", type=int, default=12)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = sorted(path for path in args.images.iterdir()
                   if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not paths:
        raise SystemExit(f"没有找到标定图片: {args.images}")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((5, 7), 0.035, 0.025, dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted: list[tuple[Path, np.ndarray, np.ndarray, np.ndarray, str]] = []
    corner_sets: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    args.review_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"跳过无法读取的图片: {path}")
            continue
        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            print(f"跳过尺寸不一致图片 {path}: {current_size} != {image_size}")
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _, _ = detector.detectBoard(gray)
        count = 0 if ids is None else len(ids)
        method = "charuco"
        if count >= 6:
            obj, img = board.matchImagePoints(corners, ids)
            review_corners = np.asarray(corners).reshape(-1, 1, 2)
            review_ids = np.asarray(ids).reshape(-1, 1)
        else:
            # 这台广角定焦相机近距离会模糊 ArUco 单元，但黑白格交点仍很稳定。
            # 对灰度图反色后，OpenCV 可按 4x6 个内角点回退检测。
            found, chess_corners = cv2.findChessboardCorners(
                255 - gray, (4, 6),
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if not found:
                print(f"拒绝 {path.name}: ChArUco 和棋盘角点均未识别")
                continue
            cv2.cornerSubPix(
                gray, chess_corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.001),
            )
            obj = np.zeros((24, 1, 3), np.float32)
            obj[:, 0, :2] = np.mgrid[0:4, 0:6].T.reshape(-1, 2) * 0.035
            img = chess_corners.astype(np.float32)
            review_corners = chess_corners
            review_ids = np.empty((0, 1), np.int32)
            method = "inverted_chessboard_fallback"
        object_points.append(np.asarray(obj, dtype=np.float32))
        image_points.append(np.asarray(img, dtype=np.float32))
        corner_sets.append(np.asarray(img).reshape(-1, 2))
        accepted.append((path, image, review_corners, review_ids, method))

    if image_size is None or len(accepted) < args.min_frames:
        raise SystemExit(
            f"有效标定图只有 {len(accepted)} 张，至少需要 {args.min_frames} 张；"
            "请改变标定板距离、角度和画面位置后重新采集。"
        )

    reference = corner_sets[0]
    pose_changes = [float(np.mean(np.linalg.norm(points - reference, axis=1)))
                    for points in corner_sets if points.shape == reference.shape]
    if not pose_changes or max(pose_changes) < 25.0:
        raise SystemExit(
            f"虽然识别到 {len(accepted)} 张，但标定板姿势几乎没有变化"
            f"（最大平均角点位移 {max(pose_changes, default=0.0):.2f} px）。"
            "必须移动到画面不同区域，并改变距离、俯仰和旋转角度后重新采集。"
        )

    (rms, camera_matrix, dist_coeffs, rvecs, tvecs,
     intrinsic_std, _extrinsic_std, per_view_errors) = cv2.calibrateCameraExtended(
        object_points, image_points, image_size, None, None
    )
    errors = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        residual = np.asarray(img).reshape(-1, 2) - projected.reshape(-1, 2)
        errors.append(float(np.linalg.norm(residual) / len(projected)))

    for index, (path, image, corners, ids, method) in enumerate(accepted):
        review = image.copy()
        if method == "charuco":
            cv2.aruco.drawDetectedCornersCharuco(review, corners, ids)
        else:
            cv2.drawChessboardCorners(review, (4, 6), corners, True)
        cv2.imwrite(str(args.review_dir / f"{index:02d}_{path.name}"), review)

    result = {
        "version": 1,
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "rms_reprojection_error_px": float(rms),
        "mean_per_view_error_px": float(np.mean(errors)),
        "max_view_rms_error_px": float(np.max(per_view_errors)),
        "intrinsic_standard_deviation": intrinsic_std.reshape(-1).tolist(),
        "frames_used": len(accepted),
        "charuco": {
            "dictionary": "DICT_5X5_100",
            "squares_x": 5,
            "squares_y": 7,
            "square_length_m": 0.035,
            "marker_length_m": 0.025,
        },
        "source_images": [path.name for path, *_ in accepted],
        "detection_methods": [method for *_, method in accepted],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(f"使用 {len(accepted)} 张图片，分辨率 {image_size[0]}x{image_size[1]}")
    print(f"RMS 重投影误差: {rms:.4f} px")
    print(f"平均单视图误差: {np.mean(errors):.4f} px")
    print(f"已保存: {args.output}")


if __name__ == "__main__":
    main()
