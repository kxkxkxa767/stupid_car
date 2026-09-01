#!/usr/bin/env python3
"""在去畸变图像中点选地面矩形，生成像素到车辆米制坐标的单应矩阵。"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


LABELS = ("近端左点", "近端右点", "远端左点", "远端右点")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--output", type=Path,
                        default=Path("main/camera_calibration/output/ground_projection.json"))
    parser.add_argument("--near-m", type=float, default=0.30)
    parser.add_argument("--far-m", type=float, default=1.20)
    parser.add_argument("--half-width-m", type=float, default=0.25)
    parser.add_argument(
        "--image-points",
        help="无需窗口，直接传入 x,y;x,y;x,y;x,y（顺序仍为近左、近右、远左、远右）",
    )
    parser.add_argument(
        "--raw-pixel-homography", action="store_true",
        help="直接在原始像素上拟合地面单应矩阵，不对中央区内参向画面边缘外推",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not (0.0 < args.near_m < args.far_m and args.half_width_m > 0.0):
        raise SystemExit("地面矩形尺寸必须满足 0 < near < far 且 half-width > 0")
    intrinsics = json.loads(args.intrinsics.read_text())
    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"无法读取图片: {args.image}")
    expected = (intrinsics["image_width"], intrinsics["image_height"])
    actual = (image.shape[1], image.shape[0])
    if actual != expected:
        raise SystemExit(f"图片分辨率 {actual} 与内参分辨率 {expected} 不一致")

    camera_matrix = np.asarray(intrinsics["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.asarray(intrinsics["dist_coeffs"], dtype=np.float64)
    undistorted = (image.copy() if args.raw_pixel_homography
                   else cv2.undistort(image, camera_matrix, dist_coeffs))
    points: list[tuple[float, float]] = []
    if args.image_points:
        try:
            points = [tuple(map(float, pair.split(",")))
                      for pair in args.image_points.split(";")]
        except ValueError as error:
            raise SystemExit("--image-points 格式错误") from error
        if len(points) != 4 or any(len(point) != 2 for point in points):
            raise SystemExit("--image-points 必须恰好包含 4 个 x,y 点")
    display = undistorted.copy()

    def redraw() -> None:
        display[:] = undistorted
        for index, point in enumerate(points):
            p = tuple(round(value) for value in point)
            cv2.circle(display, p, 6, (0, 0, 255), -1)
            cv2.putText(display, str(index + 1), (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        next_label = LABELS[len(points)] if len(points) < 4 else "按 Enter 保存"
        cv2.putText(display, f"下一点: {next_label} | U 撤销 | Esc 退出",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.imshow("XT-NetRC ground calibration", display)

    def mouse(event: int, x: int, y: int, _flags: int, _data: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((float(x), float(y)))
            redraw()

    if not args.image_points:
        cv2.namedWindow("XT-NetRC ground calibration", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("XT-NetRC ground calibration", mouse)
        redraw()
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == 27:
                cv2.destroyAllWindows()
                raise SystemExit("用户取消")
            if key in (ord("u"), ord("U")) and points:
                points.pop()
                redraw()
            if key in (10, 13) and len(points) == 4:
                break
        cv2.destroyAllWindows()

    image_points = np.asarray(points, dtype=np.float64)
    vehicle_points = np.asarray([
        [args.near_m, args.half_width_m],
        [args.near_m, -args.half_width_m],
        [args.far_m, args.half_width_m],
        [args.far_m, -args.half_width_m],
    ], dtype=np.float64)
    homography = cv2.getPerspectiveTransform(
        image_points.astype(np.float32), vehicle_points.astype(np.float32)
    )
    projected = cv2.perspectiveTransform(image_points.reshape(1, -1, 2), homography)[0]
    max_error = float(np.max(np.linalg.norm(projected - vehicle_points, axis=1)))

    result = dict(intrinsics)
    result.update({
        "ground_calibration_version": 1,
        "coordinate_system": {"x": "forward_m", "y": "left_m"},
        "image_to_vehicle_ground_homography": homography.tolist(),
        "ground_reference_image": str(args.image),
        "image_points_undistorted_px": image_points.tolist(),
        "vehicle_points_m": vehicle_points.tolist(),
        "calibration_max_error_m": max_error,
        "projection_model": ("raw_pixel_homography" if args.raw_pixel_homography
                             else "undistorted_pixel_homography"),
        "projection_dist_coeffs": (
            [0.0] * len(intrinsics["dist_coeffs"])
            if args.raw_pixel_homography else intrinsics["dist_coeffs"]
        ),
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(f"标定点最大回代误差: {max_error:.8f} m")
    print(f"已保存: {args.output}")


if __name__ == "__main__":
    main()
