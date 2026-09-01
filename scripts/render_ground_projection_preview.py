#!/usr/bin/env python3
"""把地面单应性配置渲染成带米制网格的鸟瞰预览图。"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--x-min", type=float, default=0.50)
    parser.add_argument("--x-max", type=float, default=1.40)
    parser.add_argument("--half-width", type=float, default=0.35)
    parser.add_argument("--pixels-per-meter", type=float, default=500.0)
    args = parser.parse_args()

    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"无法读取 {args.image}")
    config = json.loads(args.config.read_text())
    homography = np.asarray(
        config["image_to_vehicle_ground_homography"], dtype=np.float64
    )
    scale = args.pixels_per_meter
    width = round(2.0 * args.half_width * scale)
    height = round((args.x_max - args.x_min) * scale)
    ground_to_canvas = np.asarray([
        [0.0, -scale, args.half_width * scale],
        [-scale, 0.0, args.x_max * scale],
        [0.0, 0.0, 1.0],
    ])
    preview = cv2.warpPerspective(
        image, ground_to_canvas @ homography, (width, height),
        flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
        borderValue=(40, 40, 40),
    )

    for x_m in np.arange(args.x_min, args.x_max + 0.001, 0.10):
        y = round((args.x_max - x_m) * scale)
        cv2.line(preview, (0, y), (width - 1, y), (0, 180, 255), 1)
        cv2.putText(preview, f"x={x_m:.1f}m", (4, max(12, y - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 180, 255), 1)
    for y_m in np.arange(-0.30, 0.301, 0.10):
        x = round((args.half_width - y_m) * scale)
        cv2.line(preview, (x, 0), (x, height - 1), (0, 255, 120), 1)
    center_x = round(args.half_width * scale)
    cv2.line(preview, (center_x, 0), (center_x, height - 1), (255, 255, 255), 2)
    cv2.putText(preview, "vehicle center y=0", (center_x + 5, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    for index, (x_m, y_m) in enumerate(config.get("vehicle_points_m", []), 1):
        point = (round((args.half_width - y_m) * scale),
                 round((args.x_max - x_m) * scale))
        cv2.circle(preview, point, 6, (0, 0, 255), -1)
        cv2.circle(preview, point, 8, (255, 255, 255), 1)
        cv2.putText(preview, str(index), (point[0] + 7, point[1] - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), preview):
        raise SystemExit(f"无法写入 {args.output}")
    print(args.output)


if __name__ == "__main__":
    main()
