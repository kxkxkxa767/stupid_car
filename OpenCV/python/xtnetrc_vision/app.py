import argparse
from pathlib import Path
from typing import List, Optional, Union

import cv2

from .lane_detector import LaneConfig, LaneDetector


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="XT-NetRC Python 车道视觉测试")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera", type=int, default=0, help="摄像头编号，默认 0")
    source.add_argument("--video", type=Path, help="读取本地视频文件")
    parser.add_argument("--headless", action="store_true", help="不显示窗口")
    parser.add_argument("--canny-low", type=float, default=40.0)
    parser.add_argument("--canny-high", type=float, default=80.0)
    parser.add_argument("--kp", type=float, default=0.15)
    parser.add_argument("--kd", type=float, default=0.10)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    source: Union[int, str] = (
        args.camera if args.video is None else str(args.video.expanduser())
    )
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        print("无法打开摄像头或视频输入。")
        print("Mac 首次使用时请在系统弹窗中允许 VS Code 或终端访问摄像头。")
        return 1

    detector = LaneDetector(
        LaneConfig(
            canny_low=args.canny_low,
            canny_high=args.canny_high,
            kp=args.kp,
            kd=args.kd,
        )
    )
    frame_number = 0

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                break
            result = detector.process(frame)
            if frame_number % 10 == 0:
                print(
                    f"detected={'yes' if result.detected else 'no'} "
                    f"confidence={result.confidence:.2f} "
                    f"error_px={result.error_px:.2f} "
                    f"simulated_steering_deg={result.steering_deg:.2f}"
                )
            frame_number += 1

            if not args.headless:
                cv2.imshow("XT-NetRC Python lane view", result.overlay)
                cv2.imshow("XT-NetRC Python edges", result.edges)
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    break
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0
