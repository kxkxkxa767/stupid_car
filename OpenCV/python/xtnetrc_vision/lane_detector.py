from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np


@dataclass
class LaneConfig:
    roi_top_ratio: float = 0.55
    roi_bottom_ratio: float = 0.95
    scan_rows: int = 100
    canny_low: float = 40.0
    canny_high: float = 80.0
    kp: float = 0.15
    kd: float = 0.10
    steering_center_deg: float = 85.0
    steering_min_deg: float = 60.0
    steering_max_deg: float = 120.0


@dataclass
class LaneResult:
    detected: bool = False
    midpoint_x: float = 0.0
    error_px: float = 0.0
    steering_deg: float = 85.0
    confidence: float = 0.0
    edges: np.ndarray = field(
        default_factory=lambda: np.empty((0, 0), dtype=np.uint8)
    )
    overlay: np.ndarray = field(
        default_factory=lambda: np.empty((0, 0, 3), dtype=np.uint8)
    )


class LaneDetector:
    """Scan-line lane detector matching the safe C++ prototype."""

    def __init__(self, config: Optional[LaneConfig] = None) -> None:
        self.config = config or LaneConfig()
        self._last_error = 0.0

    def process(self, frame: Optional[np.ndarray]) -> LaneResult:
        config = self.config
        result = LaneResult(steering_deg=config.steering_center_deg)
        if frame is None or frame.size == 0:
            return result

        height, width = frame.shape[:2]
        top = int(round(height * config.roi_top_ratio))
        top = min(max(top, 0), height - 1)
        bottom = int(round(height * config.roi_bottom_ratio))
        bottom = min(max(bottom, top + 1), height)

        roi = frame[top:bottom, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0.5)
        result.edges = cv2.Canny(
            blurred, config.canny_low, config.canny_high, apertureSize=3
        )
        result.overlay = frame.copy()
        cv2.rectangle(
            result.overlay, (0, top), (width - 1, bottom - 1), (70, 70, 70), 1
        )

        rows_to_scan = min(config.scan_rows, result.edges.shape[0])
        center_x = result.edges.shape[1] // 2
        midpoint_sum = 0.0
        valid_rows = 0

        for index in range(rows_to_scan):
            y = result.edges.shape[0] - 1 - index
            row = result.edges[y]
            left_candidates = np.flatnonzero(row[: center_x + 1])
            right_candidates = np.flatnonzero(row[center_x:])
            if left_candidates.size == 0 or right_candidates.size == 0:
                continue

            left_x = int(left_candidates[-1])
            right_x = int(center_x + right_candidates[0])
            if right_x <= left_x:
                continue

            midpoint_x = (left_x + right_x) // 2
            overlay_y = top + y
            midpoint_sum += midpoint_x
            valid_rows += 1
            cv2.circle(result.overlay, (left_x, overlay_y), 2, (255, 80, 0), -1)
            cv2.circle(result.overlay, (right_x, overlay_y), 2, (255, 80, 0), -1)
            cv2.circle(result.overlay, (midpoint_x, overlay_y), 2, (0, 220, 255), -1)

        result.confidence = valid_rows / rows_to_scan if rows_to_scan else 0.0
        if valid_rows == 0:
            return result

        result.detected = True
        result.midpoint_x = midpoint_sum / valid_rows
        result.error_px = result.midpoint_x - center_x
        correction = (
            config.kp * result.error_px
            + config.kd * (result.error_px - self._last_error)
        )
        self._last_error = result.error_px
        result.steering_deg = float(
            np.clip(
                config.steering_center_deg - correction,
                config.steering_min_deg,
                config.steering_max_deg,
            )
        )
        cv2.line(
            result.overlay, (center_x, top), (center_x, bottom - 1), (0, 255, 0), 1
        )
        return result
