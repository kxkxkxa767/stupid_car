import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xtnetrc_vision import LaneDetector  # noqa: E402


def make_lane_frame(shift_x: int) -> np.ndarray:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.line(frame, (200 + shift_x, 250), (200 + shift_x, 470), (255, 255, 255), 8)
    cv2.line(frame, (440 + shift_x, 250), (440 + shift_x, 470), (255, 255, 255), 8)
    return frame


class LaneDetectorTests(unittest.TestCase):
    def test_centered_lane(self) -> None:
        result = LaneDetector().process(make_lane_frame(0))
        self.assertTrue(result.detected)
        self.assertGreater(result.confidence, 0.90)
        self.assertLess(abs(result.error_px), 5.0)

    def test_right_shift_requests_left_correction(self) -> None:
        result = LaneDetector().process(make_lane_frame(50))
        self.assertTrue(result.detected)
        self.assertGreater(result.error_px, 40.0)
        self.assertLess(result.steering_deg, 85.0)

    def test_empty_input_is_safe(self) -> None:
        result = LaneDetector().process(None)
        self.assertFalse(result.detected)
        self.assertEqual(result.steering_deg, 85.0)


if __name__ == "__main__":
    unittest.main()
