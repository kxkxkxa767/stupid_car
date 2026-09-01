#!/usr/bin/env python3
"""生成 XT-NetRC 相机标定使用的高分辨率 ChArUco 棋盘图。"""

from pathlib import Path

import cv2


SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH_MM = 35.0
MARKER_LENGTH_MM = 25.0
DPI = 300


def main() -> None:
    output = Path(__file__).resolve().parents[1] / "tmp/pdfs/xtnetrc_charuco_board.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM / 1000.0,
        MARKER_LENGTH_MM / 1000.0,
        dictionary,
    )
    width_px = round(SQUARES_X * SQUARE_LENGTH_MM / 25.4 * DPI)
    height_px = round(SQUARES_Y * SQUARE_LENGTH_MM / 25.4 * DPI)
    image = board.generateImage((width_px, height_px), marginSize=0, borderBits=1)
    if not cv2.imwrite(str(output), image):
        raise RuntimeError(f"无法写入 {output}")
    print(output)


if __name__ == "__main__":
    main()
