#!/usr/bin/env python3
"""将生成的棋盘图按精确毫米尺寸放入 A4 PDF。"""

from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    image_path = root / "tmp/pdfs/xtnetrc_charuco_board.png"
    output_path = root / "output/pdf/XT-NetRC_Charuco相机标定板_A4.pdf"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    page_width, page_height = A4
    board_width = 175 * mm
    board_height = 245 * mm
    board_x = (page_width - board_width) / 2
    board_y = 25 * mm

    pdf = canvas.Canvas(str(output_path), pagesize=A4, pageCompression=1)
    pdf.setTitle("XT-NetRC ChArUco Camera Calibration Board")
    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawCentredString(page_width / 2, page_height - 10 * mm,
                          "XT-NetRC ChArUco Camera Calibration Board")
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(
        page_width / 2,
        page_height - 14 * mm,
        "5 x 7 squares | square 35.0 mm | marker 25.0 mm | DICT_5X5_100",
    )
    pdf.drawImage(str(image_path), board_x, board_y,
                  width=board_width, height=board_height,
                  preserveAspectRatio=False, mask="auto")

    # 100 mm 校验线：打印后必须用尺测量，防止打印机自动缩放。
    line_x = (page_width - 100 * mm) / 2
    line_y = 13 * mm
    pdf.setLineWidth(0.4)
    pdf.line(line_x, line_y, line_x + 100 * mm, line_y)
    pdf.line(line_x, line_y - 2 * mm, line_x, line_y + 2 * mm)
    pdf.line(line_x + 100 * mm, line_y - 2 * mm,
             line_x + 100 * mm, line_y + 2 * mm)
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(page_width / 2, 8 * mm,
                          "Print at 100% / Actual size. This line must measure 100.0 mm.")
    pdf.showPage()
    pdf.save()
    print(output_path)


if __name__ == "__main__":
    main()
