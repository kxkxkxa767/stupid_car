import cv2
import numpy as np

camera_id = 0
width = 640
height = 480
fps = 120

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

if not cap.isOpened():
    print("无法打开摄像头画面")
    exit()

cv2.namedWindow('Blue Mask')
cv2.namedWindow('ROI')

def nothing(x):
    pass

# 蓝色HSV分割参数可调
cv2.createTrackbar('H Low', 'Blue Mask', 100, 180, nothing)
cv2.createTrackbar('H High', 'Blue Mask', 140, 180, nothing)
cv2.createTrackbar('S Low', 'Blue Mask', 10, 255, nothing)
cv2.createTrackbar('S High', 'Blue Mask', 255, 255, nothing)
cv2.createTrackbar('V Low', 'Blue Mask', 10, 255, nothing)
cv2.createTrackbar('V High', 'Blue Mask', 255, 255, nothing)

fixed_size = (200, 200)

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法打开摄像头画面")
        break

    frame_half = frame[height//2:, :]

    # 获取滑动条的值
    h_low = cv2.getTrackbarPos('H Low', 'Blue Mask')
    h_high = cv2.getTrackbarPos('H High', 'Blue Mask')
    s_low = cv2.getTrackbarPos('S Low', 'Blue Mask')
    s_high = cv2.getTrackbarPos('S High', 'Blue Mask')
    v_low = cv2.getTrackbarPos('V Low', 'Blue Mask')
    v_high = cv2.getTrackbarPos('V High', 'Blue Mask')

    hsv = cv2.cvtColor(frame_half, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([h_low, s_low, v_low])
    upper_blue = np.array([h_high, s_high, v_high])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 找底行大于220且面积最大的轮廓
    max_cnt = None
    max_area = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x, y, w_rect, h_rect = cv2.boundingRect(cnt)
        bottom = y + h_rect
        area = cv2.contourArea(cnt)
        if bottom < 230 and bottom > 180 and area > max_area and area >5000:
            max_area = area
            max_cnt = cnt

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    roi_img = np.ones(fixed_size, dtype=np.uint8) * 255  # 默认全白
    show_img = mask_bgr.copy()
    blue_bottom = -1
    if max_cnt is not None:
        rect = cv2.minAreaRect(max_cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        fill_mask = np.zeros_like(mask)
        cv2.drawContours(fill_mask, [box], 0, 255, -1)
        cv2.drawContours(fill_mask, [max_cnt], -1, 0, -1)

        show_img = mask_bgr.copy()
        show_img[fill_mask == 255] = [255,255,255]

        w = int(rect[1][0])
        h = int(rect[1][1])
        if w > 0 and h > 0:
            src_pts = box.astype("float32")
            dst_pts = np.array([[0, h-1], [0, 0], [w-1, 0], [w-1, h-1]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            roi_bin = cv2.warpPerspective(show_img, M, (w, h))
            roi_img = cv2.resize(roi_bin, fixed_size)
            roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
            roi_img = cv2.bitwise_not(roi_img)

            h2, w2 = roi_img.shape
            col_sum = np.sum(roi_img == 255, axis=0)
            threshold = int(0.7 * np.max(col_sum)) if np.max(col_sum) > 0 else 0
            white_bar_indices = np.where(col_sum > threshold)[0]
            if len(white_bar_indices) > 0:
                bar_left = white_bar_indices[0]
                bar_right = white_bar_indices[-1]
                bar_center = (bar_left + bar_right) // 2
            else:
                bar_center = w2 // 2

            left_top = roi_img[0:h2//2, 0:bar_center]
            right_top = roi_img[0:h2//2, bar_center:w2]
            left_white_ratio = np.sum(left_top == 255) / left_top.size if left_top.size > 0 else 0
            right_white_ratio = np.sum(right_top == 255) / right_top.size if right_top.size > 0 else 0

            if left_white_ratio <= 0.05 and right_white_ratio <= 0.05:
                pass
            elif left_white_ratio > right_white_ratio:
                print(f"蓝色竖条为界，左上白色比例高({left_white_ratio:.2f} > {right_white_ratio:.2f})，左转")
            elif right_white_ratio > left_white_ratio:
                print(f"蓝色竖条为界，右上白色比例高({right_white_ratio:.2f} > {left_white_ratio:.2f})，右转")

        # 输出蓝色矩形底行行数
        x, y, w_rect, h_rect = cv2.boundingRect(max_cnt)
        blue_bottom = y + h_rect
        print("蓝色矩形底行数:", blue_bottom)
        print("蓝色矩形面积:", max_area)
    else:
        print("无有效蓝色区域或底行不达标，不做识别")

    cv2.imshow('Blue Mask', show_img)
    cv2.imshow('ROI', roi_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()