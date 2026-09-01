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
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Red Mask')
cv2.namedWindow('ROI')

def nothing(x):
    pass

# 低区间
cv2.createTrackbar('H Low 1', 'Red Mask', 0, 180, nothing)
cv2.createTrackbar('H High 1', 'Red Mask', 13, 180, nothing)
cv2.createTrackbar('S Low 1', 'Red Mask', 75, 255, nothing)
cv2.createTrackbar('V Low 1', 'Red Mask', 100, 255, nothing)
cv2.createTrackbar('V High 1', 'Red Mask', 255, 255, nothing)
# 高区间
cv2.createTrackbar('H Low 2', 'Red Mask', 160, 180, nothing)
cv2.createTrackbar('H High 2', 'Red Mask', 180, 180, nothing)
cv2.createTrackbar('S Low 2', 'Red Mask', 90, 255, nothing)
cv2.createTrackbar('V Low 2', 'Red Mask', 100, 255, nothing)
cv2.createTrackbar('V High 2', 'Red Mask', 255, 255, nothing)

fixed_size = (200, 200)

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法打开摄像头画面")
        break

    # 只保留下半部分
    frame_half = frame[height//2:, :]

    # 获取滑动条的值
    h_low1 = cv2.getTrackbarPos('H Low 1', 'Red Mask')
    h_high1 = cv2.getTrackbarPos('H High 1', 'Red Mask')
    s_low1 = cv2.getTrackbarPos('S Low 1', 'Red Mask')
    v_low1 = cv2.getTrackbarPos('V Low 1', 'Red Mask')
    v_high1 = cv2.getTrackbarPos('V High 1', 'Red Mask')

    h_low2 = cv2.getTrackbarPos('H Low 2', 'Red Mask')
    h_high2 = cv2.getTrackbarPos('H High 2', 'Red Mask')
    s_low2 = cv2.getTrackbarPos('S Low 2', 'Red Mask')
    v_low2 = cv2.getTrackbarPos('V Low 2', 'Red Mask')
    v_high2 = cv2.getTrackbarPos('V High 2', 'Red Mask')

    s_high1 = 255
    s_high2 = 255

    # 红色域分割
    hsv = cv2.cvtColor(frame_half, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([h_low1, s_low1, v_low1])
    upper_red1 = np.array([h_high1, s_high1, v_high1])
    lower_red2 = np.array([h_low2, s_low2, v_low2])
    upper_red2 = np.array([h_high2, s_high2, v_high2])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 找最大轮廓
    max_cnt = None
    max_area = 0
    for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_cnt = cnt

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    roi_img = np.ones(fixed_size, dtype=np.uint8) * 255  # 默认全白
    show_img = mask_bgr.copy()
    if max_cnt is not None:
        # 最小外接旋转矩形
        rect = cv2.minAreaRect(max_cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 创建掩码：外层矩形填白，内层多边形填黑
        fill_mask = np.zeros_like(mask)
        cv2.drawContours(fill_mask, [box], 0, 255, -1)      # 外层矩形填白
        cv2.drawContours(fill_mask, [max_cnt], -1, 0, -1)   # 内层多边形填黑

        # 把掩码叠加到原图上（只显示白色填充区域，其它为原图）
        show_img = mask_bgr.copy()
        show_img[fill_mask == 255] = [255,255,255]

        # 透视变换将原图内容摆正（提取绿色外接矩形区域）
        w = int(rect[1][0])
        h = int(rect[1][1])
        if w > 0 and h > 0:
            src_pts = box.astype("float32")
            dst_pts = np.array([[0, h-1], [0, 0], [w-1, 0], [w-1, h-1]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            roi_bin = cv2.warpPerspective(show_img, M, (w, h))
            roi_img = cv2.resize(roi_bin, fixed_size)
            # 反色并转为单通道
            roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
            roi_img = cv2.bitwise_not(roi_img)

            # 以竖直白条为基准分割左右
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
                pass  # 什么都不输出
            elif left_white_ratio > right_white_ratio:
                print(f"以竖条为界，左上白色比例高({left_white_ratio:.2f} > {right_white_ratio:.2f})，左转")
            elif right_white_ratio > left_white_ratio:
                print(f"以竖条为界，右上白色比例高({right_white_ratio:.2f} > {left_white_ratio:.2f})，右转")

    cv2.imshow('Red Mask', show_img)
    cv2.imshow('ROI', roi_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()