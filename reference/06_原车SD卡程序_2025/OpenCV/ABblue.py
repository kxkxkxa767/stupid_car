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

cv2.namedWindow('Blue Mask')
cv2.namedWindow('ROI')

def nothing(x):
    pass

# HSV空间蓝色分割参数可调
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
        print("无法读取摄像头画面")
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
    max_bottom = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        x, y, w_rect, h_rect = cv2.boundingRect(cnt)
        bottom = y + h_rect
        area = cv2.contourArea(cnt)
        if bottom < 220 and bottom > 120 and area > max_area:
            max_area = area
            max_cnt = cnt
            max_bottom = bottom

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    roi_img = np.ones(fixed_size, dtype=np.uint8) * 255
    show_img = mask_bgr.copy()
    blue_bottom = -1
    if max_cnt is not None:
        rect = cv2.minAreaRect(max_cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)

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
            roi_img = np.rot90(roi_img)

            # --- AB识别：横向扫线统计峰值 ---
            _, roi_bin2 = cv2.threshold(roi_img, 127, 255, cv2.THRESH_BINARY)
            kernel = np.ones((3, 3), np.uint8)
            roi_bin2 = cv2.morphologyEx(roi_bin2, cv2.MORPH_OPEN, kernel)
            roi_bin2 = cv2.morphologyEx(roi_bin2, cv2.MORPH_CLOSE, kernel)
            row_sum = np.sum(roi_bin2 == 255, axis=1)
            threshold = int(0.3 * np.max(row_sum)) if np.max(row_sum) > 0 else 0
            min_peak_height = int(0.2 * roi_bin2.shape[1])
            filtered_peaks = []
            in_peak = False
            for i, val in enumerate(row_sum):
                if val > threshold and not in_peak:
                    peak_start = i
                    in_peak = True
                elif val > threshold and in_peak:
                    pass
                elif val <= threshold and in_peak:
                    peak_end = i - 1
                    if peak_end - peak_start > 3 and np.max(row_sum[peak_start:peak_end+1]) > min_peak_height:
                        filtered_peaks.append([peak_start, peak_end])
                    in_peak = False
            if in_peak:
                peak_end = len(row_sum) - 1
                if peak_end - peak_start > 3 and np.max(row_sum[peak_start:peak_end+1]) > min_peak_height:
                    filtered_peaks.append([peak_start, peak_end])
            peak_count = len(filtered_peaks)

            contours2, hierarchy = cv2.findContours(roi_bin2, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            hole_count = 0
            min_hole_area = 100
            if hierarchy is not None:
                for i in range(len(hierarchy[0])):
                    if hierarchy[0][i][3] != -1:
                        area = cv2.contourArea(contours2[i])
                        if area > min_hole_area:
                            hole_count += 1

            print("峰值数:", peak_count, "空洞数:", hole_count)
            if (peak_count == 1 and hole_count == 1):
                print("识别为：A")
            elif (peak_count == 2 and hole_count == 2):
                print("识别为：B")
            else:
                print("0")

        # 输出蓝色矩形底行行数
        x, y, w_rect, h_rect = cv2.boundingRect(max_cnt)
        blue_bottom = y + h_rect
        print("蓝色矩形底行数:", blue_bottom)
    else:
        print("无有效蓝色区域或底行不达标，不做识别")

    cv2.imshow('Blue Mask', show_img)
    cv2.imshow('ROI', roi_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()