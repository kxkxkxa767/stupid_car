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
cv2.createTrackbar('H High 1', 'Red Mask', 20, 180, nothing)
cv2.createTrackbar('S Low 1', 'Red Mask', 70, 255, nothing)
cv2.createTrackbar('V Low 1', 'Red Mask', 170, 255, nothing)
cv2.createTrackbar('V High 1', 'Red Mask', 255, 255, nothing)
# 高区间
cv2.createTrackbar('H Low 2', 'Red Mask', 160, 180, nothing)
cv2.createTrackbar('H High 2', 'Red Mask', 180, 180, nothing)
cv2.createTrackbar('S Low 2', 'Red Mask', 5, 255, nothing)
cv2.createTrackbar('V Low 2', 'Red Mask', 170, 255, nothing)
cv2.createTrackbar('V High 2', 'Red Mask', 255, 255, nothing)

fixed_size = (200, 200)

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
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
        box = np.intp(box)

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
            roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
            roi_img = cv2.bitwise_not(roi_img)
            roi_img = np.rot90(roi_img) 

            # --- AB识别：横向扫线统计峰值 ---
            # 二值化，确保只有0和255
            _, roi_bin2 = cv2.threshold(roi_img, 127, 255, cv2.THRESH_BINARY)
            kernel = np.ones((3, 3), np.uint8)
            roi_bin2 = cv2.morphologyEx(roi_bin2, cv2.MORPH_OPEN, kernel)
            roi_bin2 = cv2.morphologyEx(roi_bin2, cv2.MORPH_CLOSE, kernel)
            row_sum = np.sum(roi_bin2 == 255, axis=1)
            threshold = int(0.3 * np.max(row_sum)) if np.max(row_sum) > 0 else 0
            peaks = []
            in_peak = False
            min_peak_height = int(0.2 * roi_bin2.shape[1])
            filtered_peaks = []
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
            # 处理最后一个峰
            if in_peak:
                peak_end = len(row_sum) - 1
                if peak_end - peak_start > 3 and np.max(row_sum[peak_start:peak_end+1]) > min_peak_height:
                    filtered_peaks.append([peak_start, peak_end])
            peak_count = len(filtered_peaks)

            # 结合空洞数和面积辅助判断
            contours, hierarchy = cv2.findContours(roi_bin2, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            hole_count = 0
            min_hole_area = 100  # 空洞面积阈值，可根据实际调整
            if hierarchy is not None:
                for i in range(len(hierarchy[0])):
                    if hierarchy[0][i][3] != -1:
                        area = cv2.contourArea(contours[i])
                        #print("空洞面积:", area)
                        if area > min_hole_area:
                            hole_count += 1

            print("峰值数:", peak_count, "空洞数:", hole_count)
            # 综合判决
            if (peak_count == 1 and hole_count == 1):
                print("识别为：A")
            elif (peak_count == 2 and hole_count == 2):
                print("识别为：B")
            else:
                print("0")
            # 其它情况不输出

    cv2.imshow('Red Mask', show_img)
    cv2.imshow('ROI', roi_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()