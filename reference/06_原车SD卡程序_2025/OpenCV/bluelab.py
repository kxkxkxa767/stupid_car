# ...existing code...
import cv2
import numpy as np

camera_id = 0
width = 320
height = 240
fps = 120

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Lab Blue Mask')

def nothing(x):
    pass

# a、b分量滑动条初始值
cv2.createTrackbar('a Low', 'Lab Blue Mask', 130, 255, nothing)
cv2.createTrackbar('a High', 'Lab Blue Mask', 255, 255, nothing)
cv2.createTrackbar('b Low', 'Lab Blue Mask', 40, 255, nothing)
cv2.createTrackbar('b High', 'Lab Blue Mask', 100, 255, nothing)

def detect_lab_blue_area(frame, a_low, a_high, b_low, b_high):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    lower = np.array([0, a_low, b_low])
    upper = np.array([255, a_high, b_high])
    mask = cv2.inRange(lab, lower, upper)
    # 形态学操作
    kernel = np.ones((3, 3), np.uint8)
    #mask = cv2.erode(mask, kernel, iterations=2)
    #mask = cv2.dilate(mask, kernel, iterations=2)
    # 二值化
    _, binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return binary

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    # 只保留下半部分进行处理
    h = frame.shape[0]
    frame_bottom = frame[h//2 : h, :, :].copy()  # 注意 copy() 防止引用问题

    a_low = cv2.getTrackbarPos('a Low', 'Lab Blue Mask')
    a_high = cv2.getTrackbarPos('a High', 'Lab Blue Mask')
    b_low = cv2.getTrackbarPos('b Low', 'Lab Blue Mask')
    b_high = cv2.getTrackbarPos('b High', 'Lab Blue Mask')

    binary = detect_lab_blue_area(frame_bottom, a_low, a_high, b_low, b_high)

    # 查找轮廓，只画最大面积的一个框（坐标基于下半帧）
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    max_area = 0
    max_rect = None
    for cnt in contours:
        if cv2.contourArea(cnt) > 5:
            x, y, w, hbox = cv2.boundingRect(cnt)
            area = w * hbox
            if area > max_area:
                max_area = area
                max_rect = (x, y, w, hbox)

    if max_rect:
        x, y, w, hbox = max_rect
        cv2.rectangle(binary_color, (x, y), (x + w, y + hbox), (0, 255, 0), 2)

    # 为了便于调试，同时显示原图下半帧与标注
    combined = np.hstack((cv2.resize(frame_bottom, (binary_color.shape[1], binary_color.shape[0])), binary_color))
    cv2.imshow('Lab Blue Mask', combined)
    print(f"最大矩形面积: {max_area}      ")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()