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
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Lab Yellow Mask')

def nothing(x):
    pass

# a、b分量滑动条初始值（黄色一般a偏高，b很高）
cv2.createTrackbar('a Low', 'Lab Yellow Mask', 100, 255, nothing)
cv2.createTrackbar('a High', 'Lab Yellow Mask', 180, 255, nothing)
cv2.createTrackbar('b Low', 'Lab Yellow Mask', 155, 255, nothing)
cv2.createTrackbar('b High', 'Lab Yellow Mask', 180, 255, nothing)

def detect_lab_yellow_area(frame, a_low, a_high, b_low, b_high):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    lower = np.array([0, a_low, b_low])
    upper = np.array([255, a_high, b_high])
    mask = cv2.inRange(lab, lower, upper)
    # 形态学操作
    #kernel = np.ones((3, 3), np.uint8)
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

    a_low = cv2.getTrackbarPos('a Low', 'Lab Yellow Mask')
    a_high = cv2.getTrackbarPos('a High', 'Lab Yellow Mask')
    b_low = cv2.getTrackbarPos('b Low', 'Lab Yellow Mask')
    b_high = cv2.getTrackbarPos('b High', 'Lab Yellow Mask')

    binary = detect_lab_yellow_area(frame, a_low, a_high, b_low, b_high)

    # 查找轮廓
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    areas = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 5:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(binary_color, (x, y), (x + w, y + h), (0, 255, 0), 2)
            areas.append(w * h)

    cv2.imshow('Lab Yellow Mask', binary_color)
    print(f"检测到矩形面积: {areas}      ")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()