import cv2
import numpy as np

camera_id = 0
width = 640
height = 480
fps = 120

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Yellow Mask')

def nothing(x):
    pass

# 创建6个滑动条
cv2.createTrackbar('H Low', 'Yellow Mask', 30, 179, nothing)
cv2.createTrackbar('H High', 'Yellow Mask', 55, 179, nothing)
cv2.createTrackbar('S Low', 'Yellow Mask', 100, 255, nothing)
cv2.createTrackbar('S High', 'Yellow Mask', 255, 255, nothing)
cv2.createTrackbar('V Low', 'Yellow Mask', 100, 255, nothing)
cv2.createTrackbar('V High', 'Yellow Mask', 255, 255, nothing)

def detect_yellow_area(frame, h_low, h_high, s_low, s_high, v_low, v_high):
    h = frame.shape[0]
    frame = frame[h//2:, :]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([h_low, s_low, v_low])
    upper_yellow = np.array([h_high, s_high, v_high])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    _, binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return binary, frame

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    h_low = cv2.getTrackbarPos('H Low', 'Yellow Mask')
    h_high = cv2.getTrackbarPos('H High', 'Yellow Mask')
    s_low = cv2.getTrackbarPos('S Low', 'Yellow Mask')
    s_high = cv2.getTrackbarPos('S High', 'Yellow Mask')
    v_low = cv2.getTrackbarPos('V Low', 'Yellow Mask')
    v_high = cv2.getTrackbarPos('V High', 'Yellow Mask')

    binary, cropped = detect_yellow_area(frame, h_low, h_high, s_low, s_high, v_low, v_high)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    areas = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 5:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(binary_color, (x, y), (x + w, y + h), (0, 255, 0), 2)
            areas.append(w * h)

    cv2.imshow('Yellow Mask', binary_color)
    print(f"检测到矩形面积: {areas}      ")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()