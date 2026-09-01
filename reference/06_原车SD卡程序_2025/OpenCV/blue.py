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

cv2.namedWindow('Blue Mask')

def nothing(x):
    pass

cv2.createTrackbar('S Low', 'Blue Mask', 43, 255, nothing)
cv2.createTrackbar('S High', 'Blue Mask', 255, 255, nothing)

def detect_blue_area(frame, s_low, s_high):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, s_low, 10])
    upper_blue = np.array([140, s_high, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    _, binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return binary

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    s_low = cv2.getTrackbarPos('S Low', 'Blue Mask')
    s_high = cv2.getTrackbarPos('S High', 'Blue Mask')

    binary = detect_blue_area(frame, s_low, s_high)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    max_area = 0
    max_rect = None
    for cnt in contours:
        if cv2.contourArea(cnt) > 5:
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            if area > max_area:
                max_area = area
                max_rect = (x, y, w, h)

    if max_rect:
        x, y, w, h = max_rect
        cv2.rectangle(binary_color, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow('Blue Mask', binary_color)
    print(f"最大矩形面积: {max_area}      ")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()