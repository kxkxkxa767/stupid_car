import cv2
import numpy as np

camera_id = 0
width = 320
height = 240
fps = 120

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Red Mask')

def nothing(x):
    pass

# 创建所有HSV上下限滑动条
cv2.createTrackbar('H Low 1', 'Red Mask', 0, 180, nothing)
cv2.createTrackbar('H High 1', 'Red Mask', 20, 180, nothing)
cv2.createTrackbar('S Low 1', 'Red Mask', 90, 255, nothing)
cv2.createTrackbar('S High 1', 'Red Mask', 255, 255, nothing)
cv2.createTrackbar('V Low 1', 'Red Mask', 170, 255, nothing)
cv2.createTrackbar('V High 1', 'Red Mask', 255, 255, nothing)

cv2.createTrackbar('H Low 2', 'Red Mask', 160, 180, nothing)
cv2.createTrackbar('H High 2', 'Red Mask', 180, 180, nothing)
cv2.createTrackbar('S Low 2', 'Red Mask', 15, 255, nothing)
cv2.createTrackbar('S High 2', 'Red Mask', 255, 255, nothing)
cv2.createTrackbar('V Low 2', 'Red Mask', 150, 255, nothing)
cv2.createTrackbar('V High 2', 'Red Mask', 255, 255, nothing)

def detect_red_area(frame, hsv1, hsv2):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, hsv1[:3], hsv1[3:])
    mask2 = cv2.inRange(hsv, hsv2[:3], hsv2[3:])
    mask = cv2.bitwise_or(mask1, mask2)
    red_area = cv2.countNonZero(mask)
    return mask, red_area

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    # 获取所有滑动条参数
    h_low1 = cv2.getTrackbarPos('H Low 1', 'Red Mask')
    h_high1 = cv2.getTrackbarPos('H High 1', 'Red Mask')
    s_low1 = cv2.getTrackbarPos('S Low 1', 'Red Mask')
    s_high1 = cv2.getTrackbarPos('S High 1', 'Red Mask')
    v_low1 = cv2.getTrackbarPos('V Low 1', 'Red Mask')
    v_high1 = cv2.getTrackbarPos('V High 1', 'Red Mask')

    h_low2 = cv2.getTrackbarPos('H Low 2', 'Red Mask')
    h_high2 = cv2.getTrackbarPos('H High 2', 'Red Mask')
    s_low2 = cv2.getTrackbarPos('S Low 2', 'Red Mask')
    s_high2 = cv2.getTrackbarPos('S High 2', 'Red Mask')
    v_low2 = cv2.getTrackbarPos('V Low 2', 'Red Mask')
    v_high2 = cv2.getTrackbarPos('V High 2', 'Red Mask')

    hsv1 = np.array([h_low1, s_low1, v_low1])
    hsv1_high = np.array([h_high1, s_high1, v_high1])
    hsv2 = np.array([h_low2, s_low2, v_low2])
    hsv2_high = np.array([h_high2, s_high2, v_high2])

    mask1 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), hsv1, hsv1_high)
    mask2 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), hsv2, hsv2_high)
    mask = cv2.bitwise_or(mask1, mask2)
    red_area = cv2.countNonZero(mask)

    cv2.imshow('Red Mask', mask)
    print(f"红色区域像素数: {red_area}", end='\r')

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()