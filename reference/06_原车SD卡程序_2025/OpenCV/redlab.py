import cv2
import numpy as np

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture("/home/5G/video/1.1.mp4")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 120)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Lab Red Mask')

def nothing(x):
    pass

# a、b分量滑动条初始值（红色一般a高，b中等）
cv2.createTrackbar('a Low', 'Lab Red Mask', 145, 255, nothing)
cv2.createTrackbar('a High', 'Lab Red Mask', 180, 255, nothing)
cv2.createTrackbar('b Low', 'Lab Red Mask', 120, 255, nothing)
cv2.createTrackbar('b High', 'Lab Red Mask', 180, 255, nothing)

def detect_lab_red_area(frame, a_low, a_high, b_low, b_high):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    lower = np.array([0, a_low, b_low])
    upper = np.array([255, a_high, b_high])
    mask = cv2.inRange(lab, lower, upper)
    red_area = cv2.countNonZero(mask)
    return mask, red_area

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break
        
    #frame = cv2.flip(frame, 0)  # 上下
    #frame = cv2.flip(frame, 1)  # 左右    

    a_low = cv2.getTrackbarPos('a Low', 'Lab Red Mask')
    a_high = cv2.getTrackbarPos('a High', 'Lab Red Mask')
    b_low = cv2.getTrackbarPos('b Low', 'Lab Red Mask')
    b_high = cv2.getTrackbarPos('b High', 'Lab Red Mask')

    mask, red_area = detect_lab_red_area(frame, a_low, a_high, b_low, b_high)

    cv2.imshow('Lab Red Mask', mask)
    print(f"Lab红色区域像素数: {red_area}", end='\r')

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()