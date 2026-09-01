import cv2
import numpy as np

camera_id = 0
width = 320
height = 240
fps = 60

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

cv2.namedWindow('Zebra Detect')

def nothing(x):
    pass

cv2.createTrackbar('Thresh', 'Zebra Detect', 180, 255, nothing)

def DetectZebraCrossing(frame):
    # 1. 图像预处理
    zebra_img = cv2.resize(frame, (800, 400))
    gray_zebra = cv2.cvtColor(zebra_img, cv2.COLOR_BGR2GRAY)
    blur_zebra = cv2.GaussianBlur(gray_zebra, (5, 5), 5)
    _, thresh_zebra = cv2.threshold(blur_zebra, 180, 255, cv2.THRESH_BINARY)
    kernel_ero = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1))
    img_ero = cv2.erode(thresh_zebra, kernel_ero, iterations=3)
    kernel_dia = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
    img_dia = cv2.dilate(img_ero, kernel_dia, iterations=1)

    # 2. 跳变计数逻辑
    NUM = 0
    for Ysite in range(20, 55):
        Xsite = 80
        while Xsite < 720:
            if img_dia[Ysite, Xsite] == 0 and img_dia[Ysite, Xsite + 1] == 255:
                Xstart = Xsite
                net = 0
                while Xstart + 1 < 800 and img_dia[Ysite, Xstart + 1] == 255:
                    if img_dia[Ysite, Xstart] == 255:
                        net += 1
                    Xstart += 1
                #print(f"net:{net}")
                if net >= 10:
                    NUM += 1
                    Xsite = Xstart
                else:
                    Xsite += 1
            else:
                Xsite += 1
    print(f"Zebra NUM: {NUM}")
    if NUM >= 70:
        print("find zebra")
        return True
    return False

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    # 只保留下半部分
    frame_half = frame[height//2:, :]

    # 放大图像
    zebra_img = cv2.resize(frame_half, (800, 400))

    # 灰度化
    gray_zebra = cv2.cvtColor(zebra_img, cv2.COLOR_BGR2GRAY)

    # 高斯模糊
    blur_zebra = cv2.GaussianBlur(gray_zebra, (5, 5), 5)

    # 二值化（阈值可调）
    thresh_val = cv2.getTrackbarPos('Thresh', 'Zebra Detect')
    _, thresh_zebra = cv2.threshold(blur_zebra, thresh_val, 255, cv2.THRESH_BINARY)

    # 腐蚀
    kernel_ero = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1))
    img_ero = cv2.erode(thresh_zebra, kernel_ero, iterations=3)

    # 膨胀
    kernel_dia = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
    img_dia = cv2.dilate(img_ero, kernel_dia, iterations=1)

    # 显示最后的二值图
    cv2.imshow('Zebra Detect', img_dia)

    # 斑马线检测（带日志输出）
    if DetectZebraCrossing(frame_half):
        print("检测到斑马线！")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()