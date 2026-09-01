import cv2

camera_id = 0
width = 640
height = 480
fps = 30

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

win = 'LowerHalfThreshold'
cv2.namedWindow(win)

def nothing(x):
    pass

cv2.createTrackbar('Thresh', win, 128, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    h = frame.shape[0]
    lower = frame[h//2:, :]  # 下半部分
    gray = cv2.cvtColor(lower, cv2.COLOR_BGR2GRAY)

    t = cv2.getTrackbarPos('Thresh', win)
    _, binary = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)

    # 为了更直观：将结果放到同一窗口，上半部分原图，下半部分二值
    display = frame.copy()
    # 将二值图扩展到3通道以便贴回
    binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    display[h//2:, :] = binary_bgr

    cv2.putText(display, f"Thresh={t}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    cv2.imshow(win, display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()