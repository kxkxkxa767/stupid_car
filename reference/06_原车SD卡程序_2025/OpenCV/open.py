import cv2

# 指定摄像头编号和参数
camera_id = 0      # 摄像头编号
width = 640        # 分辨率宽
height = 480       # 分辨率高
fps = 120          # 帧率

# 打开摄像头
cap = cv2.VideoCapture(camera_id)

# 设置MJPG格式
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))

# 设置分辨率和帧率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

print(f"已打开摄像头 {camera_id}，分辨率: {width}x{height}，帧率: {fps}，格式: MJPG")

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    #frame = cv2.flip(frame, 0)  # 上下
    #frame = cv2.flip(frame, 1)  # 左右

    cv2.imshow('Camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()