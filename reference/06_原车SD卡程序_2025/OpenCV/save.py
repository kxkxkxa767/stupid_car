import cv2

# 摄像头参数
camera_id = 0
width = 640
height = 480
fps = 120

# 打开摄像头
cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 视频保存设置（保存为640x480）
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('new4.avi', fourcc, fps, (width, height))

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break

    #frame = cv2.flip(frame, 0)  # 上下
    #frame = cv2.flip(frame, 1)  # 左右

    # 保存原始画面
    out.write(frame)

    # 显示原始画面
    #cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()