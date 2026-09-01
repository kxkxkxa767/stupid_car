import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import time

TFLITE_PATH = "/home/5G/5G/tflite/src/tflite_BGR.tflite"
VIDEO_PATH  = "/home/5G/video/1.avi"

interpreter = tflite.Interpreter(model_path=TFLITE_PATH, num_threads=4)
interpreter.allocate_tensors()

inp = interpreter.get_input_details()[0]
out = interpreter.get_output_details()[0]
in_idx   = inp["index"]
out_idx  = out["index"]
in_scale, in_zero = inp["quantization"]
out_scale, out_zero = out["quantization"]
_, H, W, C = inp["shape"]

print(f"[Model] input dtype={inp['dtype']}, output dtype={out['dtype']}, shape={inp['shape']}")
print(f"[Quant] in_scale={in_scale}, in_zero={in_zero}, out_scale={out_scale}, out_zero={out_zero}")

def get_color(label):
    colors = [
        (0,0,0), (0,0,255), (0,255,0), (255,0,0),
        (255,255,0), (0,255,255), (255,0,255), (128,128,128)
    ]
    return colors[label % len(colors)]

cap = cv2.VideoCapture(VIDEO_PATH)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    start = time.time()

    half_y = frame.shape[0] // 2
    cropped = frame[half_y:, :, :]
    resized = cv2.resize(cropped, (W, H), interpolation=cv2.INTER_NEAREST)

    # 修正：先归一化到[0,1]，再量化
    f_buf = resized.astype(np.float32) / 255.0
    f_buf = f_buf / in_scale + in_zero
    input_data = np.clip(np.round(f_buf), -128, 127).astype(np.int8)
    input_data = input_data.reshape((1, H, W, C))

    interpreter.set_tensor(in_idx, input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(out_idx)[0]  # shape: [H, W, classes]
    out_H, out_W, out_C = output_data.shape

    # 统计类别
    mask = np.zeros((out_H, out_W), dtype=np.uint8)
    class_count = [0] * out_C
    for y in range(out_H):
        for x in range(out_W):
            scores = output_data[y, x, :]
            max_idx = np.argmax(scores)
            mask[y, x] = max_idx
            class_count[max_idx] += 1

    mask_color = np.zeros((out_H, out_W, 3), dtype=np.uint8)
    for y in range(out_H):
        for x in range(out_W):
            mask_color[y, x] = get_color(mask[y, x])

    mask_resized = cv2.resize(mask_color, (resized.shape[1], resized.shape[0]), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Mask", mask_resized)
    if cv2.waitKey(1) == 27:
        break

    print(f"类别分布: {class_count}")
    print(f"部分原始输出: {output_data.flatten()[:20]}")
    print(f"耗时: {int((time.time()-start)*1000)} ms")

cap.release()
cv2.destroyAllWindows()