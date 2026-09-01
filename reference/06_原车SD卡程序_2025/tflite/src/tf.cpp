#include <opencv2/opencv.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/delegates/xnnpack/xnnpack_delegate.h>
#include <iostream>
#include <memory>

using namespace std;
using namespace cv;

int LCDW = 160; // 模型输入宽度
int LCDH = 128; // 模型输入高度

int main() {
    // 加载 TFLite 模型
    unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile("/home/5G/5G/tflite/src/FastSCNN.tflite");
    if (!model) {
        cerr << "模型加载失败" << endl;
        return -1;
    }

    tflite::ops::builtin::BuiltinOpResolver resolver;
    unique_ptr<tflite::Interpreter> interpreter;
    int num_threads = 4;
    tflite::InterpreterBuilder(*model, resolver)(&interpreter, num_threads);
    if (!interpreter) {
        cerr << "解释器创建失败" << endl;
        return -1;
    }

    // XNNPACK Delegate加速
    TfLiteXNNPackDelegateOptions xnnpack_options = TfLiteXNNPackDelegateOptionsDefault();
    xnnpack_options.num_threads = num_threads;
    auto xnnpack_delegate = TfLiteXNNPackDelegateCreate(&xnnpack_options);
    interpreter->ModifyGraphWithDelegate(xnnpack_delegate);

    interpreter->AllocateTensors();

    // 获取输入输出信息
    int input_idx = interpreter->inputs()[0];
    TfLiteIntArray* dims = interpreter->tensor(input_idx)->dims;
    int input_height = dims->data[2];
    int input_width = dims->data[3];

    // 打开视频，设置帧率和MJPG编码
    VideoCapture cap("/home/5G/video/1.10.mp4");
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(CAP_PROP_FPS, 120);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

    if (!cap.isOpened()) {
        cerr << "视频打开失败" << endl;
        return -1;
    }

    Mat frame;
    while (true) {
        int64_t start_tick = getTickCount(); // 记录开始时间

        cap >> frame;
        if (frame.empty()) break;

        // 裁剪下半部分
        Rect roi_rect(0, frame.rows / 2, frame.cols, frame.rows - frame.rows / 2);
        Mat roi = frame(roi_rect);

        // resize到160x128
        Mat img;
        resize(roi, img, Size(LCDW, LCDH));

        // BGR转RGB
        cvtColor(img, img, COLOR_BGR2RGB);

        // HWC->CHW
        vector<Mat> rgb_channels(3);
        split(img, rgb_channels);
        float* input = interpreter->typed_tensor<float>(input_idx);
        for (int c = 0; c < 3; ++c) {
            for (int i = 0; i < 128; ++i) {
                for (int j = 0; j < 160; ++j) {
                    input[c * 128 * 160 + i * 160 + j] = rgb_channels[c].at<uchar>(i, j) / 255.0f;
                }
            }
        }

        // 推理
        interpreter->Invoke();

        // 获取输出
        int output_idx = interpreter->outputs()[0];
        float* output = interpreter->typed_tensor<float>(output_idx); // shape: [1, 2, 128, 160]

        // 取argmax
        Mat mask(128, 160, CV_8UC1);
        for (int i = 0; i < 128; ++i) {
            for (int j = 0; j < 160; ++j) {
                float v0 = output[0 * 128 * 160 + i * 160 + j];
                float v1 = output[1 * 128 * 160 + i * 160 + j];
                mask.at<uchar>(i, j) = (v1 > v0) ? 255 : 0;
            }
        }

        int64_t end_tick = getTickCount(); // 记录结束时间
        double elapsed_ms = (end_tick - start_tick) * 1000.0 / getTickFrequency();
        cout << "[Time] Frame processed in " << elapsed_ms << " ms" << endl;

        // 直接显示输出掩码
        //imshow("Segmentation Mask", mask);
        if (waitKey(1) == 27) break; // ESC退出
    }

    cap.release();
    destroyAllWindows();

    // 释放XNNPACK Delegate
    TfLiteXNNPackDelegateDelete(xnnpack_delegate);

    return 0;
}