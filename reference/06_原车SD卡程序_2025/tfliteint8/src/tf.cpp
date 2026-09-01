#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/delegates/xnnpack/xnnpack_delegate.h" // XNNPACK头文件

using namespace std;
using namespace cv;

int LCDW = 160; // 模型输入宽度
int LCDH = 128; // 模型输入高度

int main() {
    string tflite_path = "/home/5G/5G/tfliteint8/src/tflite_BGR.tflite";
    int num_threads = 4;

    // 加载模型
    auto model = tflite::FlatBufferModel::BuildFromFile(tflite_path.c_str());
    tflite::ops::builtin::BuiltinOpResolver resolver;
    unique_ptr<tflite::Interpreter> interpreter;
    tflite::InterpreterBuilder(*model, resolver)(&interpreter, num_threads);

    // 添加XNNPACK Delegate加速
    TfLiteXNNPackDelegateOptions xnnpack_options = TfLiteXNNPackDelegateOptionsDefault();
    xnnpack_options.num_threads = num_threads;
    auto xnnpack_delegate = TfLiteXNNPackDelegateCreate(&xnnpack_options);
    interpreter->ModifyGraphWithDelegate(xnnpack_delegate);

    interpreter->AllocateTensors();

    // 获取输入输出信息
    int in_idx = interpreter->inputs()[0];
    int out_idx = interpreter->outputs()[0];
    TfLiteTensor* in_tensor = interpreter->tensor(in_idx);
    TfLiteTensor* out_tensor = interpreter->tensor(out_idx);

    float in_scale = in_tensor->params.scale;
    int in_zero = in_tensor->params.zero_point;
    float out_scale = out_tensor->params.scale;
    int out_zero = out_tensor->params.zero_point;

    int H = in_tensor->dims->data[1];
    int W = in_tensor->dims->data[2];
    int C = in_tensor->dims->data[3];

    cout << "[Model] int8 input/output, shape=[1," << H << "," << W << "," << C << "], threads=" << num_threads << endl;
    cout << "[Quant] in_scale=" << in_scale << ", in_zero=" << in_zero << "; out_scale=" << out_scale << ", out_zero=" << out_zero << endl;

    auto quant_threshold_05 = [](float scale, int zero) {
        if (scale <= 0) return 0;
        int thr = static_cast<int>(round(0.5 / scale + zero));
        thr = max(-128, min(127, thr));
        return thr;
    };
    int OUT_THR = quant_threshold_05(out_scale, out_zero);

    // 打开视频
    VideoCapture cap("/home/5G/video/1.avi");
    //VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(CAP_PROP_FPS, 120);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

    // mask生成函数
    auto mask2d_from_tensor = [&](const int8_t* y_tensor, int out_h, int out_w, int out_c, Mat& mask) {
        mask.create(out_h, out_w, CV_8UC1);
        for (int i = 0; i < out_h; ++i) {
            for (int j = 0; j < out_w; ++j) {
                int idx = i * out_w * out_c + j * out_c;
                int val = 0;
                for (int k = 0; k < out_c; ++k) {
                    val = max(val, static_cast<int>(y_tensor[idx + k]));
                }
                mask.at<uchar>(i, j) = val >= OUT_THR ? 255 : 0;
            }
        }
    };

    vector<int8_t> x_buf(H * W * C);
    Mat Cropped_image, Compress_image, norm_img, mask2d, overlay;
    vector<vector<Point>> contours;

    while (true) {
        Mat Original_image;
        if (!cap.read(Original_image)) break;

        int64_t start_tick = getTickCount();

        Cropped_image = Original_image(Rect(0, Original_image.rows / 2, Original_image.cols, Original_image.rows / 2));

        Compress_image.create(LCDH, LCDW, Cropped_image.type());
        resize(Cropped_image, Compress_image, Size(LCDW, LCDH), 0, 0, INTER_LINEAR);

        // SIMD归一化
        Compress_image.convertTo(norm_img, CV_32FC3, 1.0 / 255.0);

        parallel_for_(Range(0, H * W * C), [&](const Range& range){
            const float* src = reinterpret_cast<const float*>(norm_img.ptr<float>(0));
            for (int i = range.start; i < range.end; ++i) {
                int quant = static_cast<int>(round(src[i] / in_scale + in_zero));
                x_buf[i] = static_cast<int8_t>(max(-128, min(127, quant)));
            }
        });

        // 设置输入
        memcpy(interpreter->typed_tensor<int8_t>(in_idx), x_buf.data(), x_buf.size());

        // 推理
        interpreter->Invoke();

        // 获取输出
        int out_h = out_tensor->dims->data[1];
        int out_w = out_tensor->dims->data[2];
        int out_c = out_tensor->dims->data[3];
        int8_t* y = interpreter->typed_tensor<int8_t>(out_idx);

        mask2d_from_tensor(y, out_h, out_w, out_c, mask2d);

        contours.clear();
        findContours(mask2d, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        overlay = Compress_image.clone();
        drawContours(overlay, contours, -1, Scalar(0, 255, 0), 3);

        int64_t end_tick = getTickCount();
        double elapsed_ms = (end_tick - start_tick) * 1000.0 / getTickFrequency();
        cout << "[Time] Frame processed in " << elapsed_ms << " ms" << endl;

        // 如需显示，可取消注释
        //imshow("Result", overlay);
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();

    // 释放XNNPACK Delegate
    TfLiteXNNPackDelegateDelete(xnnpack_delegate);

    return 0;
}