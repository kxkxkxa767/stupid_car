#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>

#define YunTai_X_MIN 43
#define YunTai_X_MAX 103
#define YunTai_Y_MIN 43
#define YunTai_Y_MAX 103

int YunTai_X_val = 73;
int YunTai_Y_val = 73;

// 滑动条回调
void on_trackbar_X(int pos, void* userdata) {
    if (pos < YunTai_X_MIN) pos = YunTai_X_MIN;
    if (pos > YunTai_X_MAX) pos = YunTai_X_MAX;
    YunTai_X_val = pos;
    gpioPWM(22, YunTai_X_val);
}

void on_trackbar_Y(int pos, void* userdata) {
    if (pos < YunTai_Y_MIN) pos = YunTai_Y_MIN;
    if (pos > YunTai_Y_MAX) pos = YunTai_Y_MAX;
    YunTai_Y_val = pos;
    gpioPWM(23, YunTai_Y_val);
}

int main() {
    // 初始化 pigpio
    if (gpioInitialise() < 0) {
        printf("pigpio 初始化失败\n");
        return -1;
    }
    gpioSetMode(22, PI_OUTPUT);
    gpioSetPWMfrequency(22, 50);
    gpioSetPWMrange(22, 1000);
    gpioPWM(22, YunTai_X_val);

    gpioSetMode(23, PI_OUTPUT);
    gpioSetPWMfrequency(23, 50);
    gpioSetPWMrange(23, 1000);
    gpioPWM(23, YunTai_Y_val);

    // 打开摄像头
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        printf("无法打开摄像头\n");
        gpioTerminate();
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
    cv::createTrackbar("YunTai_X", "Camera", &YunTai_X_val, YunTai_X_MAX, on_trackbar_X, NULL);
    cv::createTrackbar("YunTai_Y", "Camera", &YunTai_Y_val, YunTai_Y_MAX, on_trackbar_Y, NULL);
    cv::setTrackbarPos("YunTai_X", "Camera", YunTai_X_val);
    cv::setTrackbarPos("YunTai_Y", "Camera", YunTai_Y_val);

    cv::Mat frame;
    while (1) {
        cap >> frame;
        if (frame.empty()) {
            printf("无法读取摄像头画面\n");
            break;
        }
        cv::imshow("Camera", frame);

        // 实时读取滑动条并控制云台
        int x = cv::getTrackbarPos("YunTai_X", "Camera");
        int y = cv::getTrackbarPos("YunTai_Y", "Camera");
        if (x < YunTai_X_MIN) x = YunTai_X_MIN;
        if (y < YunTai_Y_MIN) y = YunTai_Y_MIN;
        gpioPWM(22, x);
        gpioPWM(23, y);

        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    gpioTerminate();
    return 0;
}