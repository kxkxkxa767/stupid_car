#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // 创建视频捕捉对象
    cv::VideoCapture cap(0);  // 摄像头编号

    // 设置视频编解码器、帧率和分辨率
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter out("demo.avi", fourcc, 10.0, cv::Size(640, 480));//10为帧率，可根据需要调整

    // 开始录制视频
    while (cap.isOpened()) {
        cv::Mat frame;
        bool ret = cap.read(frame);
        if (!ret) {
            break;
        }
        out.write(frame);
        cv::imshow("frame", frame);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 结束并释放资源
    cap.release();
    out.release();
    cv::destroyAllWindows();

    return 0;
}