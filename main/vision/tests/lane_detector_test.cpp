#include <cmath>
#include <iostream>

#include <opencv2/imgproc.hpp>

#include "lane_detector.hpp"

namespace {

cv::Mat make_lane_frame(int shift_x) {
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::line(frame, cv::Point(200 + shift_x, 250),
             cv::Point(200 + shift_x, 470), cv::Scalar(255, 255, 255), 8);
    cv::line(frame, cv::Point(440 + shift_x, 250),
             cv::Point(440 + shift_x, 470), cv::Scalar(255, 255, 255), 8);
    return frame;
}

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

}  // namespace

int main() {
    bool ok = true;

    xtnetrc::LaneDetector centered_detector;
    const xtnetrc::LaneResult centered =
        centered_detector.process(make_lane_frame(0));
    ok &= check(centered.detected, "centered lane should be detected");
    ok &= check(centered.confidence > 0.90,
                "centered lane confidence should be high");
    ok &= check(std::abs(centered.error_px) < 5.0,
                "centered lane error should be close to zero");

    xtnetrc::LaneDetector shifted_detector;
    const xtnetrc::LaneResult shifted =
        shifted_detector.process(make_lane_frame(50));
    ok &= check(shifted.detected, "shifted lane should be detected");
    ok &= check(shifted.error_px > 40.0,
                "right-shifted lane should have a positive error");
    ok &= check(shifted.steering_deg < 85.0,
                "right-shifted lane should request a left correction");

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: lane detector synthetic checks\n";
    return 0;
}
