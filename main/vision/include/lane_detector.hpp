#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace xtnetrc {

struct LaneConfig {
    double roi_top_ratio = 0.55;
    double roi_bottom_ratio = 0.95;
    int scan_rows = 100;
    double canny_low = 40.0;
    double canny_high = 80.0;
    double kp = 0.15;
    double kd = 0.10;
    double steering_center_deg = 85.0;
    double steering_min_deg = 60.0;
    double steering_max_deg = 120.0;
};

struct LaneResult {
    bool detected = false;
    double midpoint_x = 0.0;
    double error_px = 0.0;
    double steering_deg = 85.0;
    double confidence = 0.0;
    // 按车辆近处到远处排列的车道中心像素点，供地面投影模块使用。
    std::vector<cv::Point2f> centerline_px;
    cv::Mat edges;
    cv::Mat overlay;
};

class LaneDetector {
public:
    explicit LaneDetector(LaneConfig config = {});
    LaneResult process(const cv::Mat& frame);

private:
    LaneConfig config_;
    double last_error_ = 0.0;
};

}  // namespace xtnetrc
