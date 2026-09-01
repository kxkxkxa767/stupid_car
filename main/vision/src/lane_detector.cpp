#include "lane_detector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace xtnetrc {

LaneDetector::LaneDetector(LaneConfig config) : config_(config) {}

LaneResult LaneDetector::process(const cv::Mat& frame) {
    LaneResult result;
    if (frame.empty()) {
        return result;
    }

    const int top = std::clamp(
        static_cast<int>(std::lround(frame.rows * config_.roi_top_ratio)),
        0,
        frame.rows - 1);
    const int bottom = std::clamp(
        static_cast<int>(std::lround(frame.rows * config_.roi_bottom_ratio)),
        top + 1,
        frame.rows);

    const cv::Rect roi_rect(0, top, frame.cols, bottom - top);
    const cv::Mat roi = frame(roi_rect);

    cv::Mat gray;
    cv::Mat blurred;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0.5, 0.5);
    cv::Canny(blurred, result.edges, config_.canny_low, config_.canny_high, 3);

    result.overlay = frame.clone();
    cv::rectangle(result.overlay, roi_rect, cv::Scalar(70, 70, 70), 1);

    const int rows_to_scan = std::min(config_.scan_rows, result.edges.rows);
    const int first_y = result.edges.rows - 1;
    const int center_x = result.edges.cols / 2;
    double midpoint_sum = 0.0;
    int valid_rows = 0;

    for (int index = 0; index < rows_to_scan; ++index) {
        const int y = first_y - index;
        int left_x = -1;
        int right_x = -1;

        for (int x = center_x; x >= 0; --x) {
            if (result.edges.at<unsigned char>(y, x) != 0) {
                left_x = x;
                break;
            }
        }
        for (int x = center_x; x < result.edges.cols; ++x) {
            if (result.edges.at<unsigned char>(y, x) != 0) {
                right_x = x;
                break;
            }
        }

        if (left_x < 0 || right_x < 0 || right_x <= left_x) {
            continue;
        }

        const int midpoint_x = (left_x + right_x) / 2;
        const int overlay_y = top + y;
        midpoint_sum += midpoint_x;
        ++valid_rows;
        result.centerline_px.emplace_back(static_cast<float>(midpoint_x),
                                          static_cast<float>(overlay_y));

        cv::circle(result.overlay, cv::Point(left_x, overlay_y), 2,
                   cv::Scalar(255, 80, 0), cv::FILLED);
        cv::circle(result.overlay, cv::Point(right_x, overlay_y), 2,
                   cv::Scalar(255, 80, 0), cv::FILLED);
        cv::circle(result.overlay, cv::Point(midpoint_x, overlay_y), 2,
                   cv::Scalar(0, 220, 255), cv::FILLED);
    }

    result.confidence = rows_to_scan > 0
        ? static_cast<double>(valid_rows) / rows_to_scan
        : 0.0;

    if (valid_rows == 0) {
        result.steering_deg = config_.steering_center_deg;
        return result;
    }

    result.detected = true;
    result.midpoint_x = midpoint_sum / valid_rows;
    result.error_px = result.midpoint_x - center_x;

    const double correction =
        config_.kp * result.error_px +
        config_.kd * (result.error_px - last_error_);
    last_error_ = result.error_px;

    result.steering_deg = std::clamp(
        config_.steering_center_deg - correction,
        config_.steering_min_deg,
        config_.steering_max_deg);

    cv::line(result.overlay,
             cv::Point(center_x, top),
             cv::Point(center_x, bottom - 1),
             cv::Scalar(0, 255, 0), 1);
    return result;
}

}  // namespace xtnetrc
