#include "xtnetrc_visual_distance/visual_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace xtnetrc::visual_distance {
namespace {

constexpr double kPixelsPerMetre = 400.0;

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    const double upper = *middle;
    if (values.size() % 2 != 0) return upper;
    return 0.5 * (upper + *std::max_element(values.begin(), middle));
}


struct FlowSample {
    cv::Point2f previous;
    double flow_x{0.0};
    double flow_y{0.0};
};

}  // namespace

GroundVisualOdometry::GroundVisualOdometry(vision::GroundProjectionConfig config)
    : projector_(std::move(config)) {
    if (const auto& bounds = projector_.config().calibrated_bounds) {
        near_x_m_ = bounds->x_min; far_x_m_ = bounds->x_max;
        half_width_m_ = std::min(-bounds->y_min, bounds->y_max);
    }
    bird_size_ = {cvRound(2 * half_width_m_ * kPixelsPerMetre),
                  cvRound((far_x_m_ - near_x_m_) * kPixelsPerMetre)};
    if (bird_size_.width < 64 || bird_size_.height < 64 ||
        bird_size_.width > 4096 || bird_size_.height > 4096)
        throw std::invalid_argument("ground odometry region too small or large");
    const cv::Matx33d ground_to_bird(
        0, kPixelsPerMetre, half_width_m_ * kPixelsPerMetre,
        -kPixelsPerMetre, 0, far_x_m_ * kPixelsPerMetre, 0, 0, 1);
    image_to_bird_ = ground_to_bird * projector_.config().image_to_vehicle_ground;
    // Black padding outside the native frame is not usable ground texture.
    cv::Mat coverage(projector_.config().image_height, projector_.config().image_width,
                     CV_8UC1, cv::Scalar(255));
    cv::warpPerspective(coverage, feature_mask_, cv::Mat(image_to_bird_), bird_size_, cv::INTER_NEAREST);
    cv::erode(feature_mask_, feature_mask_, cv::Mat::ones(23,23,CV_8UC1),
              {-1,-1}, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::Mat interior(bird_size_, CV_8UC1, cv::Scalar(0));
    cv::rectangle(interior, cv::Rect(14,20,bird_size_.width-28,bird_size_.height-40),cv::Scalar(255),cv::FILLED);
    cv::bitwise_and(feature_mask_,interior,feature_mask_);
}

OdometryEstimate GroundVisualOdometry::estimate(
    const cv::Mat& previous_bgr, const cv::Mat& current_bgr) const {
    OdometryEstimate result;
    if (previous_bgr.empty() || current_bgr.empty() ||
        previous_bgr.size() != current_bgr.size() ||
        previous_bgr.cols != projector_.config().image_width ||
        previous_bgr.rows != projector_.config().image_height) {
        return result;
    }

    cv::Mat previous_gray, current_gray;
    cv::cvtColor(previous_bgr, previous_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(current_bgr, current_gray, cv::COLOR_BGR2GRAY);
    cv::Mat previous_bird, current_bird;
    const cv::Size output_size = bird_size_;
    const cv::Mat transform(image_to_bird_);
    cv::warpPerspective(previous_gray, previous_bird, transform, output_size,
                        cv::INTER_LINEAR);
    cv::warpPerspective(current_gray, current_bird, transform, output_size,
                        cv::INTER_LINEAR);

    std::vector<cv::Point2f> previous_points;
    cv::goodFeaturesToTrack(previous_bird, previous_points, 60, 0.015, 9.0,
                            feature_mask_, 5, false, 0.04);
    if (previous_points.size() < 8) return result;

    std::vector<cv::Point2f> current_points;
    std::vector<unsigned char> forward_status;
    std::vector<float> forward_error;
    cv::calcOpticalFlowPyrLK(previous_bird, current_bird, previous_points,
                            current_points, forward_status, forward_error,
                            cv::Size(21, 21), 2,
                            cv::TermCriteria(cv::TermCriteria::COUNT |
                                             cv::TermCriteria::EPS, 15, 0.01));

    std::vector<FlowSample> samples;
    for (std::size_t i = 0; i < previous_points.size(); ++i) {
        if (!forward_status[i] || forward_error[i] > 20.0f) {
            continue;
        }
        const auto point = current_points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0 || point.y < 0 ||
            point.x >= feature_mask_.cols || point.y >= feature_mask_.rows ||
            !feature_mask_.at<unsigned char>(static_cast<int>(point.y),static_cast<int>(point.x))) continue;
        const double flow_x = current_points[i].x - previous_points[i].x;
        const double flow_y = current_points[i].y - previous_points[i].y;
        if (std::isfinite(flow_x) && std::isfinite(flow_y)) {
            samples.push_back({previous_points[i], flow_x, flow_y});
        }
    }
    if (samples.size() < 8) return result;

    std::vector<double> x_values;
    std::vector<double> y_values;
    for (const auto& sample : samples) {
        x_values.push_back(sample.flow_x);
        y_values.push_back(sample.flow_y);
    }
    const double initial_x = median(x_values);
    const double initial_y = median(y_values);
    std::vector<FlowSample> inliers;
    std::vector<double> residuals_pixels;
    for (const auto& sample : samples) {
        const double residual = std::hypot(sample.flow_x - initial_x,
                                           sample.flow_y - initial_y);
        if (residual <= 3.0) {
            inliers.push_back(sample);
            residuals_pixels.push_back(residual);
        }
    }
    if (inliers.size() < 8) return result;

    x_values.clear();
    y_values.clear();
    for (const auto& sample : inliers) {
        x_values.push_back(sample.flow_x);
        y_values.push_back(sample.flow_y);
    }
    result.forward_m = median(y_values) / kPixelsPerMetre;
    result.lateral_m = -median(x_values) / kPixelsPerMetre;
    result.dispersion_m = median(residuals_pixels) / kPixelsPerMetre;
    result.tracked_points = static_cast<int>(inliers.size());

    double mean_ground_x = 0.0;
    double mean_lateral_displacement = 0.0;
    for (const auto& sample : inliers) {
        mean_ground_x += far_x_m_ - sample.previous.y / kPixelsPerMetre;
        mean_lateral_displacement += -sample.flow_x / kPixelsPerMetre;
    }
    mean_ground_x /= static_cast<double>(inliers.size());
    mean_lateral_displacement /= static_cast<double>(inliers.size());
    double numerator = 0.0;
    double denominator = 0.0;
    for (const auto& sample : inliers) {
        const double ground_x = far_x_m_ - sample.previous.y / kPixelsPerMetre;
        const double centered_x = ground_x - mean_ground_x;
        const double lateral_displacement = -sample.flow_x / kPixelsPerMetre;
        numerator += centered_x *
                     (lateral_displacement - mean_lateral_displacement);
        denominator += centered_x * centered_x;
    }
    result.yaw_rad = denominator > 1e-8 ? numerator / denominator : 0.0;

    result.valid = std::isfinite(result.forward_m) &&
                   std::isfinite(result.lateral_m) &&
                   std::isfinite(result.yaw_rad) &&
                   result.forward_m >= -0.01 && result.forward_m <= 0.045 &&
                   std::abs(result.lateral_m) <= 0.035 &&
                   std::abs(result.yaw_rad) <= 0.10 &&
                   result.dispersion_m <= 0.008;
    return result;
}

}  // namespace xtnetrc::visual_distance
