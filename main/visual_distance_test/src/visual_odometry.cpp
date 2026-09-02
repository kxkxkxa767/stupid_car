#include "xtnetrc_visual_distance/visual_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace xtnetrc::visual_distance {
namespace {

constexpr double kPixelsPerMetre = 400.0;
constexpr double kNearXMetres = 0.50;
constexpr double kFarXMetres = 1.25;
constexpr double kHalfWidthMetres = 0.32;

cv::Point2f ground_to_image(const cv::Matx33d& inverse,
                            const double x, const double y) {
    const cv::Vec3d point = inverse * cv::Vec3d(x, y, 1.0);
    return {static_cast<float>(point[0] / point[2]),
            static_cast<float>(point[1] / point[2])};
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    const double upper = *middle;
    if (values.size() % 2 != 0) return upper;
    return 0.5 * (upper + *std::max_element(values.begin(), middle));
}

cv::Size bird_size() {
    return {cvRound(2.0 * kHalfWidthMetres * kPixelsPerMetre),
            cvRound((kFarXMetres - kNearXMetres) * kPixelsPerMetre)};
}

cv::Matx33d image_to_bird(
    const vision::GroundProjectionConfig& config) {
    const cv::Matx33d ground_to_bird(
        0.0, kPixelsPerMetre, kHalfWidthMetres * kPixelsPerMetre,
        -kPixelsPerMetre, 0.0, kFarXMetres * kPixelsPerMetre,
        0.0, 0.0, 1.0);
    return ground_to_bird * config.image_to_vehicle_ground;
}

struct FlowSample {
    cv::Point2f previous;
    double flow_x{0.0};
    double flow_y{0.0};
};

}  // namespace

GroundVisualOdometry::GroundVisualOdometry(vision::GroundProjectionConfig config)
    : projector_(std::move(config)) {}

cv::Mat GroundVisualOdometry::make_ground_mask(const cv::Size size) const {
    cv::Mat mask(size, CV_8UC1, cv::Scalar(0));
    const cv::Matx33d inverse = projector_.config().image_to_vehicle_ground.inv();
    std::vector<cv::Point> polygon;
    for (const auto point : {
             ground_to_image(inverse, kNearXMetres, kHalfWidthMetres),
             ground_to_image(inverse, kFarXMetres, kHalfWidthMetres),
             ground_to_image(inverse, kFarXMetres, -kHalfWidthMetres),
             ground_to_image(inverse, kNearXMetres, -kHalfWidthMetres)}) {
        polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    }
    cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
    return mask;
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
    const cv::Size output_size = bird_size();
    const cv::Mat transform(image_to_bird(projector_.config()));
    cv::warpPerspective(previous_gray, previous_bird, transform, output_size,
                        cv::INTER_LINEAR);
    cv::warpPerspective(current_gray, current_bird, transform, output_size,
                        cv::INTER_LINEAR);

    cv::Mat feature_mask(output_size, CV_8UC1, cv::Scalar(0));
    cv::rectangle(feature_mask, cv::Rect(14, 20, output_size.width - 28,
                                        output_size.height - 40),
                  cv::Scalar(255), cv::FILLED);
    std::vector<cv::Point2f> previous_points;
    cv::goodFeaturesToTrack(previous_bird, previous_points, 60, 0.015, 9.0,
                            feature_mask, 5, false, 0.04);
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
        mean_ground_x += kFarXMetres - sample.previous.y / kPixelsPerMetre;
        mean_lateral_displacement += -sample.flow_x / kPixelsPerMetre;
    }
    mean_ground_x /= static_cast<double>(inliers.size());
    mean_lateral_displacement /= static_cast<double>(inliers.size());
    double numerator = 0.0;
    double denominator = 0.0;
    for (const auto& sample : inliers) {
        const double ground_x = kFarXMetres - sample.previous.y / kPixelsPerMetre;
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
