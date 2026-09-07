#include "xtnetrc_visual_distance/lane_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace xtnetrc::visual_distance {
namespace {

struct Candidate {
    double lateral_at_reference_m{0.0};
    double slope{0.0};
    double weight{0.0};
    cv::Vec4i image_segment{};
};

double weighted_median(std::vector<Candidate> candidates,
                       const bool use_slope = false) {
    std::sort(candidates.begin(), candidates.end(), [&](const Candidate& a,
                                                         const Candidate& b) {
        return (use_slope ? a.slope : a.lateral_at_reference_m) <
               (use_slope ? b.slope : b.lateral_at_reference_m);
    });
    double total = 0.0;
    for (const auto& candidate : candidates) total += candidate.weight;
    double accumulated = 0.0;
    for (const auto& candidate : candidates) {
        accumulated += candidate.weight;
        if (accumulated >= total * 0.5) {
            return use_slope ? candidate.slope : candidate.lateral_at_reference_m;
        }
    }
    return 0.0;
}

double total_weight(const std::vector<Candidate>& candidates) {
    double total = 0.0;
    for (const auto& candidate : candidates) total += candidate.weight;
    return total;
}

}  // namespace

LaneCorridorDetector::LaneCorridorDetector(
    vision::GroundProjectionConfig calibration)
    : projector_(std::move(calibration)) {
    // Retain the existing metric search range; calibration extent is NOT a
    // hard lane crop. Any part outside the control rectangle is extrapolation.
    roi_ = projector_.mask_for_bounds({0.55, 2.20, -0.75, 0.75});
}

void LaneCorridorDetector::reset() noexcept {
    has_previous_ = false;
    previous_center_y_m_ = 0.0;
    previous_width_m_ = 0.0;
}

LaneCorridorResult LaneCorridorDetector::process(const cv::Mat& frame) {
    LaneCorridorResult result;
    if (frame.empty() || frame.cols != projector_.config().image_width ||
        frame.rows != projector_.config().image_height) {
        return result;
    }
    result.overlay = frame.clone();

    cv::Mat gray;
    cv::Mat enhanced;
    cv::Mat edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, enhanced);
    // CLAHE 配合较宽的 Canny 迟滞阈值，兼顾夜间参考和白天正式画面。
    cv::Canny(enhanced, edges, 15.0, 45.0, 3);

    cv::bitwise_and(edges, roi_, edges);

    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(edges, segments, 1.0, CV_PI / 360.0, 14, 16.0, 10.0);
    std::vector<Candidate> left;
    std::vector<Candidate> right;
    constexpr double reference_x_m = 1.10;

    for (const auto& segment : segments) {
        const double image_dx = static_cast<double>(segment[2] - segment[0]);
        const double image_dy = static_cast<double>(segment[3] - segment[1]);
        // 跑道边界在当前固定相机画面中应有明显的纵向延伸；排除路砖纹理和阴影产生的横线。
        if (std::abs(image_dy) < 12.0 ||
            std::abs(image_dy) < 0.45 * std::abs(image_dx)) {
            continue;
        }
        const std::vector<cv::Point2f> image_points{
            {static_cast<float>(segment[0]), static_cast<float>(segment[1])},
            {static_cast<float>(segment[2]), static_cast<float>(segment[3])},
        };
        const auto ground = projector_.project(image_points, frame.size());
        const double dx = ground[1].x_forward_m - ground[0].x_forward_m;
        const double dy = ground[1].y_left_m - ground[0].y_left_m;
        if (std::abs(dx) < 0.12) continue;
        const double slope = dy / dx;
        if (std::abs(slope) > 0.45) continue;
        const double minimum_x =
            std::min(ground[0].x_forward_m, ground[1].x_forward_m);
        const double maximum_x =
            std::max(ground[0].x_forward_m, ground[1].x_forward_m);
        if (minimum_x < 0.55 || maximum_x > 2.20) continue;
        const double lateral = ground[0].y_left_m +
            (reference_x_m - ground[0].x_forward_m) * slope;
        const double pixel_length = std::hypot(segment[2] - segment[0],
                                               segment[3] - segment[1]);
        Candidate candidate{lateral, slope, pixel_length, segment};
        const double image_midpoint_x = 0.5 * (segment[0] + segment[2]);
        if (lateral >= 0.15 && lateral <= 0.75 &&
            image_midpoint_x < frame.cols * 0.58) {
            left.push_back(candidate);
        } else if (lateral <= -0.12 && lateral >= -0.75 &&
                   image_midpoint_x > frame.cols * 0.42) {
            right.push_back(candidate);
        }
    }

    result.left_segments = static_cast<int>(left.size());
    result.right_segments = static_cast<int>(right.size());
    // 调试画面始终显示候选线，即使随后因数量或几何条件不足而拒绝该帧。
    for (const auto& candidate : left) {
        const auto& line = candidate.image_segment;
        cv::line(result.overlay, {line[0], line[1]}, {line[2], line[3]},
                 cv::Scalar(0, 220, 0), 1);
    }
    for (const auto& candidate : right) {
        const auto& line = candidate.image_segment;
        cv::line(result.overlay, {line[0], line[1]}, {line[2], line[3]},
                 cv::Scalar(220, 120, 0), 1);
    }
    if (left.size() < 2 || right.size() < 2 ||
        total_weight(left) < 55.0 || total_weight(right) < 55.0) {
        return result;
    }

    result.left_y_m = weighted_median(left);
    result.right_y_m = weighted_median(right);
    result.center_y_m = (result.left_y_m + result.right_y_m) * 0.5;
    result.width_m = result.left_y_m - result.right_y_m;
    result.heading_slope =
        (weighted_median(left, true) + weighted_median(right, true)) * 0.5;
    if (result.width_m < 0.45 || result.width_m > 1.10 ||
        std::abs(result.center_y_m) > 0.25 ||
        std::abs(result.heading_slope) > 0.35) {
        return result;
    }
    if (has_previous_ &&
        (std::abs(result.center_y_m - previous_center_y_m_) > 0.15 ||
         std::abs(result.width_m - previous_width_m_) > 0.25)) {
        return result;
    }

    result.confidence = std::clamp(
        std::min(total_weight(left), total_weight(right)) / 250.0, 0.0, 1.0);
    result.valid = result.confidence >= 0.25;
    if (!result.valid) return result;

    has_previous_ = true;
    previous_center_y_m_ = result.center_y_m;
    previous_width_m_ = result.width_m;
    return result;
}

}  // namespace xtnetrc::visual_distance
