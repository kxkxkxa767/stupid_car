#pragma once

#include <opencv2/core.hpp>

#include "xtnetrc_vision/ground_projector.hpp"

namespace xtnetrc::visual_distance {

struct OdometryEstimate {
    bool valid{false};
    double forward_m{0.0};
    double lateral_m{0.0};
    double yaw_rad{0.0};
    double dispersion_m{0.0};
    int tracked_points{0};
};

class GroundVisualOdometry {
public:
    explicit GroundVisualOdometry(vision::GroundProjectionConfig config);

    [[nodiscard]] OdometryEstimate estimate(const cv::Mat& previous_bgr,
                                            const cv::Mat& current_bgr) const;

private:
    vision::GroundProjector projector_;
    double near_x_m_{0.50}, far_x_m_{1.25}, half_width_m_{0.32};
    cv::Size bird_size_;
    cv::Matx33d image_to_bird_;
    cv::Mat feature_mask_;
};

}  // namespace xtnetrc::visual_distance
