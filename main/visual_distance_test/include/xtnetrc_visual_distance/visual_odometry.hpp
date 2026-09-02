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
    [[nodiscard]] cv::Mat make_ground_mask(cv::Size size) const;

    vision::GroundProjector projector_;
};

}  // namespace xtnetrc::visual_distance
