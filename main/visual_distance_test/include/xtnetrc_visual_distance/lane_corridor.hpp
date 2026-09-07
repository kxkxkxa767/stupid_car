#pragma once

#include "xtnetrc_vision/ground_projector.hpp"

#include <opencv2/core.hpp>

namespace xtnetrc::visual_distance {

struct LaneCorridorResult {
    bool valid{false};
    double reference_x_m{1.10};  // center_y_m and boundary y values are measured here.
    double left_y_m{0.0};
    double right_y_m{0.0};
    double center_y_m{0.0};
    double width_m{0.0};
    double heading_slope{0.0};
    double confidence{0.0};
    int left_segments{0};
    int right_segments{0};
    cv::Mat overlay;
};

// 在车辆坐标系内锁定夹住车辆中心的左右跑道实线，不追踪相邻跑道。
class LaneCorridorDetector {
public:
    explicit LaneCorridorDetector(vision::GroundProjectionConfig calibration);
    [[nodiscard]] LaneCorridorResult process(const cv::Mat& frame);
    void reset() noexcept;

private:
    vision::GroundProjector projector_;
    cv::Mat roi_;
    bool has_previous_{false};
    double previous_center_y_m_{0.0};
    double previous_width_m_{0.0};
};

}  // namespace xtnetrc::visual_distance
