#pragma once

#include <string>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace xtnetrc::vision {

struct GroundPoint {
    double x_forward_m{0.0};
    double y_left_m{0.0};
};

struct GroundBounds {
    double x_min{0.0}, x_max{0.0}, y_min{0.0}, y_max{0.0};
    void validate() const;
};

struct GroundProjectionConfig {
    int image_width{0};
    int image_height{0};
    cv::Matx33d camera_matrix{cv::Matx33d::eye()};
    std::vector<double> dist_coeffs;
    cv::Matx33d image_to_vehicle_ground{cv::Matx33d::eye()};
    std::string projection_model;
    std::string camera_role;
    std::string camera_model;
    std::string camera_device_by_id;
    std::optional<GroundBounds> calibrated_bounds;

    void validate() const;
};

// 读取 Python 标定工具生成的 ground_projection.json。
[[nodiscard]] GroundProjectionConfig load_ground_projection_json(
    const std::string& path);

class GroundProjector {
public:
    explicit GroundProjector(GroundProjectionConfig config);

    // 输入必须是标定分辨率的原图。raw_pixel_homography 不做畸变逆解。
    [[nodiscard]] std::vector<GroundPoint> project(
        const std::vector<cv::Point2f>& image_points,
        cv::Size image_size) const;

    [[nodiscard]] const GroundProjectionConfig& config() const noexcept {
        return config_;
    }

    // Native-pixel mask; invalid/horizon pixels are excluded, never resized.
    [[nodiscard]] cv::Mat mask_for_bounds(const GroundBounds& bounds) const;

private:
    GroundProjectionConfig config_;
};

}  // namespace xtnetrc::vision
