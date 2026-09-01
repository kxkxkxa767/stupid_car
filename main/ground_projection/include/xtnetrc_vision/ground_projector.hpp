#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace xtnetrc::vision {

struct GroundPoint {
    double x_forward_m{0.0};
    double y_left_m{0.0};
};

struct GroundProjectionConfig {
    int image_width{0};
    int image_height{0};
    cv::Matx33d camera_matrix{cv::Matx33d::eye()};
    std::vector<double> dist_coeffs;
    cv::Matx33d image_to_vehicle_ground{cv::Matx33d::eye()};

    void validate() const;
};

// 读取 Python 标定工具生成的 ground_projection.json。
[[nodiscard]] GroundProjectionConfig load_ground_projection_json(
    const std::string& path);

class GroundProjector {
public:
    explicit GroundProjector(GroundProjectionConfig config);

    // 输入必须来自与标定相同分辨率的原始画面。函数先去畸变，再做地面单应变换。
    [[nodiscard]] std::vector<GroundPoint> project(
        const std::vector<cv::Point2f>& image_points,
        cv::Size image_size) const;

    [[nodiscard]] const GroundProjectionConfig& config() const noexcept {
        return config_;
    }

private:
    GroundProjectionConfig config_;
};

}  // namespace xtnetrc::vision
