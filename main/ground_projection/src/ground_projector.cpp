#include "xtnetrc_vision/ground_projector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace xtnetrc::vision {
namespace {

[[nodiscard]] cv::Matx33d read_matrix3x3(const cv::FileNode& node,
                                         const std::string& name) {
    if (!node.isSeq() || node.size() != 3) {
        throw std::invalid_argument(name + " must contain three rows");
    }
    cv::Matx33d matrix;
    int row = 0;
    for (const auto& row_node : node) {
        if (!row_node.isSeq() || row_node.size() != 3) {
            throw std::invalid_argument(name + " rows must contain three values");
        }
        int column = 0;
        for (const auto& value : row_node) {
            matrix(row, column++) = static_cast<double>(value);
        }
        ++row;
    }
    return matrix;
}

[[nodiscard]] bool finite_matrix(const cv::Matx33d& matrix) noexcept {
    for (const double value : matrix.val) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] cv::Point2d undistort_point(
    const cv::Point2f point, const cv::Matx33d& camera,
    const std::vector<double>& distortion) {
    const cv::Vec3d distorted = camera.inv() * cv::Vec3d(point.x, point.y, 1.0);
    const double xd = distorted[0] / distorted[2];
    const double yd = distorted[1] / distorted[2];
    double x = xd;
    double y = yd;
    const auto coefficient = [&](const std::size_t index) {
        return index < distortion.size() ? distortion[index] : 0.0;
    };
    bool converged = true;
    for (int iteration = 0; iteration < 10; ++iteration) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial_numerator =
            1.0 + coefficient(0) * r2 + coefficient(1) * r4 +
            coefficient(4) * r6;
        const double radial_denominator =
            1.0 + coefficient(5) * r2 + coefficient(6) * r4 +
            coefficient(7) * r6;
        if (std::abs(radial_numerator) < 1e-12 ||
            std::abs(radial_denominator) < 1e-12) {
            throw std::runtime_error("distortion model became singular");
        }
        const double delta_x =
            2.0 * coefficient(2) * x * y +
            coefficient(3) * (r2 + 2.0 * x * x);
        const double delta_y =
            coefficient(2) * (r2 + 2.0 * y * y) +
            2.0 * coefficient(3) * x * y;
        const double inverse_radial = radial_denominator / radial_numerator;
        x = (xd - delta_x) * inverse_radial;
        y = (yd - delta_y) * inverse_radial;
        if (!std::isfinite(x) || !std::isfinite(y) ||
            std::abs(x) > 2.0 || std::abs(y) > 2.0) {
            converged = false;
            break;
        }
    }
    if (converged) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial =
            (1.0 + coefficient(0) * r2 + coefficient(1) * r4 +
             coefficient(4) * r6) /
            (1.0 + coefficient(5) * r2 + coefficient(6) * r4 +
             coefficient(7) * r6);
        const double estimated_x = x * radial + 2.0 * coefficient(2) * x * y +
            coefficient(3) * (r2 + 2.0 * x * x);
        const double estimated_y = y * radial + coefficient(2) * (r2 + 2.0 * y * y) +
            2.0 * coefficient(3) * x * y;
        converged = std::hypot(estimated_x - xd, estimated_y - yd) < 1e-5;
    }
    if (!converged) {
        // 极端广角边缘可能没有稳定的逆解；禁止数值发散，和 OpenCV 一样回退原像素。
        return {point.x, point.y};
    }
    const cv::Vec3d pixel = camera * cv::Vec3d(x, y, 1.0);
    return {pixel[0] / pixel[2], pixel[1] / pixel[2]};
}

}  // namespace

void GroundProjectionConfig::validate() const {
    if (image_width <= 0 || image_height <= 0) {
        throw std::invalid_argument("calibration image size must be positive");
    }
    if (!finite_matrix(camera_matrix) || !finite_matrix(image_to_vehicle_ground)) {
        throw std::invalid_argument("calibration matrices must be finite");
    }
    if (std::abs(cv::determinant(cv::Mat(camera_matrix))) < 1e-12 ||
        std::abs(cv::determinant(cv::Mat(image_to_vehicle_ground))) < 1e-12) {
        throw std::invalid_argument("calibration matrices must be invertible");
    }
    if (dist_coeffs.size() != 4 && dist_coeffs.size() != 5 &&
        dist_coeffs.size() != 8) {
        throw std::invalid_argument("distortion vector must contain 4, 5 or 8 values");
    }
    for (const double value : dist_coeffs) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("distortion values must be finite");
        }
    }
}

GroundProjectionConfig load_ground_projection_json(const std::string& path) {
    cv::FileStorage file(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    if (!file.isOpened()) {
        throw std::runtime_error("cannot open ground projection configuration: " + path);
    }
    GroundProjectionConfig config;
    config.image_width = static_cast<int>(file["image_width"]);
    config.image_height = static_cast<int>(file["image_height"]);
    config.camera_matrix = read_matrix3x3(file["camera_matrix"], "camera_matrix");
    config.image_to_vehicle_ground = read_matrix3x3(
        file["image_to_vehicle_ground_homography"],
        "image_to_vehicle_ground_homography");
    cv::FileNode distortion = file["projection_dist_coeffs"];
    if (distortion.empty()) {
        distortion = file["dist_coeffs"];
    }
    if (!distortion.isSeq()) {
        throw std::invalid_argument("dist_coeffs must be an array");
    }
    for (const auto& value : distortion) {
        config.dist_coeffs.push_back(static_cast<double>(value));
    }
    config.validate();
    return config;
}

GroundProjector::GroundProjector(GroundProjectionConfig config)
    : config_(std::move(config)) {
    config_.validate();
}

std::vector<GroundPoint> GroundProjector::project(
    const std::vector<cv::Point2f>& image_points, const cv::Size image_size) const {
    if (image_size.width != config_.image_width ||
        image_size.height != config_.image_height) {
        throw std::invalid_argument("frame resolution differs from calibration resolution");
    }
    for (const auto& point : image_points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            throw std::invalid_argument("image point must be finite");
        }
    }
    if (image_points.empty()) {
        return {};
    }

    std::vector<GroundPoint> result;
    result.reserve(image_points.size());
    for (const auto& point : image_points) {
        const cv::Point2d undistorted =
            undistort_point(point, config_.camera_matrix, config_.dist_coeffs);
        const cv::Vec3d homogeneous = config_.image_to_vehicle_ground *
            cv::Vec3d(undistorted.x, undistorted.y, 1.0);
        if (std::abs(homogeneous[2]) < 1e-12) {
            throw std::runtime_error("ground projection reached the horizon");
        }
        const double x = homogeneous[0] / homogeneous[2];
        const double y = homogeneous[1] / homogeneous[2];
        if (!std::isfinite(x) || !std::isfinite(y)) {
            throw std::runtime_error("ground projection produced a non-finite point");
        }
        result.push_back({x, y});
    }
    return result;
}

}  // namespace xtnetrc::vision
