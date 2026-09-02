#include "xtnetrc_visual_distance/visual_odometry.hpp"
#include "xtnetrc_vision/ground_projector.hpp"

#include <cmath>
#include <iostream>
#include <random>

#include <opencv2/imgproc.hpp>

namespace vd = xtnetrc::visual_distance;
namespace vision = xtnetrc::vision;

int main() {
    const auto config = vision::load_ground_projection_json(
        XTNETRC_GROUND_CONFIG_PATH);
    vd::GroundVisualOdometry odometry(config);

    cv::Mat first(480, 640, CV_8UC3, cv::Scalar(25, 25, 25));
    std::mt19937 generator(42);
    std::uniform_int_distribution<int> x(30, 610);
    std::uniform_int_distribution<int> y(210, 450);
    std::uniform_int_distribution<int> color(80, 240);
    for (int i = 0; i < 900; ++i) {
        const int value = color(generator);
        cv::circle(first, {x(generator), y(generator)}, 2,
                   cv::Scalar(value, value, value), cv::FILLED);
    }

    const auto still = odometry.estimate(first, first);
    if (!still.valid || std::abs(still.forward_m) > 0.001) {
        std::cerr << "stationary estimate failed\n";
        return 1;
    }

    const cv::Matx33d ground_to_image =
        config.image_to_vehicle_ground.inv();
    const cv::Matx33d translate_ground(1.0, 0.0, -0.025,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0);
    const cv::Matx33d image_motion = ground_to_image * translate_ground *
                                     config.image_to_vehicle_ground;
    cv::Mat second;
    cv::warpPerspective(first, second, cv::Mat(image_motion), first.size());
    const auto moved = odometry.estimate(first, second);
    if (!moved.valid || std::abs(moved.forward_m - 0.025) > 0.006) {
        std::cerr << "forward estimate failed: " << moved.forward_m
                  << " valid=" << moved.valid << '\n';
        return 1;
    }

    constexpr double yaw_rad = 0.012;
    const double cosine = std::cos(yaw_rad);
    const double sine = std::sin(yaw_rad);
    const cv::Matx33d curved_ground(
        cosine, sine, -0.025 * cosine,
        -sine, cosine, 0.025 * sine,
        0.0, 0.0, 1.0);
    const cv::Matx33d curved_image_motion = ground_to_image * curved_ground *
                                            config.image_to_vehicle_ground;
    cv::Mat curved;
    cv::warpPerspective(first, curved, cv::Mat(curved_image_motion), first.size());
    const auto turned = odometry.estimate(first, curved);
    if (!turned.valid || std::abs(turned.yaw_rad - yaw_rad) > 0.008) {
        std::cerr << "yaw estimate failed: " << turned.yaw_rad
                  << " expected=" << yaw_rad << " valid=" << turned.valid << '\n';
        return 1;
    }

    std::cout << "PASS: ground visual odometry synthetic checks\n";
    return 0;
}
