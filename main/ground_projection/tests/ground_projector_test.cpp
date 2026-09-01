#include "xtnetrc_vision/ground_projector.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace projection = xtnetrc::vision;

int main() {
    int failures = 0;
    const auto expect = [&](const bool condition, const char* message) {
        if (!condition) {
            ++failures;
            std::cerr << "失败: " << message << '\n';
        }
    };

    projection::GroundProjectionConfig config;
    config.image_width = 640;
    config.image_height = 480;
    config.camera_matrix = {500.0, 0.0, 320.0,
                            0.0, 500.0, 240.0,
                            0.0, 0.0, 1.0};
    config.dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
    config.image_to_vehicle_ground = {0.01, 0.0, -3.2,
                                      0.0, 0.01, -2.4,
                                      0.0, 0.0, 1.0};
    projection::GroundProjector projector(config);
    const auto points = projector.project({{320.0F, 240.0F}, {420.0F, 240.0F}},
                                          {640, 480});
    expect(points.size() == 2, "应保留投影点数量");
    expect(std::abs(points[0].x_forward_m) < 1e-6 &&
               std::abs(points[0].y_left_m) < 1e-6,
           "主点应映射到地面原点");
    expect(std::abs(points[1].x_forward_m - 1.0) < 1e-5,
           "单应矩阵的米制缩放应生效");

    auto distorted_config = config;
    distorted_config.dist_coeffs = {-0.2, 0.05, 0.001, -0.002, 0.01};
    distorted_config.image_to_vehicle_ground = cv::Matx33d::eye();
    projection::GroundProjector distorted_projector(distorted_config);
    const auto corrected = distorted_projector.project({{100.0F, 100.0F}}, {640, 480});
    expect(std::abs(corrected[0].x_forward_m - 87.61083) < 1e-3 &&
               std::abs(corrected[0].y_left_m - 91.750656) < 1e-3,
           "自包含去畸变结果应与 OpenCV 标准实现一致");

    bool size_threw = false;
    try {
        static_cast<void>(projector.project({{1.0F, 1.0F}}, {1280, 720}));
    } catch (const std::invalid_argument&) {
        size_threw = true;
    }
    expect(size_threw, "分辨率与标定不一致时必须拒绝投影");

    bool nan_threw = false;
    try {
        static_cast<void>(projector.project(
            {{std::numeric_limits<float>::quiet_NaN(), 1.0F}}, {640, 480}));
    } catch (const std::invalid_argument&) {
        nan_threw = true;
    }
    expect(nan_threw, "NaN 像素点必须被拒绝");

    const auto loaded = projection::load_ground_projection_json(
        XTNETRC_GROUND_TEST_CONFIG_PATH);
    expect(loaded.image_width == 640 && loaded.image_height == 480,
           "应能读取 Python 工具输出格式的 JSON");
    expect(std::abs(loaded.image_to_vehicle_ground(0, 0) - 0.01) < 1e-12,
           "应正确读取单应矩阵");

    const auto real_config = projection::load_ground_projection_json(
        XTNETRC_REAL_GROUND_CONFIG_PATH);
    projection::GroundProjector real_projector(real_config);
    // 标定点属于外参数据，摄像头位置变化后也会变化。测试直接读取当前正式
    // 配置，避免把某一次姿态的旧像素坐标永久写死在代码里。
    cv::FileStorage real_file(XTNETRC_REAL_GROUND_CONFIG_PATH,
                              cv::FileStorage::READ |
                                  cv::FileStorage::FORMAT_JSON);
    std::vector<cv::Point2f> real_image_points;
    std::vector<projection::GroundPoint> expected;
    for (const auto& row : real_file["image_points_undistorted_px"]) {
        if (row.isSeq() && row.size() == 2) {
            auto value = row.begin();
            const float x = static_cast<float>(*value);
            const float y = static_cast<float>(*(++value));
            real_image_points.emplace_back(x, y);
        }
    }
    for (const auto& row : real_file["vehicle_points_m"]) {
        if (row.isSeq() && row.size() == 2) {
            auto value = row.begin();
            const double x = static_cast<double>(*value);
            const double y = static_cast<double>(*(++value));
            expected.push_back({x, y});
        }
    }
    expect(real_file.isOpened() && real_image_points.size() == 4 &&
               expected.size() == 4,
           "正式标定文件必须保存四对像素/车辆参考点");
    const auto real_points = real_projector.project(real_image_points, {640, 480});
    for (std::size_t index = 0; index < real_points.size(); ++index) {
        expect(std::abs(real_points[index].x_forward_m -
                        expected[index].x_forward_m) < 1e-5 &&
                   std::abs(real_points[index].y_left_m -
                            expected[index].y_left_m) < 1e-5,
               "C++ 应准确回代实车四个地面标定点");
    }

    if (failures != 0) {
        return 1;
    }
    std::cout << "地面投影测试全部通过\n";
    return 0;
}
