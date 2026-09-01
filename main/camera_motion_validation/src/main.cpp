#include "lane_detector.hpp"
#include "xtnetrc_motion/controller.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>

namespace motion = xtnetrc::motion;

namespace {
struct Options {
    int camera{0};
    std::string config_path{"motion_default.json"};
    std::size_t max_frames{300};
};

Options parse_options(const int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) {
                throw std::runtime_error(argument + " 缺少参数");
            }
            return argv[i];
        };
        if (argument == "--camera") {
            options.camera = std::stoi(value());
        } else if (argument == "--config") {
            options.config_path = value();
        } else if (argument == "--frames") {
            options.max_frames = static_cast<std::size_t>(std::stoul(value()));
        } else {
            throw std::runtime_error("未知参数: " + argument);
        }
    }
    return options;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        const motion::MotionConfig config =
            motion::load_motion_config_json(options.config_path);
        motion::MotionController controller(config);
        xtnetrc::LaneDetector detector;
        cv::VideoCapture camera(options.camera);
        if (!camera.isOpened()) {
            throw std::runtime_error("无法打开摄像头");
        }

        // 这里只用合成直线路径验证摄像头置信度、安全状态和速度斜坡。
        // 未做相机鸟瞰/米制标定前，绝不把像素误差伪装成真实转向路径。
        const std::vector<motion::PathPoint> safe_test_path{{0.20, 0.0}, {0.60, 0.0}};
        const std::vector<motion::PathPoint> empty_path;
        auto previous = std::chrono::steady_clock::now();
        cv::Mat frame;
        std::size_t violations = 0;
        std::size_t frame_number = 0;
        while (frame_number < options.max_frames && camera.read(frame)) {
            const auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - previous).count();
            previous = now;
            if (frame_number == 0 || dt <= 0.0 || dt > 1.0) {
                dt = 1.0 / 30.0;
            }

            const xtnetrc::LaneResult lane = detector.process(frame);
            const auto& path = lane.detected ? safe_test_path : empty_path;
            const motion::ControlCommand command =
                controller.update(path, lane.confidence, dt, true);

            const bool zero_required = command.state == motion::ControlState::Lost ||
                                       command.state == motion::ControlState::Stopped;
            if ((zero_required && (command.target_speed_mps != 0.0 ||
                                   command.steering_rad != 0.0)) ||
                command.target_speed_mps < 0.0 ||
                command.target_speed_mps > config.max_speed_mps) {
                ++violations;
            }
            if ((frame_number % 10) == 0) {
                std::cout << std::fixed << std::setprecision(3)
                          << "frame=" << frame_number
                          << " confidence=" << lane.confidence
                          << " state=" << motion::to_string(command.state)
                          << " speed=" << command.target_speed_mps
                          << " steering_deg=" << command.steering_deg()
                          << " reason=\"" << command.reason << "\"\n";
            }
            ++frame_number;
        }

        std::cout << "完成帧数=" << frame_number
                  << " 安全约束违规=" << violations
                  << " 硬件输出=disabled\n";
        return violations == 0 && frame_number > 0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 2;
    }
}
