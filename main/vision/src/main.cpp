#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "lane_detector.hpp"

namespace {

struct Options {
    int camera_index = 0;
    std::string video_path;
    bool headless = false;
    int width = 640;
    int height = 480;
    std::string snapshot_path;
    std::string capture_dir;
    std::size_t capture_count = 0;
    int capture_interval_ms = 1000;
    xtnetrc::LaneConfig lane;
};

void print_usage(const char* program) {
    std::cout
        << "用法:\n"
        << "  " << program << " [--camera 0]\n"
        << "  " << program << " --video /path/to/video.mp4\n\n"
        << "选项:\n"
        << "  --headless          不显示窗口，只输出检测数值\n"
        << "  --width N           采集宽度，默认 640（标定后不可随意更改）\n"
        << "  --height N          采集高度，默认 480（标定后不可随意更改）\n"
        << "  --snapshot PATH     预热后保存一张原始图像并退出\n"
        << "  --capture-dir DIR   定时保存一组相机标定原始图像\n"
        << "  --capture-count N   标定图数量，和 --capture-dir 配合使用\n"
        << "  --capture-interval-ms N  标定图间隔，默认 1000 ms\n"
        << "  --canny-low N       Canny 低阈值，默认 40\n"
        << "  --canny-high N      Canny 高阈值，默认 80\n"
        << "  --kp N              比例系数，默认 0.15\n"
        << "  --kd N              微分系数，默认 0.10\n"
        << "  --help              显示帮助\n";
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const char* option) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string(option) + " 缺少参数");
            }
            return argv[++i];
        };

        if (arg == "--camera") {
            options.camera_index = std::stoi(require_value("--camera"));
        } else if (arg == "--video") {
            options.video_path = require_value("--video");
        } else if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--width") {
            options.width = std::stoi(require_value("--width"));
        } else if (arg == "--height") {
            options.height = std::stoi(require_value("--height"));
        } else if (arg == "--snapshot") {
            options.snapshot_path = require_value("--snapshot");
            options.headless = true;
        } else if (arg == "--capture-dir") {
            options.capture_dir = require_value("--capture-dir");
            options.headless = true;
        } else if (arg == "--capture-count") {
            options.capture_count = std::stoul(require_value("--capture-count"));
        } else if (arg == "--capture-interval-ms") {
            options.capture_interval_ms =
                std::stoi(require_value("--capture-interval-ms"));
        } else if (arg == "--canny-low") {
            options.lane.canny_low = std::stod(require_value("--canny-low"));
        } else if (arg == "--canny-high") {
            options.lane.canny_high = std::stod(require_value("--canny-high"));
        } else if (arg == "--kp") {
            options.lane.kp = std::stod(require_value("--kp"));
        } else if (arg == "--kd") {
            options.lane.kd = std::stod(require_value("--kd"));
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("未知参数: " + arg);
        }
    }
    if (options.width <= 0 || options.height <= 0 || options.capture_interval_ms <= 0) {
        throw std::runtime_error("图像尺寸和采集间隔必须为正数");
    }
    if (!options.capture_dir.empty() && options.capture_count == 0) {
        throw std::runtime_error("--capture-dir 必须同时指定正数 --capture-count");
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        cv::VideoCapture capture;

        if (options.video_path.empty()) {
            capture.open(options.camera_index);
        } else {
            capture.open(options.video_path);
        }

        if (!capture.isOpened()) {
            std::cerr << "无法打开摄像头或视频输入。\n";
            return 1;
        }
        capture.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);

        if (!options.snapshot_path.empty()) {
            const std::filesystem::path parent =
                std::filesystem::path(options.snapshot_path).parent_path();
            if (!parent.empty()) {
                std::filesystem::create_directories(parent);
            }
        }
        if (!options.capture_dir.empty()) {
            std::filesystem::create_directories(options.capture_dir);
        }

        xtnetrc::LaneDetector detector(options.lane);
        cv::Mat frame;
        std::size_t frame_number = 0;
        std::size_t saved_count = 0;
        const auto session_id = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        auto next_capture = std::chrono::steady_clock::now();

        while (capture.read(frame)) {
            // 前 10 帧用于自动曝光稳定，保存的一律是未经叠加绘制的原始画面。
            if (frame_number >= 10 && !options.snapshot_path.empty()) {
                if (!cv::imwrite(options.snapshot_path, frame)) {
                    throw std::runtime_error("无法保存快照: " + options.snapshot_path);
                }
                std::cout << "已保存原始快照: " << options.snapshot_path << '\n';
                break;
            }
            if (frame_number >= 10 && !options.capture_dir.empty() &&
                std::chrono::steady_clock::now() >= next_capture) {
                const std::filesystem::path path =
                    std::filesystem::path(options.capture_dir) /
                    ("charuco_" + std::to_string(session_id) + "_" +
                     std::to_string(saved_count) + ".jpg");
                if (!cv::imwrite(path.string(), frame)) {
                    throw std::runtime_error("无法保存标定图: " + path.string());
                }
                ++saved_count;
                std::cout << "已保存标定图 " << saved_count << "/"
                          << options.capture_count << ": " << path << '\n';
                if (saved_count >= options.capture_count) {
                    break;
                }
                next_capture = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(options.capture_interval_ms);
            }

            const xtnetrc::LaneResult result = detector.process(frame);

            if ((frame_number++ % 10) == 0) {
                std::cout << std::fixed << std::setprecision(2)
                          << "detected=" << (result.detected ? "yes" : "no")
                          << " confidence=" << result.confidence
                          << " error_px=" << result.error_px
                          << " simulated_steering_deg=" << result.steering_deg
                          << '\n';
            }

            if (!options.headless) {
                cv::imshow("XT-NetRC lane view", result.overlay);
                cv::imshow("XT-NetRC edges", result.edges);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') {
                    break;
                }
            }
        }

        capture.release();
        cv::destroyAllWindows();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        print_usage(argv[0]);
        return 2;
    }
}
