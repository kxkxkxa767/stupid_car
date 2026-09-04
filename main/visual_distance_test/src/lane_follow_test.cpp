#include "xtnetrc_visual_distance/lane_corridor.hpp"
#include "xtnetrc_visual_distance/pigpio_vehicle.hpp"
#include "xtnetrc_vision/ground_projector.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/videoio.hpp>

namespace vd = xtnetrc::visual_distance;
namespace vision = xtnetrc::vision;

namespace {
std::atomic_bool interrupted{false};
void handle_signal(int) { interrupted = true; }
}

int main(int argc, char* argv[]) try {
    bool enable_motor = false;
    bool acknowledged = false;
    bool steering_check_only = false;
    std::string calibration = "ground_projection.json";
    std::string debug_video;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--enable-motor") enable_motor = true;
        else if (argument == "--i-understand-vehicle-will-move") acknowledged = true;
        else if (argument == "--steering-check-only") steering_check_only = true;
        else if (argument == "--calibration" && index + 1 < argc) calibration = argv[++index];
        else if (argument == "--debug-video" && index + 1 < argc) debug_video = argv[++index];
        else throw std::runtime_error("unknown or incomplete argument: " + argument);
    }
    if (enable_motor != acknowledged) {
        throw std::runtime_error("motor execution requires both explicit safety switches");
    }
    if (steering_check_only && enable_motor) {
        throw std::runtime_error("steering check cannot be combined with motor execution");
    }
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const auto projection = vision::load_ground_projection_json(calibration);
    vd::LaneCorridorDetector detector(projection);
    cv::VideoCapture camera(0, cv::CAP_V4L2);
    if (!camera.isOpened()) throw std::runtime_error("cannot open fixed camera 0");
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    camera.set(cv::CAP_PROP_FPS, 30);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    cv::VideoWriter debug_writer;
    if (!debug_video.empty() &&
        !debug_writer.open(debug_video, cv::VideoWriter::fourcc('M','J','P','G'),
                           10.0, cv::Size(640, 480))) {
        throw std::runtime_error("cannot open fixed-camera debug video");
    }

    cv::Mat frame;
    for (int index = 0; index < 20; ++index) {
        if (!camera.read(frame)) throw std::runtime_error("camera warmup failed");
    }
    int valid_frames = 0;
    double center_sum = 0.0;
    double width_sum = 0.0;
    double heading_sum = 0.0;
    int debug_frame_counter = 0;
    for (int index = 0; index < 45 && !interrupted; ++index) {
        if (!camera.read(frame)) throw std::runtime_error("camera preflight read failed");
        const auto lane = detector.process(frame);
        if (debug_writer.isOpened() && (++debug_frame_counter % 3) == 0) {
            debug_writer.write(lane.overlay.empty() ? frame : lane.overlay);
        }
        if (lane.valid) {
            ++valid_frames;
            center_sum += lane.center_y_m;
            width_sum += lane.width_m;
            heading_sum += lane.heading_slope;
        }
    }
    std::cout << std::fixed << std::setprecision(3)
              << "PREFLIGHT valid=" << valid_frames << "/45";
    if (valid_frames > 0) {
        std::cout << " center_y_m=" << center_sum / valid_frames
                  << " width_m=" << width_sum / valid_frames
                  << " heading_slope=" << heading_sum / valid_frames;
    }
    std::cout << '\n' << std::flush;
    // 夜间参考画面允许较低的瞬时检出率，但每个有效帧仍须通过双边、宽度和中心约束。
    if (interrupted || valid_frames < 24) {
        throw std::runtime_error("double solid lane preflight rejected");
    }
    if (!enable_motor && !steering_check_only) {
        std::cout << "PRECHECK_PASS camera=0 hardware_output=disabled\n";
        return 0;
    }

    // 72 是停车/机械中点；71.5 让闭环按误差在整数 71/72 之间选择，避免固定偏置。
    constexpr int mechanical_steering_center = 72;
    constexpr double driving_steering_center = 71.5;
    vd::PigpioVehicle car;
    car.initialize(mechanical_steering_center);
    if (steering_check_only) {
        std::cout << "STEERING_CHECK pwm=" << mechanical_steering_center
                  << " motor_pwm=10000 hold_s=3\n" << std::flush;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        car.stop();
        std::cout << "STEERING_CHECK_COMPLETE motor_never_started\n";
        return 0;
    }
    std::cout << "ESC_ARMING neutral_pwm=10000 wait_s=5\n" << std::flush;
    for (int index = 0; index < 50 && !interrupted; ++index) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (interrupted) throw std::runtime_error("interrupted before motor start");
    car.drive_forward(11100);
    std::cout << "MOTOR_START pwm=11100 mechanical_steering_center="
              << mechanical_steering_center
              << " driving_steering_center=" << driving_steering_center
              << " max_s=5\n" << std::flush;

    const auto start = std::chrono::steady_clock::now();
    int invalid_frames = 0;
    int steering_pwm = 72;
    int log_counter = 0;
    double filtered_center_y_m = center_sum / valid_frames;
    double filtered_heading_slope = 0.0;
    double expected_width_m = width_sum / valid_frames;
    while (!interrupted) {
        if (!camera.read(frame)) throw std::runtime_error("camera read failed");
        const auto lane = detector.process(frame);
        if (debug_writer.isOpened() && (++debug_frame_counter % 3) == 0) {
            debug_writer.write(lane.overlay.empty() ? frame : lane.overlay);
        }
        const bool geometry_consistent = lane.valid &&
            std::abs(lane.width_m - expected_width_m) <= 0.12 &&
            std::abs(lane.center_y_m - filtered_center_y_m) <= 0.08 &&
            std::abs(lane.heading_slope - filtered_heading_slope) <= 0.20;
        if (!geometry_consistent) {
            if (++invalid_frames >= 10) {
                throw std::runtime_error("current double solid lane lost");
            }
        } else {
            invalid_frames = 0;
            filtered_center_y_m =
                0.90 * filtered_center_y_m + 0.10 * lane.center_y_m;
            filtered_heading_slope =
                0.90 * filtered_heading_slope + 0.10 * lane.heading_slope;
            expected_width_m = 0.98 * expected_width_m + 0.02 * lane.width_m;
            // y 左正右负；实测 PWM 增大使车辆左转。限制在中心附近的小角度范围。
            const double controlled_center = std::abs(filtered_center_y_m) < 0.025
                                                 ? 0.0
                                                 : filtered_center_y_m;
            const double requested = driving_steering_center +
                                     18.0 * controlled_center +
                                     3.0 * filtered_heading_slope;
            const int target = std::clamp(static_cast<int>(std::lround(requested)),
                                          70, 73);
            if (target > steering_pwm) ++steering_pwm;
            else if (target < steering_pwm) --steering_pwm;
            car.set_steering_pwm(steering_pwm);
            if ((log_counter++ % 5) == 0) {
                std::cout << "lane_center_y_m=" << lane.center_y_m
                          << " filtered_center_y_m=" << filtered_center_y_m
                          << " width_m=" << lane.width_m
                          << " heading_slope=" << lane.heading_slope
                          << " confidence=" << lane.confidence
                          << " steering_pwm=" << steering_pwm << '\n' << std::flush;
            }
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= 5.0) break;
    }
    car.stop();
    camera.release();
    std::cout << "MOTOR_STOP reason=" << (interrupted ? "signal" : "5s test complete")
              << '\n';
    return interrupted ? 3 : 0;
} catch (const std::exception& error) {
    std::cerr << "STOP error=\"" << error.what() << "\"\n";
    return 2;
}
