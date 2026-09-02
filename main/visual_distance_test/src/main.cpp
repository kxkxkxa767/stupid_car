#include "xtnetrc_visual_distance/pigpio_vehicle.hpp"
#include "xtnetrc_visual_distance/visual_odometry.hpp"
#include "xtnetrc_vision/ground_projector.hpp"

#include <atomic>
#include <algorithm>
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

struct Options {
    int camera{0};
    std::string calibration{"ground_projection.json"};
    double target_m{1.0};
    double max_seconds{10.0};
    double distance_scale{1.0};
    int motor_pwm{10250};
    int steering_center_pwm{72};
    bool enable_motor{false};
    bool acknowledged{false};
};

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto value = [&]() {
            if (++i >= argc) throw std::runtime_error(argument + " missing value");
            return std::string(argv[i]);
        };
        if (argument == "--camera") options.camera = std::stoi(value());
        else if (argument == "--calibration") options.calibration = value();
        else if (argument == "--target-m") options.target_m = std::stod(value());
        else if (argument == "--max-seconds") options.max_seconds = std::stod(value());
        else if (argument == "--distance-scale") options.distance_scale = std::stod(value());
        else if (argument == "--motor-pwm") options.motor_pwm = std::stoi(value());
        else if (argument == "--steering-center-pwm") options.steering_center_pwm = std::stoi(value());
        else if (argument == "--enable-motor") options.enable_motor = true;
        else if (argument == "--i-understand-vehicle-will-move") options.acknowledged = true;
        else throw std::runtime_error("unknown argument: " + argument);
    }
    if (options.target_m <= 0.0 || options.target_m > 1.0 ||
        options.max_seconds <= 0.0 || options.max_seconds > 12.0 ||
        options.distance_scale < 0.5 || options.distance_scale > 1.5 ||
        options.motor_pwm < 10200 || options.motor_pwm > 11200 ||
        options.steering_center_pwm < 60 || options.steering_center_pwm > 80) {
        throw std::runtime_error("test parameters exceed hard safety limits");
    }
    if (options.enable_motor != options.acknowledged) {
        throw std::runtime_error("motor execution requires both explicit safety switches");
    }
    return options;
}
}  // namespace

int main(int argc, char** argv) {
    vd::PigpioVehicle vehicle;
    try {
        const Options options = parse_options(argc, argv);
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        const auto calibration =
            vision::load_ground_projection_json(options.calibration);
        vd::GroundVisualOdometry odometry(calibration);
        cv::VideoCapture camera(options.camera, cv::CAP_V4L2);
        if (!camera.isOpened()) throw std::runtime_error("cannot open camera");
        camera.set(cv::CAP_PROP_FRAME_WIDTH, calibration.image_width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, calibration.image_height);
        camera.set(cv::CAP_PROP_FPS, 30);
        camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

        cv::Mat previous;
        cv::Mat current;
        for (int i = 0; i < 20; ++i) {
            if (!camera.read(previous)) throw std::runtime_error("camera warmup failed");
        }

        int valid_preflight = 0;
        double preflight_drift = 0.0;
        for (int i = 0; i < 30 && !interrupted; ++i) {
            if (!camera.read(current)) throw std::runtime_error("camera read failed");
            const auto estimate = odometry.estimate(previous, current);
            if (estimate.valid) {
                ++valid_preflight;
                preflight_drift += estimate.forward_m;
            }
            previous = current.clone();
        }
        std::cout << std::fixed << std::setprecision(4)
                  << "preflight_valid=" << valid_preflight << "/30"
                  << " drift_m=" << preflight_drift << '\n';
        if (interrupted || valid_preflight < 24 || std::abs(preflight_drift) > 0.035) {
            throw std::runtime_error("visual preflight rejected");
        }
        if (!options.enable_motor) {
            std::cout << "PRECHECK_PASS hardware_output=disabled\n";
            return 0;
        }

        vehicle.initialize(options.steering_center_pwm);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        vehicle.drive_forward(options.motor_pwm);
        const auto start = std::chrono::steady_clock::now();
        double distance = 0.0;
        double heading_rad = 0.0;
        int invalid_frames = 0;
        int frame = 0;
        while (!interrupted && distance < options.target_m) {
            if (!camera.read(current)) throw std::runtime_error("camera read failed while moving");
            const auto estimate = odometry.estimate(previous, current);
            previous = current.clone();
            if (!estimate.valid) {
                if (++invalid_frames >= 5) {
                    throw std::runtime_error("visual tracking lost");
                }
            } else {
                const double scaled_step = estimate.forward_m * options.distance_scale;
                // 30 FPS 下单帧 4 cm 已对应约 1.2 m/s，超过本车当前测试速度。
                // 拒绝相位相关跳到相邻黑线或重复地面纹理造成的假位移。
                if (std::abs(scaled_step) > 0.04) {
                    if (++invalid_frames >= 5) {
                        throw std::runtime_error("visual step exceeds physical limit");
                    }
                } else {
                    invalid_frames = 0;
                    distance = std::max(0.0, distance + scaled_step);
                    heading_rad += estimate.yaw_rad;
                    if ((frame++ % 5) == 0) {
                        std::cout << "distance_m=" << distance
                                  << " raw_step_m=" << estimate.forward_m
                                  << " scaled_step_m=" << scaled_step
                                  << " heading_deg=" << heading_rad * 180.0 / M_PI
                                  << " tracks=" << estimate.tracked_points
                                  << " dispersion_m=" << estimate.dispersion_m << '\n';
                    }
                }
            }
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed >= options.max_seconds) {
                throw std::runtime_error("maximum run time reached before target");
            }
        }
        vehicle.stop();
        constexpr double wheelbase_m = 0.23861;
        const double steering_rad = distance > 0.05
            ? std::atan(wheelbase_m * heading_rad / distance)
            : 0.0;
        std::cout << "STOP target_reached distance_m=" << distance
                  << " heading_deg=" << heading_rad * 180.0 / M_PI
                  << " estimated_steering_deg=" << steering_rad * 180.0 / M_PI
                  << '\n';
        return distance >= options.target_m ? 0 : 1;
    } catch (const std::exception& error) {
        vehicle.stop();
        std::cerr << "STOP error=\"" << error.what() << "\"\n";
        return 2;
    }
}
