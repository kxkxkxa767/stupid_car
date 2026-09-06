#include "xtnetrc_visual_distance/visual_odometry.hpp"
#include "xtnetrc_camera/frame_source.hpp"
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
namespace {
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }
}
// Read-only odometry diagnostic. All motor ownership lives in car_runtime.
int main(int argc, char** argv) try {
    std::string calibration_path{"ground_projection.json"}, device;
    bool mismatch = false;
    double duration = 10;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto value = [&]() {
            if (++i == argc) throw std::invalid_argument("missing option value");
            return std::string(argv[i]);
        };
        if (argument == "--calibration") calibration_path = value();
        else if (argument == "--camera-device") device = value();
        else if (argument == "--allow-camera-mismatch") mismatch = true;
        else if (argument == "--max-seconds") {
            const auto text = value(); std::size_t used = 0; duration = std::stod(text, &used);
            if (used != text.size()) throw std::invalid_argument("invalid duration");
        } else if (argument == "--help") {
            std::cout << "Read-only VO: --calibration PATH --camera-device PATH --max-seconds N\n"
                         "Motor experiments moved to xtnetrc_car_runtime.\n";
            return 0;
        } else throw std::invalid_argument("unknown/retired option: " + argument + "; motors require xtnetrc_car_runtime");
    }
    if (!std::isfinite(duration) || duration <= 0 || duration > 60)
        throw std::invalid_argument("duration must be 0..60 seconds");
    std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler);
    const auto calibration = xtnetrc::vision::load_ground_projection_json(calibration_path);
    const auto identity = xtnetrc::camera::check_identity(calibration, device, mismatch);
    std::cout << "camera=" << identity.device << " matched=" << identity.matched << " hardware_output=never_enabled\n";
    auto camera = xtnetrc::camera::open_fixed_camera(calibration, identity);
    xtnetrc::visual_distance::GroundVisualOdometry odometry(calibration);
    const auto start = xtnetrc::camera::Clock::now();
    cv::Mat previous;
    double distance = 0;
    while (!interrupted && std::chrono::duration<double>(xtnetrc::camera::Clock::now() - start).count() < duration) {
        xtnetrc::camera::Frame current;
        if (!camera->read(current, std::chrono::milliseconds(100))) { previous.release(); continue; }
        if (!previous.empty()) {
            const auto estimate = odometry.estimate(previous, current.image);
            if (estimate.valid) distance += estimate.forward_m;
            std::cout << "valid=" << estimate.valid << " forward_m=" << estimate.forward_m
                      << " distance_estimate_m=" << distance << '\n';
        }
        previous = current.image;
    }
    return interrupted ? 130 : 0;
} catch (const std::exception& error) {
    std::cerr << "READ_ONLY_STOP " << error.what() << '\n'; return 2;
}
