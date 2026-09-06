#include "xtnetrc_speed/speed_loop.hpp"
#include "xtnetrc_speed/wit_sensor.hpp"
#include "xtnetrc_hardware/byte_source.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace speed = xtnetrc::speed;

namespace {

struct Options {
    std::string port{"/dev/ttyUSB0"};
    int duration_s{60};
    char forward_axis{'x'};
    double forward_sign{1.0};
};

[[noreturn]] void usage(const char* program, const int exit_code) {
    std::ostream& output = exit_code == 0 ? std::cout : std::cerr;
    output << "用法: " << program
           << " [--port /dev/ttyUSB0] [--duration 秒]"
              " [--forward-axis x|-x|y|-y]\n"
              "只读取 WIT GPS/IMU，不访问 GPIO，不启动电机。\n";
    std::exit(exit_code);
}

Options parse_options(const int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help") {
            usage(argv[0], 0);
        }
        if (index + 1 >= argc) {
            usage(argv[0], 2);
        }
        const std::string value = argv[++index];
        if (argument == "--port") {
            options.port = value;
        } else if (argument == "--duration") {
            options.duration_s = std::stoi(value);
            if (options.duration_s <= 0 || options.duration_s > 3600) {
                throw std::invalid_argument("duration must be in 1..3600 seconds");
            }
        } else if (argument == "--forward-axis") {
            if (value == "x" || value == "+x") {
                options.forward_axis = 'x';
                options.forward_sign = 1.0;
            } else if (value == "-x") {
                options.forward_axis = 'x';
                options.forward_sign = -1.0;
            } else if (value == "y" || value == "+y") {
                options.forward_axis = 'y';
                options.forward_sign = 1.0;
            } else if (value == "-y") {
                options.forward_axis = 'y';
                options.forward_sign = -1.0;
            } else {
                throw std::invalid_argument("forward axis must be x, -x, y or -y");
            }
        } else {
            usage(argv[0], 2);
        }
    }
    return options;
}

double seconds_since(const std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

double forward_acceleration(const speed::Vector3& value, const Options& options) {
    const double selected = options.forward_axis == 'x' ? value.x : value.y;
    return options.forward_sign * selected;
}

}  // namespace

int main(int argc, char* argv[]) try {
    const Options options = parse_options(argc, argv);
    auto serial = xtnetrc::hardware::open_serial(options.port, 9600);
    speed::WitFrameParser parser;
    speed::GpsImuSpeedEstimator estimator;
    const auto start = std::chrono::steady_clock::now();
    double last_print_s = -1.0;
    std::vector<double> bias_samples;
    bool bias_ready = false;
    std::optional<double> latest_acceleration;

    std::cout << "mode=read-only port=" << options.port
              << " baud=9600 duration_s=" << options.duration_s
              << " forward_axis=" << (options.forward_sign < 0.0 ? "-" : "")
              << options.forward_axis << '\n';

    while (seconds_since(start) < options.duration_s) {
        std::uint8_t buffer[256];
        const auto count = serial->read(buffer, sizeof(buffer), std::chrono::milliseconds(200));
        const double now_s = seconds_since(start);
        if (count > 0) {
            for (std::size_t index = 0; index < count; ++index) {
                const auto type = parser.feed(buffer[index]);
                if (!type) {
                    continue;
                }
                const auto& data = parser.data();
                if (*type == 0x51 && data.acceleration_mps2) {
                    const double acceleration =
                        forward_acceleration(*data.acceleration_mps2, options);
                    latest_acceleration = acceleration;
                    if (now_s <= 3.0) {
                        bias_samples.push_back(acceleration);
                    } else {
                        if (!bias_ready) {
                            double total = 0.0;
                            for (const double sample : bias_samples) {
                                total += sample;
                            }
                            const double bias = bias_samples.empty()
                                                    ? 0.0
                                                    : total / bias_samples.size();
                            estimator.set_acceleration_bias(bias);
                            bias_ready = true;
                            std::cout << "acceleration_bias_mps2=" << bias << '\n';
                        }
                        estimator.update_acceleration(acceleration, now_s);
                    }
                } else if (*type == 0x58 && data.gps_speed_mps && data.satellites &&
                           data.hdop) {
                    static_cast<void>(estimator.update_gps(*data.gps_speed_mps,
                                                           *data.satellites,
                                                           *data.hdop, now_s));
                }
            }
        }

        if (now_s - last_print_s >= 1.0) {
            const auto& data = parser.data();
            const speed::SpeedEstimate estimate = estimator.estimate(now_s);
            std::cout << std::fixed << std::setprecision(3)
                      << "t=" << now_s
                      << " sat=" << data.satellites.value_or(0)
                      << " hdop=" << data.hdop.value_or(0.0)
                      << " gps_mps=" << data.gps_speed_mps.value_or(0.0)
                      << " fused_mps=" << estimate.speed_mps
                      << " valid=" << (estimate.valid ? 1 : 0)
                      << " accel_mps2=" << latest_acceleration.value_or(0.0)
                      << " frames=" << data.valid_frames
                      << " bad=" << data.bad_checksums << '\n';
            last_print_s = now_s;
        }
    }
    return 0;
} catch (const std::exception& error) {
    std::cerr << "错误: " << error.what() << '\n';
    return 1;
}
