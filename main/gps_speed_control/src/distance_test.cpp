#include "xtnetrc_speed/wit_sensor.hpp"
#include "xtnetrc_visual_distance/pigpio_vehicle.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace speed = xtnetrc::speed;
namespace vehicle = xtnetrc::visual_distance;

namespace {

std::atomic_bool stop_requested{false};

void handle_signal(int) {
    stop_requested.store(true);
}

double now_s(const std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 == 0 ? (values[middle - 1] + values[middle]) / 2.0
                                  : values[middle];
}

int open_serial() {
    const int descriptor = ::open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
        throw std::runtime_error("cannot open /dev/ttyUSB0");
    }
    termios settings{};
    if (tcgetattr(descriptor, &settings) != 0) {
        ::close(descriptor);
        throw std::runtime_error("tcgetattr failed");
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, B9600);
    cfsetospeed(&settings, B9600);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    if (tcsetattr(descriptor, TCSANOW, &settings) != 0) {
        ::close(descriptor);
        throw std::runtime_error("tcsetattr failed");
    }
    tcflush(descriptor, TCIFLUSH);
    return descriptor;
}

template <typename Callback>
void read_available(const int descriptor, speed::WitFrameParser& parser,
                    const Callback& on_frame) {
    pollfd item{descriptor, POLLIN, 0};
    static_cast<void>(::poll(&item, 1, 100));
    std::uint8_t bytes[256];
    const ssize_t count = ::read(descriptor, bytes, sizeof(bytes));
    if (count <= 0) {
        return;
    }
    for (ssize_t index = 0; index < count; ++index) {
        const auto type = parser.feed(bytes[index]);
        if (type) {
            on_frame(*type, parser.data());
        }
    }
}

}  // namespace

int main(int argc, char* argv[]) try {
    bool enable_motor = false;
    bool acknowledged = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        enable_motor |= argument == "--enable-motor";
        acknowledged |= argument == "--i-understand-vehicle-will-move";
    }
    if (!enable_motor || !acknowledged) {
        std::cerr << "拒绝启动：必须同时指定 --enable-motor 和 "
                     "--i-understand-vehicle-will-move\n";
        return 2;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    const int serial = open_serial();
    speed::WitFrameParser parser;
    int satellites = 0;
    double hdop = 99.0;
    double last_speed_accept_s = -99.0;
    std::vector<double> stationary_speeds;
    const auto preflight_start = std::chrono::steady_clock::now();

    std::cout << "PREFLIGHT GPS 5s motor=neutral\n" << std::flush;
    while (now_s(preflight_start) < 5.0 && !stop_requested.load()) {
        read_available(serial, parser, [&](const std::uint8_t type,
                                           const speed::WitSensorData& data) {
            const double elapsed = now_s(preflight_start);
            if (type == 0x5A && data.satellites && data.hdop) {
                satellites = *data.satellites;
                hdop = *data.hdop;
            }
            if (type == 0x58 && data.gps_speed_mps && satellites >= 6 &&
                hdop > 0.0 && hdop <= 3.0 && elapsed - last_speed_accept_s >= 0.18) {
                stationary_speeds.push_back(*data.gps_speed_mps);
                last_speed_accept_s = elapsed;
            }
        });
    }
    if (stop_requested.load() || stationary_speeds.size() < 10 || satellites < 6 ||
        hdop <= 0.0 || hdop > 3.0) {
        ::close(serial);
        throw std::runtime_error("GPS preflight failed");
    }
    const double noise_speed_mps = median(stationary_speeds);
    std::cout << "PREFLIGHT_OK sat=" << satellites << " hdop=" << hdop
              << " static_speed_mps=" << noise_speed_mps << '\n' << std::flush;

    vehicle::PigpioVehicle car;
    car.initialize(72);
    std::cout << "ESC_ARMING neutral_pwm=10000 wait_s=5\n" << std::flush;
    for (int tenth = 0; tenth < 50 && !stop_requested.load(); ++tenth) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (stop_requested.load()) {
        car.stop();
        ::close(serial);
        std::cout << "MOTOR_STOP reason=signal_before_start\n";
        return 3;
    }
    car.drive_forward(11100);

    constexpr double stop_command_distance_m = 2.70;
    constexpr double hard_timeout_s = 10.0;
    const auto drive_start = std::chrono::steady_clock::now();
    double distance_m = 0.0;
    double filtered_speed_mps = 0.0;
    double last_measurement_s = -1.0;
    double last_valid_gps_s = 0.0;
    double last_log_s = -1.0;
    std::vector<double> speed_window;
    std::string stop_reason = "hard timeout";
    std::cout << "MOTOR_START pwm=11100 steering=72 stop_at_m="
              << stop_command_distance_m << '\n' << std::flush;

    while (!stop_requested.load()) {
        const double elapsed = now_s(drive_start);
        if (elapsed >= hard_timeout_s) {
            break;
        }
        read_available(serial, parser, [&](const std::uint8_t type,
                                           const speed::WitSensorData& data) {
            if (type == 0x5A && data.satellites && data.hdop) {
                satellites = *data.satellites;
                hdop = *data.hdop;
            }
            if (type != 0x58 || !data.gps_speed_mps || satellites < 6 ||
                hdop <= 0.0 || hdop > 3.0 ||
                elapsed - last_valid_gps_s < 0.18) {
                return;
            }
            speed_window.push_back(*data.gps_speed_mps);
            if (speed_window.size() > 5) {
                speed_window.erase(speed_window.begin());
            }
            const double raw_filtered = median(speed_window);
            const double corrected = std::sqrt(std::max(
                0.0, raw_filtered * raw_filtered - noise_speed_mps * noise_speed_mps));
            filtered_speed_mps = last_measurement_s < 0.0
                                     ? corrected
                                     : 0.65 * filtered_speed_mps + 0.35 * corrected;
            if (last_measurement_s >= 0.0) {
                const double dt = elapsed - last_measurement_s;
                if (dt > 0.0 && dt < 0.6) {
                    distance_m += filtered_speed_mps * dt;
                }
            }
            last_measurement_s = elapsed;
            last_valid_gps_s = elapsed;
        });
        if (elapsed - last_valid_gps_s > 2.5) {
            stop_reason = "GPS timeout";
            break;
        }
        if (distance_m >= stop_command_distance_m) {
            stop_reason = "distance threshold";
            break;
        }
        if (elapsed - last_log_s >= 0.5) {
            std::cout << std::fixed << std::setprecision(3)
                      << "t=" << elapsed << " speed=" << filtered_speed_mps
                      << " distance=" << distance_m << " sat=" << satellites
                      << " hdop=" << hdop << '\n' << std::flush;
            last_log_s = elapsed;
        }
    }

    if (stop_requested.load()) {
        stop_reason = "signal";
    }
    car.stop();
    ::close(serial);
    std::cout << std::fixed << std::setprecision(3)
              << "MOTOR_STOP reason=" << stop_reason
              << " integrated_distance_m=" << distance_m << '\n' << std::flush;
    return stop_reason == "distance threshold" ? 0 : 3;
} catch (const std::exception& error) {
    std::cerr << "STOP error=" << error.what() << '\n';
    return 1;
}
