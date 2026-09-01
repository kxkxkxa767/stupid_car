#include "xtnetrc_gimbal/gimbal.hpp"
#include "xtnetrc_gimbal/pigpio_driver.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace gimbal = xtnetrc::gimbal;

namespace {

[[noreturn]] void usage(const char* program) {
    std::cerr
        << "用法:\n"
        << "  " << program << " --config 文件 center [--hold-ms 500]\n"
        << "  " << program
        << " --config 文件 move PAN_PWM TILT_PWM [--hold-ms 500]\n\n"
        << "方向：PAN 减小向车辆右侧，增大向左侧；TILT 减小向下，增大向上。\n"
        << "命令会自动夹紧到安全范围，并且绝不会写 GPIO 12/13。\n";
    std::exit(2);
}

int parse_int(const std::string& text, const char* name) {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) {
        throw std::invalid_argument(std::string(name) + " must be an integer");
    }
    return value;
}

}  // namespace

int main(const int argc, char** argv) {
    if (argc < 4 || std::string(argv[1]) != "--config") {
        usage(argv[0]);
    }

    try {
        const std::string config_path = argv[2];
        const std::string command = argv[3];
        int argument_index = 4;
        int requested_pan = 0;
        int requested_tilt = 0;

        const gimbal::GimbalConfig config =
            gimbal::load_gimbal_config_json(config_path);
        if (command == "center") {
            requested_pan = config.pan_center_pwm;
            requested_tilt = config.tilt_center_pwm;
        } else if (command == "move") {
            if (argc < 6) {
                usage(argv[0]);
            }
            requested_pan = parse_int(argv[4], "PAN_PWM");
            requested_tilt = parse_int(argv[5], "TILT_PWM");
            argument_index = 6;
        } else {
            usage(argv[0]);
        }

        int hold_ms = 500;
        if (argument_index < argc) {
            if (std::string(argv[argument_index]) != "--hold-ms" ||
                argument_index + 1 >= argc || argument_index + 2 != argc) {
                usage(argv[0]);
            }
            hold_ms = parse_int(argv[argument_index + 1], "hold-ms");
            if (hold_ms < 0 || hold_ms > 60000) {
                throw std::invalid_argument("hold-ms must be between 0 and 60000");
            }
        }

        gimbal::PigpioDriver driver;
        gimbal::GimbalController controller(driver, config);
        controller.initialize();
        const gimbal::GimbalPosition applied =
            controller.move_to(requested_pan, requested_tilt);

        std::cout << "云台位置：X=" << applied.pan_pwm
                  << "，Y=" << applied.tilt_pwm << '\n';
        if (applied.pan_pwm != requested_pan ||
            applied.tilt_pwm != requested_tilt) {
            std::cout << "请求超出安全范围，已自动限位。\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
        controller.release();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "云台控制失败：" << error.what() << '\n';
        return 1;
    }
}
