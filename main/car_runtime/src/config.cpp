#include "xtnetrc_runtime/control.hpp"
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <opencv2/core.hpp>

namespace xtnetrc::runtime {
namespace {
class Object {
public:
    explicit Object(cv::FileNode node) : node_(node) {
        if (!node_.isMap()) throw std::invalid_argument("runtime config requires JSON objects");
        for (auto item : node_) if (!keys_.insert(item.name()).second)
            throw std::invalid_argument("duplicate config key: " + item.name());
    }
    cv::FileNode take(const char* key) {
        if (!keys_.erase(key)) throw std::invalid_argument(std::string("missing config key: ") + key);
        return node_[key];
    }
    double number(const char* key) {
        const auto node = take(key);
        if (!node.isInt() && !node.isReal()) throw std::invalid_argument(std::string("expected numeric config: ") + key);
        const double value = static_cast<double>(node);
        if (!std::isfinite(value)) throw std::invalid_argument("non-finite config");
        return value;
    }
    int integer(const char* key) {
        const double value = number(key);
        if (std::floor(value) != value || std::abs(value) > 1000000) throw std::invalid_argument("invalid config integer");
        return static_cast<int>(value);
    }
    bool flag(const char* key) {
        const int value = integer(key);
        if (value != 0 && value != 1) throw std::invalid_argument("config flags must be 0/1");
        return value == 1;
    }
    std::string text(const char* key) {
        const auto node = take(key);
        if (!node.isString() || static_cast<std::string>(node).empty()) throw std::invalid_argument("config string missing/empty");
        return static_cast<std::string>(node);
    }
    void finish() const {
        if (!keys_.empty()) throw std::invalid_argument("unknown config key: " + *keys_.begin());
    }
private:
    cv::FileNode node_;
    std::set<std::string> keys_;
};
}
RuntimeConfig load_runtime_config(const std::string& path) {
    cv::FileStorage file(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    if (!file.isOpened()) throw std::runtime_error("cannot open runtime configuration: " + path);
    Object root(file.root());
    Object board(root.take("board")), control(root.take("control")), files(root.take("files"));
    RuntimeConfig config;
    if (board.text("driver") != "raspberry_pi_pigpio") throw std::invalid_argument("unknown board driver");
#define READ_BOARD(name) config.board.name = board.integer(#name)
    READ_BOARD(steering_gpio); READ_BOARD(motor_gpio);
    READ_BOARD(steering_hz); READ_BOARD(steering_range);
    READ_BOARD(motor_hz); READ_BOARD(motor_range);
    READ_BOARD(steering_min); READ_BOARD(steering_center); READ_BOARD(steering_max);
    READ_BOARD(motor_neutral); READ_BOARD(motor_min); READ_BOARD(motor_max);
#undef READ_BOARD
    config.serial_device = board.text("serial_device");
    config.serial_baud = board.integer("serial_baud");
#define READ_CONTROL(name) config.name = control.number(#name)
    READ_CONTROL(target_speed_mps); READ_CONTROL(maximum_run_s); READ_CONTROL(maximum_distance_m);
    READ_CONTROL(lane_timeout_s); READ_CONTROL(gps_timeout_s); READ_CONTROL(steering_pwm_per_rad);
    READ_CONTROL(speed_pi_period_s);
    READ_CONTROL(feedforward_speed_mps); READ_CONTROL(feedforward_pwm);
#undef READ_CONTROL
    config.actuator_calibration_verified = control.flag("actuator_calibration_verified");
    config.ground_calibration_verified = control.flag("ground_calibration_verified");
    Object pi(control.take("speed_pi"));
    config.pi.kp_pwm_per_mps = pi.number("kp_pwm_per_mps");
    config.pi.ki_pwm_per_m = pi.number("ki_pwm_per_m");
    config.pi.maximum_trim_pwm = pi.number("maximum_trim_pwm");
    config.pi.integral_limit_m = pi.number("integral_limit_m");
    const auto base = std::filesystem::absolute(path).parent_path();
    config.calibration_path = (base / files.text("ground_projection")).lexically_normal().string();
    config.motion = motion::load_motion_config_json((base / files.text("motion")).lexically_normal().string());
    root.finish(); board.finish(); control.finish(); files.finish(); pi.finish();
    config.validate();
    return config;
}
}  // namespace xtnetrc::runtime
