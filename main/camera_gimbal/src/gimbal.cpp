#include "xtnetrc_gimbal/gimbal.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace xtnetrc::gimbal {

namespace {

void validate_axis(const char* name, const int minimum, const int center,
                   const int maximum, const int pwm_range) {
    if (minimum <= 0 || minimum >= center || center >= maximum ||
        maximum >= pwm_range) {
        throw std::invalid_argument(std::string(name) +
                                    " limits must satisfy 0 < min < center < max < range");
    }
}

}  // namespace

void GimbalConfig::validate() const {
    // 本模块只允许实车确认的 GPIO 22/23。即使配置文件被误改，也不能让
    // 云台模块向车辆转向 GPIO 12 或电调 GPIO 13 输出任何信号。
    if (pan_gpio_bcm != 22 || tilt_gpio_bcm != 23) {
        throw std::invalid_argument(
            "XT-NetRC camera gimbal requires BCM GPIO 22 for pan and 23 for tilt");
    }

    // 当前 PWM 数值是按这组频率和范围实车测得，改变任一项都会改变脉宽。
    if (pwm_frequency_hz != 50 || pwm_range != 1000) {
        throw std::invalid_argument(
            "XT-NetRC gimbal calibration requires 50 Hz and PWM range 1000");
    }

    validate_axis("pan", pan_min_pwm, pan_center_pwm, pan_max_pwm, pwm_range);
    validate_axis("tilt", tilt_min_pwm, tilt_center_pwm, tilt_max_pwm, pwm_range);

    // 代码层的绝对安全包络不能被配置文件放宽。配置可以选择更小范围，
    // 但绝不能越过 2026-08-31 实车验证后保留余量的边界。
    if (pan_min_pwm < 28 || pan_max_pwm > 122 ||
        tilt_min_pwm < 58 || tilt_max_pwm > 122) {
        throw std::invalid_argument(
            "gimbal limits exceed the XT-NetRC measured software-safe envelope");
    }

    if (max_step_pwm < 1 || max_step_pwm > 10) {
        throw std::invalid_argument("max_step_pwm must be between 1 and 10");
    }
    if (step_interval_ms < 20 || step_interval_ms > 1000) {
        throw std::invalid_argument("step_interval_ms must be between 20 and 1000");
    }
}

GimbalController::GimbalController(PwmDriver& driver, GimbalConfig config,
                                   SleepFunction sleeper)
    : driver_(driver), config_(config), sleeper_(std::move(sleeper)) {
    config_.validate();
    if (!sleeper_) {
        sleeper_ = [](const std::chrono::milliseconds duration) {
            std::this_thread::sleep_for(duration);
        };
    }
    current_ = {config_.pan_center_pwm, config_.tilt_center_pwm};
}

GimbalController::~GimbalController() {
    release();
}

void GimbalController::initialize() {
    if (initialized_) {
        return;
    }

    try {
        driver_.initialize();
        driver_.configure(config_.pan_gpio_bcm, config_.pwm_frequency_hz,
                          config_.pwm_range);
        driver_.configure(config_.tilt_gpio_bcm, config_.pwm_frequency_hz,
                          config_.pwm_range);

        // 启动时的物理位置未知，不能假设当前位置后做插值；先直接写入经过
        // 实车验证的中心脉宽，之后所有动作再从中心小步移动。
        driver_.write(config_.pan_gpio_bcm, config_.pan_center_pwm);
        driver_.write(config_.tilt_gpio_bcm, config_.tilt_center_pwm);
        current_ = {config_.pan_center_pwm, config_.tilt_center_pwm};
        initialized_ = true;
    } catch (...) {
        // 即使第二个引脚配置失败，也应立即释放第一个引脚和 pigpio。
        try {
            driver_.write(config_.pan_gpio_bcm, 0);
            driver_.write(config_.tilt_gpio_bcm, 0);
        } catch (...) {
        }
        driver_.shutdown();
        throw;
    }
}

GimbalPosition GimbalController::clamp_position(const int pan_pwm,
                                                const int tilt_pwm) const noexcept {
    return {
        std::clamp(pan_pwm, config_.pan_min_pwm, config_.pan_max_pwm),
        std::clamp(tilt_pwm, config_.tilt_min_pwm, config_.tilt_max_pwm),
    };
}

int GimbalController::approach(const int current, const int target,
                               const int max_step) noexcept {
    if (current < target) {
        return std::min(current + max_step, target);
    }
    if (current > target) {
        return std::max(current - max_step, target);
    }
    return current;
}

void GimbalController::require_initialized() const {
    if (!initialized_) {
        throw std::logic_error("gimbal controller is not initialized");
    }
}

GimbalPosition GimbalController::move_to(const int pan_pwm, const int tilt_pwm) {
    require_initialized();
    const GimbalPosition target = clamp_position(pan_pwm, tilt_pwm);

    while (current_.pan_pwm != target.pan_pwm ||
           current_.tilt_pwm != target.tilt_pwm) {
        const GimbalPosition next{
            approach(current_.pan_pwm, target.pan_pwm, config_.max_step_pwm),
            approach(current_.tilt_pwm, target.tilt_pwm, config_.max_step_pwm),
        };

        // 只写发生变化的轴；无论调用方给多大的数值，写入值都已经过限位。
        if (next.pan_pwm != current_.pan_pwm) {
            driver_.write(config_.pan_gpio_bcm, next.pan_pwm);
        }
        if (next.tilt_pwm != current_.tilt_pwm) {
            driver_.write(config_.tilt_gpio_bcm, next.tilt_pwm);
        }
        current_ = next;
        sleeper_(std::chrono::milliseconds(config_.step_interval_ms));
    }
    return target;
}

GimbalPosition GimbalController::move_relative(const int pan_delta_pwm,
                                               const int tilt_delta_pwm) {
    require_initialized();
    // 先提升到 long long，避免调用方传入 INT_MAX/INT_MIN 时发生有符号溢出。
    const long long requested_pan =
        static_cast<long long>(current_.pan_pwm) + pan_delta_pwm;
    const long long requested_tilt =
        static_cast<long long>(current_.tilt_pwm) + tilt_delta_pwm;
    const int safe_pan = static_cast<int>(std::clamp(
        requested_pan, static_cast<long long>(config_.pan_min_pwm),
        static_cast<long long>(config_.pan_max_pwm)));
    const int safe_tilt = static_cast<int>(std::clamp(
        requested_tilt, static_cast<long long>(config_.tilt_min_pwm),
        static_cast<long long>(config_.tilt_max_pwm)));
    return move_to(safe_pan, safe_tilt);
}

GimbalPosition GimbalController::center() {
    return move_to(config_.pan_center_pwm, config_.tilt_center_pwm);
}

GimbalPosition GimbalController::current_position() const {
    require_initialized();
    return current_;
}

void GimbalController::release() noexcept {
    if (!initialized_) {
        return;
    }

    // duty=0 仅停止本模块占用的云台 PWM，不会触碰转向和电调 GPIO 12/13。
    try {
        driver_.write(config_.pan_gpio_bcm, 0);
    } catch (...) {
    }
    try {
        driver_.write(config_.tilt_gpio_bcm, 0);
    } catch (...) {
    }
    driver_.shutdown();
    initialized_ = false;
}

}  // namespace xtnetrc::gimbal
