#include "xtnetrc_speed/speed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace xtnetrc::speed {
namespace {

[[nodiscard]] double clamp(const double value, const double minimum,
                           const double maximum) noexcept {
    return std::max(minimum, std::min(value, maximum));
}

}  // namespace

void SpeedEstimatorConfig::validate() const {
    if (minimum_satellites <= 0 || gps_median_window == 0) {
        throw std::invalid_argument("GPS satellite and window limits must be positive");
    }
    const double values[]{maximum_hdop, gps_timeout_s, gps_min_update_interval_s,
                          gps_correction_gain, acceleration_deadband_mps2,
                          maximum_speed_mps};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("speed estimator configuration must be finite");
        }
    }
    if (maximum_hdop <= 0.0 || gps_timeout_s <= 0.0 ||
        gps_min_update_interval_s < 0.0 || gps_correction_gain <= 0.0 ||
        gps_correction_gain > 1.0 || acceleration_deadband_mps2 < 0.0 ||
        maximum_speed_mps <= 0.0) {
        throw std::invalid_argument("invalid speed estimator configuration range");
    }
}

GpsImuSpeedEstimator::GpsImuSpeedEstimator(SpeedEstimatorConfig config)
    : config_(std::move(config)) {
    config_.validate();
}

void GpsImuSpeedEstimator::reset() noexcept {
    gps_window_.clear();
    speed_mps_ = 0.0;
    last_acceleration_time_s_ = 0.0;
    last_gps_time_s_ = 0.0;
    satellites_ = 0;
    hdop_ = 0.0;
    acceleration_initialized_ = false;
    gps_initialized_ = false;
}

void GpsImuSpeedEstimator::set_acceleration_bias(const double bias_mps2) {
    if (!std::isfinite(bias_mps2)) {
        throw std::invalid_argument("acceleration bias must be finite");
    }
    acceleration_bias_mps2_ = bias_mps2;
}

void GpsImuSpeedEstimator::update_acceleration(const double forward_acceleration_mps2,
                                               const double now_s) {
    if (!std::isfinite(forward_acceleration_mps2) || !std::isfinite(now_s)) {
        return;
    }
    if (!acceleration_initialized_) {
        acceleration_initialized_ = true;
        last_acceleration_time_s_ = now_s;
        return;
    }
    const double dt_s = now_s - last_acceleration_time_s_;
    last_acceleration_time_s_ = now_s;
    if (dt_s <= 0.0 || dt_s > 0.5 || !gps_initialized_) {
        return;
    }

    double acceleration = forward_acceleration_mps2 - acceleration_bias_mps2_;
    if (std::abs(acceleration) < config_.acceleration_deadband_mps2) {
        acceleration = 0.0;
    }
    speed_mps_ = clamp(speed_mps_ + acceleration * dt_s, 0.0,
                       config_.maximum_speed_mps);
}

bool GpsImuSpeedEstimator::update_gps(const double gps_speed_mps,
                                      const int satellites, const double hdop,
                                      const double now_s) {
    if (!std::isfinite(gps_speed_mps) || !std::isfinite(hdop) ||
        !std::isfinite(now_s) || gps_speed_mps < 0.0 ||
        gps_speed_mps > config_.maximum_speed_mps ||
        satellites < config_.minimum_satellites || hdop <= 0.0 ||
        hdop > config_.maximum_hdop) {
        return false;
    }
    if (gps_initialized_ && now_s - last_gps_time_s_ < config_.gps_min_update_interval_s) {
        return false;
    }

    gps_window_.push_back(gps_speed_mps);
    while (gps_window_.size() > config_.gps_median_window) {
        gps_window_.pop_front();
    }
    const double filtered_gps = median_gps_speed();
    if (!gps_initialized_) {
        speed_mps_ = filtered_gps;
        gps_initialized_ = true;
    } else {
        speed_mps_ += config_.gps_correction_gain * (filtered_gps - speed_mps_);
    }
    speed_mps_ = clamp(speed_mps_, 0.0, config_.maximum_speed_mps);
    last_gps_time_s_ = now_s;
    satellites_ = satellites;
    hdop_ = hdop;
    return true;
}

SpeedEstimate GpsImuSpeedEstimator::estimate(const double now_s) const noexcept {
    if (!gps_initialized_) {
        return {};
    }
    if (!std::isfinite(now_s) || now_s < last_gps_time_s_) {
        return {false, 0.0, 0.0, satellites_, hdop_, "invalid clock"};
    }
    const double age = now_s - last_gps_time_s_;
    if (age > config_.gps_timeout_s) {
        return {false, 0.0, age, satellites_, hdop_, "GPS timeout"};
    }
    return {true, speed_mps_, age, satellites_, hdop_, "GPS/IMU estimate valid"};
}

double GpsImuSpeedEstimator::median_gps_speed() const {
    std::vector<double> sorted(gps_window_.begin(), gps_window_.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t middle = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
        return (sorted[middle - 1] + sorted[middle]) / 2.0;
    }
    return sorted[middle];
}

void SpeedPiConfig::validate() const {
    const double values[]{kp_pwm_per_mps, ki_pwm_per_m, maximum_trim_pwm,
                          integral_limit_m};
    for (const double value : values) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument("PI speed controller configuration must be finite and nonnegative");
        }
    }
    if (maximum_trim_pwm <= 0.0 || integral_limit_m <= 0.0) {
        throw std::invalid_argument("PI trim and integral limits must be positive");
    }
}

SpeedPiController::SpeedPiController(SpeedPiConfig config) : config_(std::move(config)) {
    config_.validate();
}

void SpeedPiController::reset() noexcept {
    integral_error_m_ = 0.0;
}

SpeedLoopOutput SpeedPiController::update(const double target_speed_mps,
                                          const SpeedEstimate& estimate,
                                          const double dt_s) {
    if (!std::isfinite(target_speed_mps) || !std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("target speed and dt must be finite; dt must be positive");
    }
    if (target_speed_mps <= 0.0 || !estimate.valid ||
        !std::isfinite(estimate.speed_mps)) {
        reset();
        return {};
    }

    const double error = target_speed_mps - estimate.speed_mps;
    const double candidate_integral = clamp(integral_error_m_ + error * dt_s,
                                            -config_.integral_limit_m,
                                            config_.integral_limit_m);
    const double unclamped = config_.kp_pwm_per_mps * error +
                             config_.ki_pwm_per_m * candidate_integral;
    const double trim = clamp(unclamped, -config_.maximum_trim_pwm,
                              config_.maximum_trim_pwm);

    // 条件积分：输出已饱和且误差还在把它推向饱和时，不继续累积。
    const bool pushing_upper = unclamped > config_.maximum_trim_pwm && error > 0.0;
    const bool pushing_lower = unclamped < -config_.maximum_trim_pwm && error < 0.0;
    if (!pushing_upper && !pushing_lower) {
        integral_error_m_ = candidate_integral;
    }
    return {true, trim, error};
}

}  // namespace xtnetrc::speed
