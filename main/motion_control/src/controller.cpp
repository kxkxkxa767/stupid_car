#include "xtnetrc_motion/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xtnetrc::motion {
namespace {

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] double degrees_to_radians(const double degrees) noexcept {
    return degrees * kPi / 180.0;
}

[[nodiscard]] double radians_to_degrees(const double radians) noexcept {
    return radians * 180.0 / kPi;
}

[[nodiscard]] double clamp(const double value, const double minimum,
                           const double maximum) noexcept {
    return std::max(minimum, std::min(value, maximum));
}

}  // namespace

std::string_view to_string(const ControlState state) noexcept {
    switch (state) {
        case ControlState::Stopped:
            return "stopped";
        case ControlState::Tracking:
            return "tracking";
        case ControlState::Degraded:
            return "degraded";
        case ControlState::Lost:
            return "lost";
    }
    return "unknown";
}

void MotionConfig::validate() const {
    const double values[]{
        wheelbase_m,          lookahead_m,              max_steering_deg,
        max_steering_rate_deg_s, max_speed_mps,         min_curve_speed_mps,
        degraded_max_speed_mps, curvature_speed_gain,   confidence_stop,
        confidence_degraded, max_path_distance_m,       max_lateral_offset_m,
        max_accel_mps2,       max_decel_mps2,           confidence_hysteresis,
    };
    for (const double value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("all motion configuration values must be finite");
        }
    }

    // 这些参数为零或负数时没有物理意义，会导致除零或反向限幅。
    const bool positive_values =
        wheelbase_m > 0.0 && lookahead_m > 0.0 && max_steering_deg > 0.0 &&
        max_steering_rate_deg_s > 0.0 && max_speed_mps > 0.0 &&
        min_curve_speed_mps > 0.0 && degraded_max_speed_mps > 0.0 &&
        max_path_distance_m > 0.0 && max_lateral_offset_m > 0.0 &&
        max_accel_mps2 > 0.0 && max_decel_mps2 > 0.0;
    if (!positive_values) {
        throw std::invalid_argument("motion dimensions, limits and speeds must be positive");
    }
    if (max_steering_deg >= 90.0 || curvature_speed_gain < 0.0) {
        throw std::invalid_argument(
            "steering must be below 90 degrees and curvature gain cannot be negative");
    }
    if (!(0.0 <= confidence_stop && confidence_stop < confidence_degraded &&
          confidence_degraded <= 1.0)) {
        throw std::invalid_argument(
            "confidence thresholds must satisfy 0 <= stop < degraded <= 1");
    }
    if (min_curve_speed_mps > max_speed_mps ||
        degraded_max_speed_mps > max_speed_mps) {
        throw std::invalid_argument("limited speed cannot exceed maximum speed");
    }
    if (confidence_stop <= 0.0 || confidence_hysteresis < 0.0 ||
        confidence_degraded + confidence_hysteresis > 1.0 ||
        confidence_stop + confidence_hysteresis >=
            confidence_degraded - confidence_hysteresis) {
        throw std::invalid_argument("confidence hysteresis leaves no usable transition band");
    }
    if (confidence_reacquire_frames == 0) {
        throw std::invalid_argument("confidence_reacquire_frames must be positive");
    }
}

double ControlCommand::steering_deg() const noexcept {
    return radians_to_degrees(steering_rad);
}

MotionController::MotionController(MotionConfig config) : config_(std::move(config)) {
    config_.validate();
}

void MotionController::reset() noexcept {
    last_steering_rad_ = 0.0;
    last_speed_mps_ = 0.0;
    reacquisition_required_ = false;
    reacquire_count_ = 0;
    tracking_latched_ = false;
}

ControlCommand MotionController::update(const std::vector<PathPoint>& path_m,
                                        double confidence,
                                        const std::optional<double> dt_s,
                                        const bool enabled) {
    if (dt_s.has_value() && (!std::isfinite(*dt_s) || *dt_s <= 0.0)) {
        throw std::invalid_argument("dt_s must be a finite positive number");
    }

    // NaN 或无穷置信度视为完全不可信；普通越界值则截断到 0～1。
    confidence = std::isfinite(confidence) ? clamp(confidence, 0.0, 1.0) : 0.0;

    // 安全状态优先，不使用历史结果来“猜测”车辆应该继续怎样行驶。
    if (!enabled) {
        return stop(ControlState::Stopped, confidence, "controller disabled");
    }
    if (confidence < config_.confidence_stop) {
        return stop(ControlState::Lost, confidence, "confidence below stop threshold");
    }

    // 严格检查整条路径，不能悄悄丢弃坏点后继续驱动车辆。
    std::vector<PathPoint> forward_path;
    forward_path.reserve(path_m.size());
    double previous_distance = -1.0;
    for (const auto& point : path_m) {
        if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) || point.x_m < 0.0) {
            return stop(ControlState::Lost, confidence, "path contains invalid coordinates");
        }
        const double distance = std::hypot(point.x_m, point.y_m);
        if (!std::isfinite(distance) || distance > config_.max_path_distance_m ||
            std::abs(point.y_m) > config_.max_lateral_offset_m) {
            return stop(ControlState::Lost, confidence, "path exceeds configured bounds");
        }
        if (distance + 1e-9 < previous_distance) {
            return stop(ControlState::Lost, confidence, "path is not ordered near to far");
        }
        previous_distance = distance;
        if (distance > 1e-9) {
            forward_path.push_back(point);
        }
    }
    if (forward_path.empty()) {
        return stop(ControlState::Lost, confidence, "no forward path");
    }

    const PathPoint target = select_target(forward_path);
    const double target_distance = std::hypot(target.x_m, target.y_m);
    const double distance_squared = target_distance * target_distance;
    if (distance_squared <= 1e-9) {
        return stop(ControlState::Lost, confidence, "target is too close");
    }

    // Pure Pursuit：k = 2y/(x²+y²)，自行车模型：delta = atan(L*k)。
    const double curvature = 2.0 * target.y_m / distance_squared;
    if (!std::isfinite(curvature)) {
        return stop(ControlState::Lost, confidence, "computed curvature is invalid");
    }
    const double max_steering_rad = degrees_to_radians(config_.max_steering_deg);
    const double max_curvature = std::tan(max_steering_rad) / config_.wheelbase_m;
    if (std::abs(curvature) > max_curvature + 1e-9) {
        return stop(ControlState::Lost, confidence, "path curvature exceeds steering capability");
    }
    const double requested_steering = std::atan(config_.wheelbase_m * curvature);

    // 先限制机械转角，再按控制周期限制转角变化速度。
    const double bounded_steering =
        clamp(requested_steering, -max_steering_rad, max_steering_rad);
    const double steering = limit_steering_rate(bounded_steering, dt_s);

    // 发生过失线/关闭后，需要连续若干帧高于恢复阈值才允许重新起步。
    if (reacquisition_required_) {
        if (confidence < config_.confidence_stop + config_.confidence_hysteresis) {
            reacquire_count_ = 0;
            last_steering_rad_ = 0.0;
            last_speed_mps_ = 0.0;
            return {ControlState::Lost, 0.0, 0.0, 0.0, std::nullopt, confidence,
                    "waiting for confidence recovery"};
        }
        ++reacquire_count_;
        if (reacquire_count_ < config_.confidence_reacquire_frames) {
            last_steering_rad_ = 0.0;
            last_speed_mps_ = 0.0;
            return {ControlState::Lost, 0.0, 0.0, 0.0, std::nullopt, confidence,
                    "confirming recovered path"};
        }
        reacquisition_required_ = false;
        reacquire_count_ = 0;
    }

    // 弯道越急，按 1/(1+gain*|k|) 平滑降速，再限制在设定范围内。
    double speed = config_.max_speed_mps /
                   (1.0 + config_.curvature_speed_gain * std::abs(curvature));
    speed = clamp(speed, config_.min_curve_speed_mps, config_.max_speed_mps);

    if (tracking_latched_) {
        tracking_latched_ =
            confidence >= config_.confidence_degraded - config_.confidence_hysteresis;
    } else {
        tracking_latched_ =
            confidence >= config_.confidence_degraded + config_.confidence_hysteresis;
    }

    ControlState state = tracking_latched_ ? ControlState::Tracking
                                           : ControlState::Degraded;
    std::string_view reason = tracking_latched_ ? "tracking"
                                                : "tracking with reduced confidence";
    if (state == ControlState::Degraded) {
        speed = std::min(speed, config_.degraded_max_speed_mps);
    }

    speed = limit_speed_rate(speed, dt_s);
    if (state == ControlState::Degraded) {
        speed = std::min(speed, config_.degraded_max_speed_mps);
    }

    last_steering_rad_ = steering;
    last_speed_mps_ = speed;
    return {state, steering, curvature, speed, target, confidence, reason};
}

PathPoint MotionController::select_target(const std::vector<PathPoint>& path_m) const {
    // 在路径线段与前视圆的交点处插值，避免点密度改变转向结果。
    PathPoint previous{0.0, 0.0};
    for (const auto& current : path_m) {
        if (std::hypot(current.x_m, current.y_m) >= config_.lookahead_m) {
            const double dx = current.x_m - previous.x_m;
            const double dy = current.y_m - previous.y_m;
            const double a = dx * dx + dy * dy;
            const double b = 2.0 * (previous.x_m * dx + previous.y_m * dy);
            const double c = previous.x_m * previous.x_m +
                             previous.y_m * previous.y_m -
                             config_.lookahead_m * config_.lookahead_m;
            const double discriminant = std::max(0.0, b * b - 4.0 * a * c);
            const double root = (-b + std::sqrt(discriminant)) / (2.0 * a);
            const double t = clamp(root, 0.0, 1.0);
            return {previous.x_m + t * dx, previous.y_m + t * dy};
        }
        previous = current;
    }

    return path_m.back();
}

double MotionController::limit_steering_rate(
    const double requested_rad, const std::optional<double> dt_s) const {
    // 不传周期时只应用绝对转角限制，便于离线演示和算法测试。
    if (!dt_s.has_value()) {
        return requested_rad;
    }
    // 本周期允许的最大变化量 = 最大转角速度 × 周期时长。
    const double max_change_rad =
        degrees_to_radians(config_.max_steering_rate_deg_s) * *dt_s;
    return clamp(requested_rad, last_steering_rad_ - max_change_rad,
                 last_steering_rad_ + max_change_rad);
}

double MotionController::limit_speed_rate(
    const double requested_mps, const std::optional<double> dt_s) const noexcept {
    if (!dt_s.has_value()) {
        return requested_mps;
    }
    return clamp(requested_mps,
                 std::max(0.0, last_speed_mps_ - config_.max_decel_mps2 * *dt_s),
                 last_speed_mps_ + config_.max_accel_mps2 * *dt_s);
}

ControlCommand MotionController::stop(const ControlState state, const double confidence,
                                      const std::string_view reason) noexcept {
    // 清除旧转角，避免路径恢复后沿用丢失前的转向命令。
    last_steering_rad_ = 0.0;
    last_speed_mps_ = 0.0;
    reacquisition_required_ = true;
    reacquire_count_ = 0;
    tracking_latched_ = false;
    return {state, 0.0, 0.0, 0.0, std::nullopt, confidence, reason};
}

}  // namespace xtnetrc::motion
