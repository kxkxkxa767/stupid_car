#include "xtnetrc_runtime/control.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace xtnetrc::runtime {
void RuntimeConfig::validate() const {
    board.validate(); motion.validate(); pi.validate();
    const double positive[]{target_speed_mps, maximum_run_s, maximum_distance_m,
        lane_timeout_s, gps_timeout_s, feedforward_speed_mps, speed_pi_period_s};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("runtime limits must be positive and finite");
    if (!std::isfinite(steering_pwm_per_rad) || std::abs(steering_pwm_per_rad) < 1.0 ||
        std::abs(steering_pwm_per_rad) > 1000 || !std::isfinite(feedforward_pwm) ||
        feedforward_pwm < board.motor_min || feedforward_pwm > board.motor_max ||
        target_speed_mps > 0.3 || maximum_run_s > 10 || maximum_distance_m > 3 ||
        lane_timeout_s > 0.25 || gps_timeout_s > 2.5 || speed_pi_period_s < 0.2 || speed_pi_period_s > 2 ||
        feedforward_speed_mps < 0.01 || feedforward_speed_mps > 3 || serial_device.empty())
        throw std::invalid_argument("runtime configuration exceeds guarded test envelope");
}

ControlPipeline::ControlPipeline(RuntimeConfig config)
    : config_(std::move(config)), motion_(config_.motion), speed_(config_.pi) {
    config_.validate(); reset();
}

void ControlPipeline::reset() {
    motion_.reset(); speed_.reset(); last_motion_ = {};
    static_cast<void>(motion_.update({}, 0, std::nullopt, false));
    last_sequence_ = 0; last_lane_s_ = 0; distance_m_ = 0; started_s_ = -1; fault_latched_ = false;
    last_pi_s_ = -1; last_speed_ = {};
}

RuntimeOutput ControlPipeline::update(const LaneObservation& lane,
                                      const speed::SpeedEstimate& velocity,
                                      double now_s, double dt_s, bool enabled) {
    RuntimeOutput out;
    out.pwm = {config_.board.steering_center, config_.board.motor_neutral};
    out.distance_m = distance_m_;
    auto stop = [&](const char* reason) {
        motion_.reset(); speed_.reset(); last_motion_ = {};
        last_pi_s_ = -1; last_speed_ = {};
        static_cast<void>(motion_.update({}, 0, std::nullopt, false));
        if (started_s_ >= 0) fault_latched_ = true;
        out.reason = reason;
        return out;
    };
    if (!enabled) return stop("disabled");
    if (fault_latched_) return stop("safety stop latched; explicit new run required");
    if (!std::isfinite(now_s) || !std::isfinite(dt_s) || dt_s <= 0 || dt_s > 0.15)
        return stop("control clock invalid/late");
    const double geometry[]{lane.captured_s, lane.reference_x_m, lane.center_y_m,
        lane.heading_slope, lane.width_m, lane.confidence};
    for (double value : geometry) if (!std::isfinite(value)) return stop("non-finite lane input");
    if (!lane.valid || lane.sequence == 0 || lane.sequence < last_sequence_ ||
        lane.captured_s > now_s || now_s - lane.captured_s > config_.lane_timeout_s ||
        lane.width_m < 0.45 || lane.width_m > 1.10 || std::abs(lane.center_y_m) > 0.25 ||
        std::abs(lane.heading_slope) > 0.35 || lane.reference_x_m <= 0)
        return stop("lane invalid/stale/out of corridor");
    if (!velocity.valid || !std::isfinite(velocity.speed_mps) || velocity.speed_mps < 0 ||
        velocity.speed_mps > 3 || !std::isfinite(velocity.gps_age_s) || velocity.gps_age_s < 0 ||
        velocity.gps_age_s > config_.gps_timeout_s)
        return stop("GPS speed invalid/stale");
    if (velocity.speed_mps > config_.target_speed_mps + 0.15)
        return stop("measured speed exceeds guarded target margin");
    if (started_s_ >= 0) {
        if (now_s < started_s_ || now_s - started_s_ >= config_.maximum_run_s)
            return stop("run deadline");
        distance_m_ += velocity.speed_mps * dt_s;
        out.distance_m = distance_m_;
        if (distance_m_ >= config_.maximum_distance_m) return stop("distance threshold (GPS estimate)");
    }
    if (lane.sequence != last_sequence_) {
        if (last_sequence_ && lane.captured_s <= last_lane_s_) return stop("nonmonotonic frame timestamp");
        // center_y is defined at reference_x, not at the rear axle. Extrapolate
        // the observed local centerline explicitly; do not feed pixels to control.
        std::vector<motion::PathPoint> path;
        for (double x : {0.20, 0.45, 0.70, 1.10, 1.50, 2.0})
            path.push_back({x, lane.center_y_m + (x - lane.reference_x_m) * lane.heading_slope});
        const double frame_dt = last_sequence_ ? lane.captured_s - last_lane_s_ : dt_s;
        last_motion_ = motion_.update(path, lane.confidence, frame_dt, true);
        last_sequence_ = lane.sequence;
        last_lane_s_ = lane.captured_s;
    } else if (lane.captured_s != last_lane_s_) {
        return stop("frame sequence reused with changed timestamp");
    }
    out.motion = last_motion_;
    out.motion.target_speed_mps = std::min(out.motion.target_speed_mps, config_.target_speed_mps);
    if (out.motion.target_speed_mps <= 0) {
        last_speed_ = {}; last_pi_s_ = -1; speed_.reset();
    } else if (last_pi_s_ < 0 || now_s - last_pi_s_ >= config_.speed_pi_period_s) {
        const double pi_dt = last_pi_s_ < 0 ? dt_s : now_s - last_pi_s_;
        last_speed_ = speed_.update(out.motion.target_speed_mps, velocity, pi_dt);
        last_pi_s_ = now_s;
    }
    out.speed = last_speed_;
    if (!out.speed.enabled) {
        if (started_s_ >= 0) return stop("motion controller lost");
        out.reason = "waiting for motion confidence confirmation";
        return out;
    }
    const double steering = config_.board.steering_center +
        config_.steering_pwm_per_rad * out.motion.steering_rad;
    const double feedforward = config_.board.motor_neutral +
        (config_.feedforward_pwm - config_.board.motor_neutral) *
        out.motion.target_speed_mps / config_.feedforward_speed_mps;
    out.pwm.steering = static_cast<int>(std::lround(std::clamp(steering,
        static_cast<double>(config_.board.steering_min), static_cast<double>(config_.board.steering_max))));
    out.pwm.motor = static_cast<int>(std::lround(std::clamp(feedforward + out.speed.pwm_trim,
        static_cast<double>(config_.board.motor_min), static_cast<double>(config_.board.motor_max))));
    out.drive = true;
    if (started_s_ < 0) started_s_ = now_s;
    out.reason = "lane + motion + GPS PI";
    return out;
}

GpsInput::GpsInput(double timeout_s) : estimator_([&] {
    speed::SpeedEstimatorConfig config; config.gps_timeout_s = timeout_s; return config;
}()), timeout_s_(timeout_s) {}

void GpsInput::accept(std::uint8_t type, const speed::WitSensorData& data, double now_s) {
    if (!std::isfinite(now_s)) return;
    if (type == 0x5A && data.satellites && data.hdop) {
        satellites_ = *data.satellites; hdop_ = *data.hdop; quality_time_s_ = now_s;
    }
    if (type == 0x58 && data.gps_speed_mps && quality_time_s_ >= 0 &&
        now_s >= quality_time_s_ && now_s - quality_time_s_ <= timeout_s_ &&
        estimator_.update_gps(*data.gps_speed_mps, satellites_, hdop_, now_s)) ++accepted_;
}

speed::SpeedEstimate GpsInput::estimate(double now_s) const {
    if (quality_time_s_ < 0 || !std::isfinite(now_s) || now_s < quality_time_s_ ||
        now_s - quality_time_s_ > timeout_s_ || satellites_ < 6 || !std::isfinite(hdop_) ||
        hdop_ <= 0 || hdop_ > 3) return {};
    return estimator_.estimate(now_s);
}
}  // namespace xtnetrc::runtime
