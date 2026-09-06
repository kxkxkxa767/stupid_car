#pragma once
#include "xtnetrc_hardware/vehicle.hpp"
#include "xtnetrc_motion/controller.hpp"
#include "xtnetrc_speed/speed_loop.hpp"
#include "xtnetrc_speed/wit_sensor.hpp"
#include <cstdint>
#include <string>

namespace xtnetrc::runtime {
struct RuntimeConfig {
    hardware::VehicleProfile board;
    motion::MotionConfig motion;
    speed::SpeedPiConfig pi;
    std::string calibration_path, serial_device;
    int serial_baud{9600};
    double target_speed_mps{0.2};
    double maximum_run_s{5.0};
    double maximum_distance_m{3.0};
    double lane_timeout_s{0.25};
    double gps_timeout_s{1.0};
    double speed_pi_period_s{1.0};
    double steering_pwm_per_rad{122.45};
    double feedforward_speed_mps{0.2};
    double feedforward_pwm{11100};
    bool actuator_calibration_verified{false};
    bool ground_calibration_verified{false};
    void validate() const;
};
RuntimeConfig load_runtime_config(const std::string& path);

// Sensor interfaces carry SI values and monotonic timestamps, not OpenCV or GPIO.
struct LaneObservation {
    std::uint64_t sequence{0};
    double captured_s{0};
    bool valid{false};
    double reference_x_m{1.10}, center_y_m{0}, heading_slope{0}, width_m{0}, confidence{0};
};
struct RuntimeOutput {
    hardware::PwmCommand pwm;
    motion::ControlCommand motion;
    speed::SpeedLoopOutput speed;
    bool drive{false};
    double distance_m{0};
    std::string reason{"disabled"};
};

class ControlPipeline {
public:
    explicit ControlPipeline(RuntimeConfig config);
    RuntimeOutput update(const LaneObservation& lane, const speed::SpeedEstimate& velocity,
                         double now_s, double dt_s, bool enabled);
    void reset();
private:
    RuntimeConfig config_;
    motion::MotionController motion_;
    speed::SpeedPiController speed_;
    motion::ControlCommand last_motion_;
    std::uint64_t last_sequence_{0};
    double last_lane_s_{0}, distance_m_{0}, started_s_{-1};
    double last_pi_s_{-1};
    speed::SpeedLoopOutput last_speed_;
    bool fault_latched_{false};
};

// Only accepts a speed packet when a recent quality packet is also present.
// IMU prediction is deliberately not enabled until axis/bias are calibrated.
class GpsInput {
public:
    explicit GpsInput(double timeout_s);
    void accept(std::uint8_t type, const speed::WitSensorData& data, double now_s);
    speed::SpeedEstimate estimate(double now_s) const;
    std::size_t accepted_samples() const noexcept { return accepted_; }
private:
    speed::GpsImuSpeedEstimator estimator_;
    double timeout_s_, quality_time_s_{-1};
    int satellites_{0};
    double hdop_{99};
    std::size_t accepted_{0};
};
}  // namespace xtnetrc::runtime
