#pragma once

#include <cstddef>
#include <deque>
#include <string_view>

namespace xtnetrc::speed {

struct SpeedEstimatorConfig {
    int minimum_satellites{6};
    double maximum_hdop{3.0};
    double gps_timeout_s{2.5};
    double gps_min_update_interval_s{0.18};
    std::size_t gps_median_window{5};
    double gps_correction_gain{0.35};
    double acceleration_deadband_mps2{0.03};
    double maximum_speed_mps{3.0};

    void validate() const;
};

struct SpeedEstimate {
    bool valid{false};
    double speed_mps{0.0};
    double gps_age_s{0.0};
    int satellites{0};
    double hdop{0.0};
    std::string_view reason{"waiting for GPS"};
};

// GPS 提供无漂移的慢速校正，车辆前向加速度只负责两次 GPS 更新之间的短时预测。
class GpsImuSpeedEstimator {
public:
    explicit GpsImuSpeedEstimator(SpeedEstimatorConfig config = {});

    void reset() noexcept;
    void set_acceleration_bias(double bias_mps2);
    void update_acceleration(double forward_acceleration_mps2, double now_s);

    // 返回 true 表示本次 GPS 测量通过质量门限并被接收。
    [[nodiscard]] bool update_gps(double gps_speed_mps, int satellites,
                                  double hdop, double now_s);
    [[nodiscard]] SpeedEstimate estimate(double now_s) const noexcept;

private:
    [[nodiscard]] double median_gps_speed() const;

    SpeedEstimatorConfig config_;
    std::deque<double> gps_window_;
    double speed_mps_{0.0};
    double acceleration_bias_mps2_{0.0};
    double last_acceleration_time_s_{0.0};
    double last_gps_time_s_{0.0};
    int satellites_{0};
    double hdop_{0.0};
    bool acceleration_initialized_{false};
    bool gps_initialized_{false};
};

struct SpeedPiConfig {
    // 输出是叠加到已标定前馈 PWM 上的修正量，不包含电调中位和方向。
    double kp_pwm_per_mps{12.0};
    double ki_pwm_per_m{3.0};
    double maximum_trim_pwm{8.0};
    double integral_limit_m{1.5};

    void validate() const;
};

struct SpeedLoopOutput {
    bool enabled{false};
    double pwm_trim{0.0};
    double speed_error_mps{0.0};
};

class SpeedPiController {
public:
    explicit SpeedPiController(SpeedPiConfig config = {});

    void reset() noexcept;
    [[nodiscard]] SpeedLoopOutput update(double target_speed_mps,
                                         const SpeedEstimate& estimate,
                                         double dt_s);

private:
    SpeedPiConfig config_;
    double integral_error_m_{0.0};
};

}  // namespace xtnetrc::speed
