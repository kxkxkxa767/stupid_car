#pragma once

#include <optional>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace xtnetrc::motion {

// 车辆坐标系中的路径点：x 向前、y 向左，单位均为米。
struct PathPoint {
    double x_m{0.0};
    double y_m{0.0};
};

// 控制器状态。未来硬件层只能在 Tracking 或 Degraded 状态下接受速度目标。
enum class ControlState {
    Stopped,   // 人工关闭或程序尚未启动。
    Tracking,  // 路径可信，可以正常跟踪。
    Degraded,  // 路径置信度偏低，只允许低速跟踪。
    Lost       // 路径丢失或无效，目标速度必须为零。
};

[[nodiscard]] std::string_view to_string(ControlState state) noexcept;

// Pure Pursuit、速度规划和安全策略的参数，统一使用 SI 单位。
struct MotionConfig {
    // 实测轴距，是曲率换算为前轮转角时的自行车模型参数。
    double wheelbase_m{0.23861};

    // 前视距离越大越平稳，越小则对近处弯道响应越快。
    double lookahead_m{0.45};

    // 最大前轮转角目前是保守初值，必须通过实车标定后再调整。
    double max_steering_deg{25.0};

    // 限制相邻控制周期的转角变化，减轻舵机冲击和蛇形摆动。
    double max_steering_rate_deg_s{120.0};

    // 以下速度只是物理目标，并不对应任何未经标定的电调 PWM。
    double max_speed_mps{0.60};
    double min_curve_speed_mps{0.18};
    double degraded_max_speed_mps{0.20};

    // 数值越大，曲率增大时降速越明显。
    double curvature_speed_gain{0.80};

    // confidence < stop 时停车；stop <= confidence < degraded 时低速运行。
    double confidence_stop{0.25};
    double confidence_degraded{0.55};

    // 路径输入的物理边界。超出边界通常表示像素坐标误传成了米。
    double max_path_distance_m{5.0};
    double max_lateral_offset_m{2.0};

    // 正常跟踪时的纵向速度变化率；任何失线/关闭仍然立即停车。
    double max_accel_mps2{0.80};
    double max_decel_mps2{1.20};

    // 迟滞避免置信度在阈值附近来回切换；失线后需连续多帧确认。
    double confidence_hysteresis{0.05};
    std::size_t confidence_reacquire_frames{3};

    // 配置有误时抛出 std::invalid_argument，阻止控制器启动。
    void validate() const;
};

// 从工程配置文件的 "motion" 对象读取全部参数。缺项、重复项或非法值均拒绝。
[[nodiscard]] MotionConfig load_motion_config_json(const std::string& path);

// 单次控制周期的输出，不包含 GPIO 编号、PWM 或电调命令。
struct ControlCommand {
    ControlState state{ControlState::Stopped};
    double steering_rad{0.0};          // 前轮目标转角，左正右负。
    double curvature_inv_m{0.0};       // 路径曲率，单位 1/m，左正右负。
    double target_speed_mps{0.0};      // 停止或失线状态下始终为零。
    std::optional<PathPoint> target_point_m;  // 本周期采用的前视点。
    double confidence{0.0};
    std::string_view reason{"controller disabled"};

    [[nodiscard]] double steering_deg() const noexcept;
};

class MotionController {
public:
    explicit MotionController(MotionConfig config = {});

    // 清除历史转角；人工接管、重新开始跟踪时应调用。
    void reset() noexcept;

    // path_m 必须按由近到远排列，并采用车辆坐标系和米制单位。
    // dt_s 用于转角变化率限制；离线计算时可以传 std::nullopt。
    // enabled=false、路径无效或置信度过低时都会立即输出零速度。
    [[nodiscard]] ControlCommand update(
        const std::vector<PathPoint>& path_m,
        double confidence,
        std::optional<double> dt_s = std::nullopt,
        bool enabled = true);

    [[nodiscard]] const MotionConfig& config() const noexcept { return config_; }

private:
    [[nodiscard]] PathPoint select_target(const std::vector<PathPoint>& path_m) const;
    [[nodiscard]] double limit_steering_rate(
        double requested_rad, std::optional<double> dt_s) const;
    [[nodiscard]] double limit_speed_rate(
        double requested_mps, std::optional<double> dt_s) const noexcept;
    [[nodiscard]] ControlCommand stop(
        ControlState state, double confidence, std::string_view reason) noexcept;

    MotionConfig config_;
    double last_steering_rad_{0.0};
    double last_speed_mps_{0.0};
    bool reacquisition_required_{false};
    std::size_t reacquire_count_{0};
    bool tracking_latched_{false};
};

}  // namespace xtnetrc::motion
