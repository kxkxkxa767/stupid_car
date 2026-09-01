#pragma once

#include <chrono>
#include <functional>
#include <string>

namespace xtnetrc::gimbal {

// 本车云台使用 pigpio 的“用户 PWM 范围”数值，不是微秒值。
// 在 50 Hz、range=1000 时，一个 PWM 单位约等于 20 微秒。
struct GimbalConfig {
    int pan_gpio_bcm{22};
    int tilt_gpio_bcm{23};
    int pwm_frequency_hz{50};
    int pwm_range{1000};

    // 水平轴实测电气范围为 25~125；这里各留 3 个单位余量。
    int pan_center_pwm{72};
    int pan_min_pwm{28};
    int pan_max_pwm{122};

    // 俯仰轴在 55 以下已顶住机械限位，因此安全下限设为 58。
    int tilt_center_pwm{73};
    int tilt_min_pwm{58};
    int tilt_max_pwm{122};

    // 每个周期最多改变一个 PWM 单位，避免云台瞬间大角度抽动。
    int max_step_pwm{1};
    int step_interval_ms{20};

    // 配置不符合本车已验证的 50 Hz/range=1000 模式时拒绝启动。
    void validate() const;
};

// 严格读取配置文件中的 "camera_gimbal" 对象。缺项、重复项和未知项均拒绝。
[[nodiscard]] GimbalConfig load_gimbal_config_json(const std::string& path);

struct GimbalPosition {
    int pan_pwm{72};
    int tilt_pwm{73};
};

// GPIO 驱动抽象。单元测试使用内存模拟驱动，树莓派使用 PigpioDriver。
class PwmDriver {
public:
    virtual ~PwmDriver() = default;
    virtual void initialize() = 0;
    virtual void configure(int gpio_bcm, int frequency_hz, int pwm_range) = 0;
    virtual void write(int gpio_bcm, int duty_cycle) = 0;
    virtual void shutdown() noexcept = 0;
};

using SleepFunction = std::function<void(std::chrono::milliseconds)>;

class GimbalController {
public:
    explicit GimbalController(PwmDriver& driver,
                              GimbalConfig config = {},
                              SleepFunction sleeper = {});
    ~GimbalController();

    GimbalController(const GimbalController&) = delete;
    GimbalController& operator=(const GimbalController&) = delete;

    // 初始化时只配置 GPIO 22/23，并先回到实测中心 X=72、Y=73。
    void initialize();

    // 超出范围的目标会先夹紧到软件安全范围，再以小步长移动。
    // 返回值是实际采用的安全目标，可用于网页或日志回显。
    [[nodiscard]] GimbalPosition move_to(int pan_pwm, int tilt_pwm);
    [[nodiscard]] GimbalPosition move_relative(int pan_delta_pwm,
                                               int tilt_delta_pwm);
    [[nodiscard]] GimbalPosition center();

    // 停止两路 PWM 并释放 pigpio。可重复调用，不会触碰车辆 GPIO 12/13。
    void release() noexcept;

    [[nodiscard]] bool initialized() const noexcept { return initialized_; }
    [[nodiscard]] GimbalPosition current_position() const;
    [[nodiscard]] const GimbalConfig& config() const noexcept { return config_; }

private:
    [[nodiscard]] GimbalPosition clamp_position(int pan_pwm,
                                                int tilt_pwm) const noexcept;
    [[nodiscard]] static int approach(int current, int target, int max_step) noexcept;
    void require_initialized() const;

    PwmDriver& driver_;
    GimbalConfig config_;
    SleepFunction sleeper_;
    GimbalPosition current_{};
    bool initialized_{false};
};

}  // namespace xtnetrc::gimbal
