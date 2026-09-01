#include "xtnetrc_gimbal/gimbal.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gimbal = xtnetrc::gimbal;

namespace {

int failures = 0;

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "失败: " << message << '\n';
    }
}

class FakePwmDriver final : public gimbal::PwmDriver {
public:
    void initialize() override {
        initialized = true;
        ++initialize_count;
    }

    void configure(const int gpio_bcm, const int frequency_hz,
                   const int pwm_range) override {
        if (!initialized) {
            throw std::logic_error("fake driver was not initialized");
        }
        configurations[gpio_bcm] = {frequency_hz, pwm_range};
    }

    void write(const int gpio_bcm, const int duty_cycle) override {
        if (!initialized) {
            throw std::logic_error("fake driver was not initialized");
        }
        writes.emplace_back(gpio_bcm, duty_cycle);
    }

    void shutdown() noexcept override {
        if (initialized) {
            ++shutdown_count;
        }
        initialized = false;
    }

    bool initialized{false};
    int initialize_count{0};
    int shutdown_count{0};
    std::map<int, std::pair<int, int>> configurations;
    std::vector<std::pair<int, int>> writes;
};

void test_configuration_loader() {
    const gimbal::GimbalConfig config =
        gimbal::load_gimbal_config_json(XTNETRC_GIMBAL_TEST_CONFIG_PATH);
    expect(config.pan_gpio_bcm == 22 && config.tilt_gpio_bcm == 23,
           "配置必须载入两路实车 BCM GPIO");
    expect(config.pan_min_pwm == 28 && config.pan_max_pwm == 122,
           "配置必须载入水平轴安全限位");
    expect(config.tilt_min_pwm == 58 && config.tilt_max_pwm == 122,
           "配置必须载入俯仰轴安全限位");

    const std::string bad_path = "/tmp/xtnetrc_bad_gimbal_config.json";
    {
        std::ofstream output(bad_path);
        output << "{\"camera_gimbal\":{\"pan_gpio_bcm\":22}}";
    }
    bool threw = false;
    try {
        static_cast<void>(gimbal::load_gimbal_config_json(bad_path));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "缺项配置必须拒绝启动");
    std::remove(bad_path.c_str());
}

void test_invalid_config() {
    gimbal::GimbalConfig config;
    config.tilt_min_pwm = 50;  // 实测已进入机械堵转区。
    bool threw = false;
    try {
        config.validate();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "配置文件不得把俯仰下限放入实测堵转区");

    config = {};
    config.pwm_frequency_hz = 100;
    threw = false;
    try {
        config.validate();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "改变频率会改变脉宽，必须拒绝启动");

    config = {};
    config.pan_gpio_bcm = 12;
    threw = false;
    try {
        config.validate();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "配置文件不得让云台模块触碰车辆转向 GPIO 12");
}

void test_initialization_limits_and_smoothing() {
    FakePwmDriver driver;
    gimbal::GimbalConfig config;
    config.max_step_pwm = 4;
    int sleep_count = 0;
    gimbal::GimbalController controller(
        driver, config,
        [&sleep_count](std::chrono::milliseconds) { ++sleep_count; });

    bool threw_before_initialize = false;
    try {
        static_cast<void>(controller.move_to(72, 73));
    } catch (const std::logic_error&) {
        threw_before_initialize = true;
    }
    expect(threw_before_initialize, "未初始化不得输出 PWM");

    controller.initialize();
    expect(driver.configurations[22] == std::make_pair(50, 1000),
           "水平轴必须配置为 50 Hz/range 1000");
    expect(driver.configurations[23] == std::make_pair(50, 1000),
           "俯仰轴必须配置为 50 Hz/range 1000");
    expect(driver.writes.size() >= 2 && driver.writes[0] == std::make_pair(22, 72) &&
               driver.writes[1] == std::make_pair(23, 73),
           "初始化必须先回到原厂中心");

    const std::size_t movement_begin = driver.writes.size();
    const gimbal::GimbalPosition applied = controller.move_to(1000, -1000);
    expect(applied.pan_pwm == 122 && applied.tilt_pwm == 58,
           "越界目标必须夹紧到实测安全范围");

    std::map<int, int> previous{{22, 72}, {23, 73}};
    for (std::size_t index = movement_begin; index < driver.writes.size(); ++index) {
        const auto [gpio, duty] = driver.writes[index];
        const bool safe = gpio == 22 ? (duty >= 28 && duty <= 122)
                                     : (duty >= 58 && duty <= 122);
        expect(safe, "硬件写入值不得越过软件限位");
        expect(std::abs(duty - previous[gpio]) <= config.max_step_pwm,
               "相邻写入不得超过平滑移动步长");
        previous[gpio] = duty;
    }
    expect(sleep_count > 0, "移动过程必须执行步间等待");

    const gimbal::GimbalPosition centered = controller.center();
    expect(centered.pan_pwm == 72 && centered.tilt_pwm == 73,
           "回中必须恢复实测中心值");

    const gimbal::GimbalPosition saturated = controller.move_relative(
        std::numeric_limits<int>::max(), std::numeric_limits<int>::min());
    expect(saturated.pan_pwm == 122 && saturated.tilt_pwm == 58,
           "极端相对输入必须无溢出地限制到安全边界");

    controller.release();
    expect(driver.shutdown_count == 1, "释放时必须关闭驱动");
    expect(driver.writes[driver.writes.size() - 2] == std::make_pair(22, 0) &&
               driver.writes.back() == std::make_pair(23, 0),
           "释放时必须停止两路云台 PWM");
}

}  // namespace

int main() {
    test_configuration_loader();
    test_invalid_config();
    test_initialization_limits_and_smoothing();
    if (failures != 0) {
        std::cerr << failures << " 项云台控制测试失败\n";
        return 1;
    }
    std::cout << "云台控制测试全部通过\n";
    return 0;
}
