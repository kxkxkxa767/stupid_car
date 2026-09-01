#pragma once

#include "xtnetrc_gimbal/gimbal.hpp"

#include <memory>

namespace xtnetrc::gimbal {

// 运行时动态加载树莓派已有的 libpigpio.so。
// 这样头文件不会依赖 pigpio.h，Mac 交叉编译也无需再次下载依赖。
class PigpioDriver final : public PwmDriver {
public:
    PigpioDriver();
    ~PigpioDriver() override;

    PigpioDriver(const PigpioDriver&) = delete;
    PigpioDriver& operator=(const PigpioDriver&) = delete;

    void initialize() override;
    void configure(int gpio_bcm, int frequency_hz, int pwm_range) override;
    void write(int gpio_bcm, int duty_cycle) override;
    void shutdown() noexcept override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace xtnetrc::gimbal
