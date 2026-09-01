#include "xtnetrc_gimbal/pigpio_driver.hpp"

#include <cstring>
#include <dlfcn.h>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace xtnetrc::gimbal {
namespace {

constexpr unsigned kPiOutput = 1U;

template <typename Function>
[[nodiscard]] Function load_symbol(void* handle, const char* name) {
    dlerror();
    void* symbol = dlsym(handle, name);
    const char* error = dlerror();
    if (error != nullptr || symbol == nullptr) {
        throw std::runtime_error(std::string("libpigpio missing symbol ") + name);
    }

    // POSIX 保证 dlsym 可返回函数地址；memcpy 避免 ISO C++ 对
    // void* 直接 reinterpret_cast 为函数指针产生的编译器警告。
    Function function{};
    static_assert(sizeof(function) == sizeof(symbol),
                  "function pointer and object pointer sizes differ");
    std::memcpy(&function, &symbol, sizeof(function));
    return function;
}

}  // namespace

struct PigpioDriver::Impl {
    using Initialize = int (*)();
    using Terminate = void (*)();
    using SetMode = int (*)(unsigned, unsigned);
    using SetPwmFrequency = int (*)(unsigned, unsigned);
    using SetPwmRange = int (*)(unsigned, unsigned);
    using Pwm = int (*)(unsigned, unsigned);

    void* handle{nullptr};
    Initialize gpio_initialize{nullptr};
    Terminate gpio_terminate{nullptr};
    SetMode gpio_set_mode{nullptr};
    SetPwmFrequency gpio_set_pwm_frequency{nullptr};
    SetPwmRange gpio_set_pwm_range{nullptr};
    Pwm gpio_pwm{nullptr};
    bool active{false};
    std::unordered_map<int, int> configured_ranges;
};

PigpioDriver::PigpioDriver() : impl_(std::make_unique<Impl>()) {}

PigpioDriver::~PigpioDriver() {
    shutdown();
}

void PigpioDriver::initialize() {
    if (impl_->active) {
        return;
    }

    // 原车常见安装位置均可由动态链接器通过这些 soname 找到。
    for (const char* candidate : {"libpigpio.so", "libpigpio.so.1"}) {
        impl_->handle = dlopen(candidate, RTLD_NOW | RTLD_LOCAL);
        if (impl_->handle != nullptr) {
            break;
        }
    }
    if (impl_->handle == nullptr) {
        throw std::runtime_error(
            "cannot load libpigpio.so; run this hardware command on the Raspberry Pi");
    }

    try {
        impl_->gpio_initialize =
            load_symbol<Impl::Initialize>(impl_->handle, "gpioInitialise");
        impl_->gpio_terminate =
            load_symbol<Impl::Terminate>(impl_->handle, "gpioTerminate");
        impl_->gpio_set_mode =
            load_symbol<Impl::SetMode>(impl_->handle, "gpioSetMode");
        impl_->gpio_set_pwm_frequency = load_symbol<Impl::SetPwmFrequency>(
            impl_->handle, "gpioSetPWMfrequency");
        impl_->gpio_set_pwm_range = load_symbol<Impl::SetPwmRange>(
            impl_->handle, "gpioSetPWMrange");
        impl_->gpio_pwm = load_symbol<Impl::Pwm>(impl_->handle, "gpioPWM");
        if (impl_->gpio_initialize() < 0) {
            throw std::runtime_error(
                "gpioInitialise failed; check permissions and other pigpio users");
        }
        impl_->active = true;
    } catch (...) {
        if (impl_->handle != nullptr) {
            dlclose(impl_->handle);
            impl_->handle = nullptr;
        }
        throw;
    }
}

void PigpioDriver::configure(const int gpio_bcm, const int frequency_hz,
                             const int pwm_range) {
    if (!impl_->active) {
        throw std::logic_error("pigpio driver is not initialized");
    }
    if (impl_->gpio_set_mode(static_cast<unsigned>(gpio_bcm), kPiOutput) < 0) {
        throw std::runtime_error("gpioSetMode failed for GPIO " +
                                 std::to_string(gpio_bcm));
    }
    if (impl_->gpio_set_pwm_frequency(static_cast<unsigned>(gpio_bcm),
                                      static_cast<unsigned>(frequency_hz)) < 0) {
        throw std::runtime_error("gpioSetPWMfrequency failed for GPIO " +
                                 std::to_string(gpio_bcm));
    }
    if (impl_->gpio_set_pwm_range(static_cast<unsigned>(gpio_bcm),
                                  static_cast<unsigned>(pwm_range)) < 0) {
        throw std::runtime_error("gpioSetPWMrange failed for GPIO " +
                                 std::to_string(gpio_bcm));
    }
    impl_->configured_ranges[gpio_bcm] = pwm_range;
}

void PigpioDriver::write(const int gpio_bcm, const int duty_cycle) {
    if (!impl_->active) {
        throw std::logic_error("pigpio driver is not initialized");
    }
    const auto range = impl_->configured_ranges.find(gpio_bcm);
    if (range == impl_->configured_ranges.end()) {
        throw std::logic_error("GPIO was not configured by this gimbal driver");
    }
    if (duty_cycle < 0 || duty_cycle > range->second) {
        throw std::invalid_argument("PWM duty cycle is outside configured range");
    }
    if (impl_->gpio_pwm(static_cast<unsigned>(gpio_bcm),
                        static_cast<unsigned>(duty_cycle)) < 0) {
        throw std::runtime_error("gpioPWM failed for GPIO " +
                                 std::to_string(gpio_bcm));
    }
}

void PigpioDriver::shutdown() noexcept {
    if (!impl_) {
        return;
    }
    if (impl_->active && impl_->gpio_terminate != nullptr) {
        impl_->gpio_terminate();
    }
    impl_->active = false;
    impl_->configured_ranges.clear();
    if (impl_->handle != nullptr) {
        dlclose(impl_->handle);
        impl_->handle = nullptr;
    }
}

}  // namespace xtnetrc::gimbal
