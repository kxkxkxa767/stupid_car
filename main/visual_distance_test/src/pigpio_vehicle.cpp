#include "xtnetrc_visual_distance/pigpio_vehicle.hpp"

#include <cstring>
#include <dlfcn.h>
#include <stdexcept>
#include <string>

namespace xtnetrc::visual_distance {
namespace {

template <typename Function>
Function load_symbol(void* handle, const char* name) {
    dlerror();
    void* symbol = dlsym(handle, name);
    if (symbol == nullptr || dlerror() != nullptr) {
        throw std::runtime_error(std::string("libpigpio missing symbol ") + name);
    }
    Function function{};
    static_assert(sizeof(function) == sizeof(symbol));
    std::memcpy(&function, &symbol, sizeof(function));
    return function;
}

}  // namespace

struct PigpioVehicle::Impl {
    using Initialize = int (*)();
    using Terminate = void (*)();
    using SetMode = int (*)(unsigned, unsigned);
    using SetFrequency = int (*)(unsigned, unsigned);
    using SetRange = int (*)(unsigned, unsigned);
    using Pwm = int (*)(unsigned, unsigned);

    void* handle{nullptr};
    Initialize initialize{nullptr};
    Terminate terminate{nullptr};
    SetMode set_mode{nullptr};
    SetFrequency set_frequency{nullptr};
    SetRange set_range{nullptr};
    Pwm pwm{nullptr};
    bool active{false};
};

PigpioVehicle::PigpioVehicle() : impl_(std::make_unique<Impl>()) {}
PigpioVehicle::~PigpioVehicle() { stop(); }

void PigpioVehicle::initialize(const int steering_center_pwm) {
    if (impl_->active) {
        return;
    }
    for (const char* name : {"libpigpio.so", "libpigpio.so.1"}) {
        impl_->handle = dlopen(name, RTLD_NOW | RTLD_LOCAL);
        if (impl_->handle != nullptr) break;
    }
    if (impl_->handle == nullptr) {
        throw std::runtime_error("cannot load libpigpio");
    }
    try {
        impl_->initialize = load_symbol<Impl::Initialize>(impl_->handle, "gpioInitialise");
        impl_->terminate = load_symbol<Impl::Terminate>(impl_->handle, "gpioTerminate");
        impl_->set_mode = load_symbol<Impl::SetMode>(impl_->handle, "gpioSetMode");
        impl_->set_frequency = load_symbol<Impl::SetFrequency>(impl_->handle, "gpioSetPWMfrequency");
        impl_->set_range = load_symbol<Impl::SetRange>(impl_->handle, "gpioSetPWMrange");
        impl_->pwm = load_symbol<Impl::Pwm>(impl_->handle, "gpioPWM");
        if (impl_->initialize() < 0) {
            throw std::runtime_error("gpioInitialise failed");
        }
        impl_->active = true;
        if (impl_->set_mode(12, 1) < 0 || impl_->set_frequency(12, 50) < 0 ||
            impl_->set_range(12, 1000) < 0 ||
            impl_->set_mode(13, 1) < 0 || impl_->set_frequency(13, 200) < 0 ||
            impl_->set_range(13, 40000) < 0 ||
            impl_->pwm(12, static_cast<unsigned>(steering_center_pwm)) < 0 ||
            impl_->pwm(13, 10000) < 0) {
            throw std::runtime_error("failed to configure vehicle PWM");
        }
    } catch (...) {
        stop();
        throw;
    }
}

void PigpioVehicle::drive_forward(const int motor_pwm) {
    if (!impl_->active || motor_pwm < 10200 || motor_pwm > 11200) {
        throw std::invalid_argument("motor PWM must be within safe test range 10200..11200");
    }
    if (impl_->pwm(13, static_cast<unsigned>(motor_pwm)) < 0) {
        throw std::runtime_error("failed to write motor PWM");
    }
}

void PigpioVehicle::stop() noexcept {
    if (!impl_) return;
    if (impl_->active) {
        if (impl_->pwm != nullptr) {
            impl_->pwm(13, 10000);
        }
        if (impl_->terminate != nullptr) {
            impl_->terminate();
        }
    }
    impl_->active = false;
    if (impl_->handle != nullptr) {
        dlclose(impl_->handle);
        impl_->handle = nullptr;
    }
}

}  // namespace xtnetrc::visual_distance
