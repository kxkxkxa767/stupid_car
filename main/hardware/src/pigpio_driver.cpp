#include "xtnetrc_hardware/vehicle.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#if defined(__linux__) && defined(XTNETRC_WITH_PIGPIO)
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

namespace xtnetrc::hardware {
#if defined(__linux__) && defined(XTNETRC_WITH_PIGPIO)
namespace {
template <class Function> Function symbol(void* handle, const char* name) {
    void* address = dlsym(handle, name);
    if (!address) throw std::runtime_error(std::string("missing pigpio symbol: ") + name);
    Function result;
    static_assert(sizeof(result) == sizeof(address));
    std::memcpy(&result, &address, sizeof(result));
    return result;
}

class PigpioDriver final : public VehicleDriver {
public:
    explicit PigpioDriver(VehicleProfile profile) : profile_(profile) {
        profile_.validate();
        // Measured safety envelope belongs to this board adapter, not control
        // algorithms. A different board must supply a different adapter/profile.
        if (profile_.steering_gpio != 12 || profile_.motor_gpio != 13 ||
            profile_.steering_hz != 50 || profile_.steering_range != 1000 ||
            profile_.motor_hz != 200 || profile_.motor_range != 40000 ||
            profile_.steering_min < 65 || profile_.steering_max > 80 ||
            profile_.motor_neutral != 10000 || profile_.motor_min < 10200 ||
            profile_.motor_max > 11200) {
            throw std::invalid_argument("profile exceeds measured Raspberry Pi vehicle envelope");
        }
    }
    ~PigpioDriver() override {
        neutral();
        if (active_) terminate_();
        if (handle_) dlclose(handle_);
        if (lock_ >= 0) ::close(lock_);
    }
    void initialize() override {
        if (active_ || handle_) throw std::logic_error("driver cannot initialize twice");
        // Cooperating staging programs share this lock. It does not stop or
        // take over the original vendor service, which must be checked manually.
        lock_ = ::open("/run/lock/xtnetrc-actuators.lock", O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) != 0)
            throw std::runtime_error("cannot acquire exclusive XT-NetRC actuator lock");
        handle_ = dlopen("libpigpio.so", RTLD_NOW | RTLD_LOCAL);
        if (!handle_) handle_ = dlopen("libpigpio.so.1", RTLD_NOW | RTLD_LOCAL);
        if (!handle_) throw std::runtime_error("cannot load libpigpio");
        auto initialize = symbol<int(*)()>(handle_, "gpioInitialise");
        terminate_ = symbol<void(*)()>(handle_, "gpioTerminate");
        auto mode = symbol<int(*)(unsigned,unsigned)>(handle_, "gpioSetMode");
        auto frequency = symbol<int(*)(unsigned,unsigned)>(handle_, "gpioSetPWMfrequency");
        auto range = symbol<int(*)(unsigned,unsigned)>(handle_, "gpioSetPWMrange");
        pwm_ = symbol<int(*)(unsigned,unsigned)>(handle_, "gpioPWM");
        if (initialize() < 0) throw std::runtime_error("gpioInitialise failed");
        active_ = true;
        if (mode(profile_.motor_gpio, 1) < 0 ||
            frequency(profile_.motor_gpio, profile_.motor_hz) != profile_.motor_hz ||
            range(profile_.motor_gpio, profile_.motor_range) < 0 ||
            pwm_(profile_.motor_gpio, profile_.motor_neutral) < 0 ||
            mode(profile_.steering_gpio, 1) < 0 ||
            frequency(profile_.steering_gpio, profile_.steering_hz) != profile_.steering_hz ||
            range(profile_.steering_gpio, profile_.steering_range) < 0 ||
            pwm_(profile_.steering_gpio, profile_.steering_center) < 0) {
            neutral();
            throw std::runtime_error("failed to configure measured PWM profile");
        }
    }
    void write(PwmCommand command) override {
        if (!active_) throw std::logic_error("driver not initialized");
        if (pwm_(profile_.steering_gpio, command.steering) < 0 ||
            pwm_(profile_.motor_gpio, command.motor) < 0)
            throw std::runtime_error("pigpio write failed");
    }
    void neutral() noexcept override {
        if (active_) {
            pwm_(profile_.motor_gpio, profile_.motor_neutral);
            pwm_(profile_.steering_gpio, profile_.steering_center);
        }
    }
private:
    VehicleProfile profile_;
    int lock_{-1};
    void* handle_{nullptr};
    void (*terminate_)(){};
    int (*pwm_)(unsigned,unsigned){};
    bool active_{false};
};
}  // namespace
#endif

std::unique_ptr<VehicleDriver> make_pigpio_vehicle(VehicleProfile profile) {
#if defined(__linux__) && defined(XTNETRC_WITH_PIGPIO)
    return std::make_unique<PigpioDriver>(profile);
#else
    (void)profile;
    throw std::runtime_error("pigpio output unavailable: requires Linux and XTNETRC_WITH_PIGPIO=ON");
#endif
}
}  // namespace xtnetrc::hardware
