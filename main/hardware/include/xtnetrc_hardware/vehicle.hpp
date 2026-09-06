#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace xtnetrc::hardware {

// Board-specific electrical profile; algorithms must not include GPIO numbers.
struct VehicleProfile {
    int steering_gpio{12}, motor_gpio{13};
    int steering_hz{50}, steering_range{1000};
    int motor_hz{200}, motor_range{40000};
    int steering_min{65}, steering_center{72}, steering_max{80};
    int motor_neutral{10000}, motor_min{10200}, motor_max{11200};
    void validate() const;
};

struct PwmCommand {
    int steering{72};
    int motor{10000};
};

// Exclusive actuator owner. New boards implement this interface; mock tests do
// not load GPIO libraries. neutral() must be bounded and must never throw.
class VehicleDriver {
public:
    virtual ~VehicleDriver() = default;
    virtual void initialize() = 0;
    virtual void write(PwmCommand command) = 0;
    virtual void neutral() noexcept = 0;
};

std::unique_ptr<VehicleDriver> make_pigpio_vehicle(VehicleProfile profile);

// Independent command lease and absolute run deadline. Expiration is latched:
// submitting again cannot restart the motor. A new explicit arm is required.
// This is a process-level watchdog, NOT protection against SIGKILL/kernel/power
// failure; those still require a physical emergency stop / external watchdog.
class GuardedVehicle {
public:
    GuardedVehicle(std::unique_ptr<VehicleDriver> driver, VehicleProfile profile,
                   std::chrono::milliseconds lease = std::chrono::milliseconds(300),
                   std::chrono::milliseconds maximum_run = std::chrono::seconds(10));
    ~GuardedVehicle();
    GuardedVehicle(const GuardedVehicle&) = delete;
    GuardedVehicle& operator=(const GuardedVehicle&) = delete;
    void arm();
    void submit(PwmCommand command);
    void stop() noexcept;
    bool tripped() const;

private:
    void watch();
    std::unique_ptr<VehicleDriver> driver_;
    VehicleProfile profile_;
    std::chrono::milliseconds lease_, maximum_run_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::thread watchdog_;
    bool armed_{false}, stopping_{false}, tripped_{false}, driving_{false};
    std::chrono::steady_clock::time_point refreshed_{}, started_{};
};

}  // namespace xtnetrc::hardware
