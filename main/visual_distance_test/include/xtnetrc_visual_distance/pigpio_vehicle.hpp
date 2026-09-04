#pragma once

#include <memory>

namespace xtnetrc::visual_distance {

class PigpioVehicle {
public:
    PigpioVehicle();
    ~PigpioVehicle();
    PigpioVehicle(const PigpioVehicle&) = delete;
    PigpioVehicle& operator=(const PigpioVehicle&) = delete;

    void initialize(int steering_center_pwm);
    void set_steering_pwm(int steering_pwm);
    void drive_forward(int motor_pwm);
    void stop() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace xtnetrc::visual_distance
