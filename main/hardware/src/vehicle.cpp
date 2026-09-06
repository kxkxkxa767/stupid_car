#include "xtnetrc_hardware/vehicle.hpp"

#include <stdexcept>
#include <utility>

namespace xtnetrc::hardware {

void VehicleProfile::validate() const {
    if (steering_gpio < 0 || motor_gpio < 0 || steering_gpio == motor_gpio ||
        steering_hz <= 0 || motor_hz <= 0 ||
        !(0 < steering_min && steering_min <= steering_center &&
          steering_center <= steering_max && steering_max < steering_range) ||
        !(0 < motor_neutral && motor_neutral < motor_min &&
          motor_min <= motor_max && motor_max < motor_range)) {
        throw std::invalid_argument("invalid vehicle electrical profile");
    }
}

GuardedVehicle::GuardedVehicle(std::unique_ptr<VehicleDriver> driver,
                             VehicleProfile profile,
                             std::chrono::milliseconds lease,
                             std::chrono::milliseconds maximum_run)
    : driver_(std::move(driver)), profile_(profile), lease_(lease), maximum_run_(maximum_run) {
    profile_.validate();
    if (!driver_ || lease_.count() <= 0 || lease_ > std::chrono::milliseconds(500) ||
        maximum_run_ <= lease_ || maximum_run_ > std::chrono::seconds(12)) {
        throw std::invalid_argument("invalid actuator watchdog limits");
    }
}

GuardedVehicle::~GuardedVehicle() { stop(); }

void GuardedVehicle::arm() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (armed_ || stopping_ || tripped_) throw std::logic_error("vehicle already armed/stopped");
    driver_->initialize();
    driver_->neutral();
    armed_ = true;
    try { watchdog_ = std::thread(&GuardedVehicle::watch, this); }
    catch (...) { armed_ = false; driver_->neutral(); throw; }
}

void GuardedVehicle::submit(PwmCommand command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_ || stopping_ || tripped_) throw std::runtime_error("actuator is not armed or watchdog latched");
    const auto now = std::chrono::steady_clock::now();
    // Check here too: a delayed submit must not revive an expired lease before
    // the watchdog thread happens to be scheduled.
    if ((driving_ && (now - refreshed_ >= lease_ || now - started_ >= maximum_run_)) ||
        command.steering < profile_.steering_min || command.steering > profile_.steering_max ||
        (command.motor != profile_.motor_neutral &&
         (command.motor < profile_.motor_min || command.motor > profile_.motor_max))) {
        tripped_ = true;
        driver_->neutral();
        throw std::runtime_error("expired or out-of-envelope actuator command");
    }
    if (!driving_ && command.motor != profile_.motor_neutral) {
        driving_ = true;
        started_ = now;
    }
    refreshed_ = now;
    try { driver_->write(command); }
    catch (...) { tripped_ = true; driver_->neutral(); throw; }
    wake_.notify_all();
}

void GuardedVehicle::watch() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stopping_) {
        wake_.wait_for(lock, std::chrono::milliseconds(10));
        const auto now = std::chrono::steady_clock::now();
        if (!tripped_ && driving_ &&
            (now - refreshed_ >= lease_ || now - started_ >= maximum_run_)) {
            tripped_ = true;
            driver_->neutral();
        }
    }
}

void GuardedVehicle::stop() noexcept {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        if (armed_) driver_->neutral();
        armed_ = false;
    }
    wake_.notify_all();
    if (watchdog_.joinable()) watchdog_.join();
}

bool GuardedVehicle::tripped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tripped_;
}

}  // namespace xtnetrc::hardware
