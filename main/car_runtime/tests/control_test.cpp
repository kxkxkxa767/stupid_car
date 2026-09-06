#include "xtnetrc_runtime/control.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace rt = xtnetrc::runtime;
namespace sp = xtnetrc::speed;
void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
rt::RuntimeConfig config() {
    rt::RuntimeConfig result;
    result.serial_device = "mock";
    result.motion.max_accel_mps2 = 10;
    result.board.steering_min = 70; result.board.steering_max = 73;
    return result;
}
rt::LaneObservation lane(std::uint64_t seq, double time) { return {seq, time, true, 1.10, 0, 0, 0.7, 0.9}; }
sp::SpeedEstimate velocity(double value = 0.0) { return {true, value, 0.1, 8, 1.0, "mock GPS"}; }
rt::RuntimeOutput warm(rt::ControlPipeline& controller, double measured = 0.0) {
    rt::RuntimeOutput out;
    for (int i = 1; i <= 10; ++i) out = controller.update(lane(i, 1.0 + i * 0.05), velocity(measured), 1.0 + i * 0.05, 0.05, true);
    check(out.drive, "unified pipeline never reached drive state");
    return out;
}
int main() try {
    rt::ControlPipeline slow(config()), fast(config());
    const auto a = warm(slow, 0.02), b = warm(fast, 0.22);
    check(a.speed.pwm_trim > b.speed.pwm_trim && a.pwm.motor > b.pwm.motor, "GPS PI does not influence actual motor command");
    check(a.motion.target_speed_mps <= 0.2 && a.pwm.steering == 72, "target cap/straight center incorrect");
    {
        rt::ControlPipeline controller(config()); warm(controller);
        auto point = lane(11, 1.55); point.center_y_m = 0.03;
        auto out = controller.update(point, velocity(), 1.55, 0.05, true);
        check(out.drive && out.motion.steering_rad > 0 && out.pwm.steering > 72, "left lane offset has wrong steering sign");
    }
    for (int fault = 0; fault < 8; ++fault) {
        rt::ControlPipeline controller(config()); warm(controller);
        auto point = lane(11, 1.55); auto speed = velocity(); double dt = 0.05;
        if (fault == 0) point.captured_s = 0;
        if (fault == 1) point.captured_s = 2;
        if (fault == 2) point.valid = false;
        if (fault == 3) speed.valid = false;
        if (fault == 4) speed.gps_age_s = 10;
        if (fault == 5) dt = 0.3;
        if (fault == 6) point.center_y_m = std::numeric_limits<double>::quiet_NaN();
        if (fault == 7) speed.speed_mps = 0.5;
        const auto out = controller.update(point, speed, 1.55, dt, true);
        check(!out.drive && out.pwm.motor == 10000, "fault failed to neutralize");
        check(!controller.update(lane(12,1.6), velocity(),1.6,0.05,true).drive, "fault restarted without reset");
    }
    {
        rt::ControlPipeline controller(config()); warm(controller);
        check(!controller.update(lane(11,7), velocity(),7,0.05,true).drive, "maximum runtime ignored");
    }
    {
        auto limits = config(); limits.maximum_distance_m = 0.05;
        rt::ControlPipeline controller(limits);
        rt::RuntimeOutput out;
        bool drove = false;
        for (int i = 1; i < 30; ++i) {
            out = controller.update(lane(i,1+i*0.05), velocity(0.2), 1+i*0.05,0.05,true);
            drove |= out.drive;
        }
        check(drove && !out.drive && out.distance_m >= 0.05, "distance stop missing");
    }
    {
        rt::ControlPipeline controller(config());
        auto first = lane(1,1);
        for (int i = 0; i < 3; ++i)
            check(!controller.update(first,velocity(),1+i*.05,.05,true).drive, "duplicate frame counted as confidence recovery");
    }
    {
        rt::GpsInput input(1);
        sp::WitSensorData data;
        data.gps_speed_mps = 0.1; data.satellites = 8; data.hdop = 1;
        input.accept(0x58,data,1);
        check(!input.estimate(1).valid, "GPS accepted without quality packet");
        input.accept(0x5A,data,1); input.accept(0x58,data,1.1);
        check(input.estimate(1.2).valid, "valid GPS pair rejected");
        input.accept(0x58,data,2.2);
        check(!input.estimate(2.2).valid, "stale GPS quality reused");
        input.accept(0x5A,data,2.3); input.accept(0x58,data,2.4);
        data.satellites = 2; input.accept(0x5A,data,2.5);
        check(!input.estimate(2.5).valid, "bad quality did not invalidate speed immediately");
    }
    std::cout << "integrated motion/GPS PI/freshness/latched stop tests passed\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
