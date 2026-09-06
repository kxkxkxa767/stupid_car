#include "xtnetrc_hardware/vehicle.hpp"
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace std::chrono_literals;
namespace hw = xtnetrc::hardware;
struct State { std::atomic_int motor{0}, neutral_calls{0}; bool fail{false}; };
struct Mock final : hw::VehicleDriver {
    explicit Mock(State& value) : state(value) {}
    void initialize() override { state.motor = 10000; }
    void write(hw::PwmCommand command) override {
        if (state.fail) throw std::runtime_error("mock write failure");
        state.motor = command.motor;
    }
    void neutral() noexcept override { state.motor = 10000; ++state.neutral_calls; }
    State& state;
};
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
template<class F> void rejects(F fn, const char* message) {
    bool threw = false; try { fn(); } catch (const std::exception&) { threw = true; }
    check(threw, message);
}
int main() try {
    hw::VehicleProfile profile;
    {
        State state;
        hw::GuardedVehicle vehicle(std::make_unique<Mock>(state), profile, 80ms, 400ms);
        rejects([&] { vehicle.submit({72, 11100}); }, "unarmed output allowed");
        vehicle.arm(); check(state.motor == 10000, "arming did not neutralize");
        vehicle.submit({72, 11100});
        // Simulate a blocked camera/control thread: no refreshes at all.
        std::this_thread::sleep_for(180ms);
        check(state.motor == 10000 && vehicle.tripped(), "independent lease failed to stop motor");
        rejects([&] { vehicle.submit({72, 11100}); }, "expired watchdog restarted motor");
    }
    {
        State state;
        hw::GuardedVehicle vehicle(std::make_unique<Mock>(state), profile, 100ms, 220ms);
        vehicle.arm();
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < 400ms && !vehicle.tripped()) {
            try { vehicle.submit({72, 11100}); } catch (const std::exception&) { break; }
            std::this_thread::sleep_for(15ms);
        }
        check(vehicle.tripped() && state.motor == 10000, "heartbeat extended absolute deadline");
    }
    {
        State state;
        hw::GuardedVehicle vehicle(std::make_unique<Mock>(state), profile);
        vehicle.arm();
        rejects([&] { vehicle.submit({90, 11100}); }, "unsafe steering accepted");
        check(vehicle.tripped() && state.motor == 10000, "invalid command did not latch neutral");
    }
    {
        State state;
        hw::GuardedVehicle vehicle(std::make_unique<Mock>(state), profile);
        vehicle.arm(); state.fail = true;
        rejects([&] { vehicle.submit({72, 11100}); }, "driver failure swallowed");
        check(vehicle.tripped() && state.motor == 10000, "driver fault failed to stop");
        vehicle.stop(); vehicle.stop();
    }
    std::cout << "mock actuator/watchdog tests passed; no physical hardware used\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
