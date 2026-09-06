#include "xtnetrc_runtime/app.hpp"
#include "xtnetrc_runtime/vision_worker.hpp"
#include "xtnetrc_hardware/byte_source.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <opencv2/imgcodecs.hpp>

namespace xtnetrc::runtime {
namespace {
volatile std::sig_atomic_t interrupted = 0;
void handle_signal(int) { interrupted = 1; }
struct Options {
    std::string config{"main/config/runtime_2023.json"}, calibration, device, serial, replay, overlay;
    bool motor{false}, ack{false}, mismatch{false}, unverified{false}, help{false};
};
Options parse(int argc, char** argv) {
    Options out;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() {
            if (++i == argc) throw std::invalid_argument(arg + " requires a value");
            return std::string(argv[i]);
        };
        if (arg == "--config") out.config = value();
        else if (arg == "--calibration") out.calibration = value();
        else if (arg == "--camera-device") out.device = value();
        else if (arg == "--serial-device") out.serial = value();
        else if (arg == "--replay-image") out.replay = value();
        else if (arg == "--save-overlay") out.overlay = value();
        else if (arg == "--enable-motor") out.motor = true;
        else if (arg == "--i-understand-vehicle-will-move") out.ack = true;
        else if (arg == "--allow-camera-mismatch") out.mismatch = true;
        else if (arg == "--allow-unverified-calibration") out.unverified = true;
        else if (arg == "--help") out.help = true;
        else throw std::invalid_argument("unknown option: " + arg + "; use --help (legacy PWM options retired)");
    }
    if (out.motor != out.ack) throw std::invalid_argument("motor requires both explicit safety switches");
    if (out.motor && !out.replay.empty()) throw std::invalid_argument("replay can never enable hardware output");
    if (!out.replay.empty() && (!out.device.empty() || !out.serial.empty()))
        throw std::invalid_argument("replay cannot select live devices");
    return out;
}
}

int run(int argc, char** argv) try {
    const auto options = parse(argc, argv);
    if (options.help) {
        std::cout << "XT-NetRC unified guarded runtime (default: NO hardware output)\n"
            "  --config PATH  --calibration PATH  --camera-device /dev/v4l/by-id/...\n"
            "  --serial-device /dev/serial/by-id/...\n"
            "  --replay-image PATH  --save-overlay PATH (offline replay never drives)\n"
            "  --enable-motor --i-understand-vehicle-will-move\n"
            "  --allow-camera-mismatch (explicit degraded mode; target <= 0.10 m/s)\n"
            "  --allow-unverified-calibration (guarded commissioning only)\n"
            "Resolution, stale-data, emergency stop and watchdog checks cannot be bypassed.\n";
        return 0;
    }
    interrupted = 0;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    auto config = load_runtime_config(options.config);
    if (!options.calibration.empty()) config.calibration_path = options.calibration;
    if (!options.serial.empty()) config.serial_device = options.serial;
    const auto calibration = vision::load_ground_projection_json(config.calibration_path);
    const bool replay = !options.replay.empty();
    const bool unverified = !config.actuator_calibration_verified || !config.ground_calibration_verified;
    if (options.motor && unverified && !options.unverified)
        throw std::runtime_error("calibration not independently verified; inspect profile and use explicit --allow-unverified-calibration only for guarded commissioning");
    camera::IdentityCheck identity;
    std::unique_ptr<camera::FrameSource> source;
    if (replay) {
        source = camera::open_replay_image(options.replay, {calibration.image_width, calibration.image_height});
        std::cout << "REPLAY hardware_output=disabled gps=synthetic_zero_speed\n";
    } else {
        identity = camera::check_identity(calibration, options.device, options.mismatch);
        std::cout << "CAMERA device=" << identity.device << " calibration_device=" << identity.expected_device
                  << " matched=" << identity.matched << " size=" << calibration.image_width << 'x' << calibration.image_height << '\n';
        source = camera::open_fixed_camera(calibration, identity);
    }
    const bool degraded = (!replay && !identity.matched) || unverified;
    if (degraded) {
        config.target_speed_mps = std::min(config.target_speed_mps, 0.10);
        std::cerr << "WARNING degraded calibration/identity: target <= 0.10 m/s; physical speed and ground distances are NOT guaranteed. "
                     "Manual supervision required; no automatic fallback to another camera.\n";
    }
    std::unique_ptr<hardware::ByteSource> serial;
    if (!replay) serial = hardware::open_serial(config.serial_device, config.serial_baud);
    GpsInput gps(config.gps_timeout_s);
    speed::WitFrameParser parser;
    VisionWorker vision(std::move(source), calibration);
    // Declared after vision: neutral/stop before joining perception, even when
    // perception is exceptionally slow during stack unwinding.
    std::unique_ptr<hardware::GuardedVehicle> vehicle;
    ControlPipeline pipeline(config);
    auto start = camera::Clock::now();
    auto next_tick = start;
    double previous_s = monotonic_seconds(start), drive_start_s = -1, armed_s = -1;
    std::uint64_t preflight_sequence = 0;
    int consecutive_good = 0, ticks = 0;
    bool moved = false, preflight_passed = false;
    RuntimeOutput output;
    std::string stop_reason = "observation complete";
    std::cout << "START hardware_output=" << (options.motor ? "requested; awaiting preflight" : "disabled") << std::endl;
    while (!interrupted) {
        next_tick += std::chrono::milliseconds(50);  // 20 Hz control, independent perception worker.
        if (serial) {
            // At 9600 baud a full 256-byte batch already represents about
            // 267 ms of wire time; reject backlog instead of stamping it fresh.
            std::uint8_t bytes[256];
            const auto count = serial->read(bytes, sizeof(bytes), std::chrono::milliseconds(0));
            const double received_s = monotonic_seconds();
            // Saturation indicates backlog; never treat queued serial data as fresh.
            if (count == sizeof(bytes)) throw std::runtime_error("GPS input backlog");
            for (std::size_t i = 0; i < count; ++i) {
                const auto type = parser.feed(bytes[i]);
                if (type) gps.accept(*type, parser.data(), received_s);
            }
        }
        const auto snapshot = vision.snapshot();
        const double now_s = monotonic_seconds();
        const double dt_s = now_s - previous_s;
        previous_s = now_s;
        if (!snapshot.error.empty()) throw std::runtime_error(snapshot.error);
        const auto velocity = replay ? speed::SpeedEstimate{true, 0, 0, 10, 1, "synthetic replay"} : gps.estimate(now_s);
        const bool fresh = snapshot.lane.sequence && snapshot.lane.valid &&
            snapshot.lane.captured_s <= now_s && now_s - snapshot.lane.captured_s <= config.lane_timeout_s;
        if (snapshot.lane.sequence != preflight_sequence) {
            consecutive_good = fresh && snapshot.lane.confidence >= 0.55 ? consecutive_good + 1 : 0;
            preflight_sequence = snapshot.lane.sequence;
        }
        if (!fresh || !velocity.valid) consecutive_good = 0;
        const bool ready = consecutive_good >= 10 && velocity.valid && (replay || gps.accepted_samples() >= 5);
        preflight_passed |= ready;
        if (options.motor && !vehicle && ready && velocity.speed_mps <= 0.10) {
            if (interrupted) break;
            vehicle = std::make_unique<hardware::GuardedVehicle>(
                hardware::make_pigpio_vehicle(config.board), config.board,
                std::chrono::milliseconds(300),
                std::chrono::milliseconds(static_cast<int>(config.maximum_run_s * 1000)));
            vehicle->arm();
            armed_s = now_s;
            std::cout << "PREFLIGHT_PASS ESC_ARMING neutral for 5s; steering_center="
                      << config.board.steering_center << std::endl;
        }
        const bool enabled = !options.motor || (vehicle && now_s - armed_s >= 5.0 && ready);
        if (enabled && !interrupted) {
            output = pipeline.update(snapshot.lane, velocity, now_s, std::max(dt_s, 1e-6), true);
            if (vehicle) {
                if (vehicle->tripped()) throw std::runtime_error("actuator watchdog tripped");
                if (moved && !output.drive) { stop_reason = output.reason; break; }
                if (output.drive) {
                    if (interrupted) break;
                    vehicle->submit(output.pwm);
                    if (!moved) { moved = true; drive_start_s = now_s; }
                }
            }
        } else if (moved) {
            stop_reason = "preflight/data validity lost"; break;
        }
        if (++ticks % 10 == 0) {
            std::cout << std::fixed << std::setprecision(3)
                << "OBS lane=" << fresh << " confidence=" << snapshot.lane.confidence
                << " speed_valid=" << velocity.valid << " speed_mps=" << velocity.speed_mps
                << " target_mps=" << output.motion.target_speed_mps << " pi_trim=" << output.speed.pwm_trim
                << " steering_pwm=" << output.pwm.steering << " motor_pwm=" << output.pwm.motor
                << " distance_estimate_m=" << output.distance_m << " reason=" << output.reason << std::endl;
        }
        if (moved && now_s - drive_start_s >= config.maximum_run_s) { stop_reason = "run deadline"; break; }
        const double elapsed = std::chrono::duration<double>(camera::Clock::now() - start).count();
        if (!options.motor && elapsed >= config.maximum_run_s) break;
        if (options.motor && !moved && elapsed >= 25) { stop_reason = "preflight/arming timeout"; break; }
        // A missed tick is not followed by a burst of stale catch-up commands.
        if (next_tick < camera::Clock::now()) next_tick = camera::Clock::now();
        std::this_thread::sleep_until(next_tick);
    }
    if (vehicle) vehicle->stop();
    vision.stop();
    if (!options.overlay.empty()) {
        const auto last = vision.snapshot();
        if (!last.overlay.empty() && !cv::imwrite(options.overlay, last.overlay))
            throw std::runtime_error("cannot save final overlay");
    }
    std::cout << "STOP reason=" << (interrupted ? "signal" : stop_reason)
              << " hardware_output=" << (options.motor ? "neutral" : "never_enabled")
              << " preflight_passed=" << preflight_passed << std::endl;
    const bool normal_stop = stop_reason == "run deadline" || stop_reason == "distance threshold (GPS estimate)";
    return interrupted ? 130 : (options.motor ? (moved && normal_stop ? 0 : 3) : (preflight_passed ? 0 : 3));
} catch (const std::exception& error) {
    std::cerr << "STOP error=" << error.what() << '\n';
    return 2;
}
}  // namespace xtnetrc::runtime
