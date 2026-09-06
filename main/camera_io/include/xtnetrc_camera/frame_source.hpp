#pragma once
#include "xtnetrc_vision/ground_projector.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace xtnetrc::camera {
using Clock = std::chrono::steady_clock;
struct Frame { cv::Mat image; Clock::time_point captured_at; };
struct IdentityCheck {
    bool matched{false};
    std::string device;
    std::string expected_device;
};

// No silent fallback to camera 0. An override is explicit and reported, and
// never authorizes resizing/cropping or bypasses frame-age/actuator watchdogs.
IdentityCheck check_identity(const vision::GroundProjectionConfig& calibration,
                             const std::string& requested_device,
                             bool allow_mismatch = false);
class FrameSource {
public:
    virtual ~FrameSource() = default;
    // nullopt-style false means no fresh frame within the bounded wait.
    virtual bool read(Frame& frame, std::chrono::milliseconds timeout) = 0;
};
std::unique_ptr<FrameSource> open_fixed_camera(
    const vision::GroundProjectionConfig& calibration, const IdentityCheck& identity);
std::unique_ptr<FrameSource> open_replay_image(const std::string& path, cv::Size required_size);
}  // namespace xtnetrc::camera
