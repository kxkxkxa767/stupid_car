#include "xtnetrc_camera/frame_source.hpp"
#include <filesystem>
#include <stdexcept>
#include <thread>
#include <opencv2/imgcodecs.hpp>

namespace xtnetrc::camera {
IdentityCheck check_identity(const vision::GroundProjectionConfig& calibration,
                             const std::string& requested_device, bool allow_mismatch) {
    IdentityCheck result;
    result.expected_device = calibration.camera_device_by_id;
    result.device = requested_device.empty() ? result.expected_device : requested_device;
    if (result.device.empty()) throw std::invalid_argument("no camera device identity; specify --camera-device");
    std::error_code error;
    result.matched = calibration.camera_role == "front_fixed" &&
        !result.expected_device.empty() &&
        std::filesystem::equivalent(result.device, result.expected_device, error) && !error;
    if (!result.matched && !allow_mismatch)
        throw std::runtime_error("camera identity differs/unavailable; inspect by-id mapping or explicitly use --allow-camera-mismatch for limited operation");
    return result;
}

namespace {
class ReplayImage final : public FrameSource {
public:
    ReplayImage(const std::string& path, cv::Size size) : image_(cv::imread(path)) {
        if (image_.empty() || image_.size() != size)
            throw std::runtime_error("replay image missing or resolution differs from calibration");
    }
    bool read(Frame& frame, std::chrono::milliseconds timeout) override {
        std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(33)));
        frame = {image_.clone(), Clock::now()};
        return true;
    }
private:
    cv::Mat image_;
};
}
std::unique_ptr<FrameSource> open_replay_image(const std::string& path, cv::Size size) {
    return std::make_unique<ReplayImage>(path, size);
}
}  // namespace xtnetrc::camera
