#include "xtnetrc_runtime/vision_worker.hpp"
#include <stdexcept>
#include <utility>
namespace xtnetrc::runtime {
double monotonic_seconds(camera::Clock::time_point time) {
    return std::chrono::duration<double>(time.time_since_epoch()).count();
}
VisionWorker::VisionWorker(std::unique_ptr<camera::FrameSource> source,
                           vision::GroundProjectionConfig calibration)
    : source_(std::move(source)), detector_(std::move(calibration)) {
    if (!source_) throw std::invalid_argument("missing frame source");
    worker_ = std::thread(&VisionWorker::work, this);
}
VisionWorker::~VisionWorker() { stop(); }
void VisionWorker::stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
}
VisionSnapshot VisionWorker::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}
void VisionWorker::work() {
    try {
        std::uint64_t sequence = 0;
        while (running_) {
            camera::Frame frame;
            if (!source_->read(frame, std::chrono::milliseconds(100))) continue;
            if (!running_) break;
            const auto lane = detector_.process(frame.image);
            VisionSnapshot value;
            value.lane = {++sequence, monotonic_seconds(frame.captured_at), lane.valid,
                          lane.reference_x_m, lane.center_y_m, lane.heading_slope,
                          lane.width_m, lane.confidence};
            value.overlay = lane.overlay;
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(value);
        }
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_.error = error.what();
        latest_.lane.valid = false;
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_.error = "unknown perception failure";
        latest_.lane.valid = false;
    }
}
}  // namespace xtnetrc::runtime
