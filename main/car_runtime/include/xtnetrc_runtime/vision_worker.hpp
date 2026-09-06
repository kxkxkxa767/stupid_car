#pragma once
#include "xtnetrc_camera/frame_source.hpp"
#include "xtnetrc_runtime/control.hpp"
#include "xtnetrc_visual_distance/lane_corridor.hpp"
#include <atomic>
#include <mutex>
#include <thread>

namespace xtnetrc::runtime {
double monotonic_seconds(camera::Clock::time_point time = camera::Clock::now());
struct VisionSnapshot {
    LaneObservation lane;
    std::string error;
    cv::Mat overlay;
};
// Single-slot latest-frame mailbox: camera/vision never runs on the control
// thread. Queue age is retained all the way from kernel capture to actuation.
class VisionWorker {
public:
    VisionWorker(std::unique_ptr<camera::FrameSource> source,
                 vision::GroundProjectionConfig calibration);
    ~VisionWorker();
    VisionSnapshot snapshot() const;
    void stop();
private:
    void work();
    std::unique_ptr<camera::FrameSource> source_;
    visual_distance::LaneCorridorDetector detector_;
    mutable std::mutex mutex_;
    VisionSnapshot latest_;
    std::atomic_bool running_{true};
    std::thread worker_;
};
}  // namespace xtnetrc::runtime
