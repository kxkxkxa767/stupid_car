#include "xtnetrc_runtime/vision_worker.hpp"
#include <fstream>
#include <filesystem>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <opencv2/imgproc.hpp>
using namespace std::chrono_literals;
namespace rt = xtnetrc::runtime;
namespace cam = xtnetrc::camera;
void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
struct TimedSource final : cam::FrameSource {
    bool read(cam::Frame& frame, std::chrono::milliseconds wait) override {
        std::this_thread::sleep_for(wait);
        frame = {cv::Mat::zeros(480,640,CV_8UC3), cam::Clock::now() - 2s};
        return true;
    }
};
int main() try {
    const auto config = rt::load_runtime_config(XTNETRC_RUNTIME_CONFIG);
    check(config.board.steering_center == 72 && !config.actuator_calibration_verified,
          "measured center or unverified status changed");
    const auto calibration = xtnetrc::vision::load_ground_projection_json(config.calibration_path);
    check(calibration.camera_role == "front_fixed" && calibration.camera_device_by_id ==
          "/dev/v4l/by-id/usb-XWF_1080P_PC_Camera_XWF_1080P_PC_Camera_240122004-video-index0",
          "fixed XWF metadata missing or gimbal calibration selected");
    check(calibration.projection_model == "raw_pixel_homography", "raw projection mode missing");
    {
        // End-to-end: rendered lane -> real OpenCV detector -> metric path ->
        // motion controller -> GPS PI -> bounded command (never real GPIO).
        auto synthetic = calibration;
        synthetic.image_to_vehicle_ground = {0,-0.008,3.0, -0.003,0,0.96, 0,0,1};
        synthetic.dist_coeffs = {0,0,0,0,0};
        cv::Mat frame(480,640,CV_8UC3,cv::Scalar(30,30,30));
        for (int x : {215,225,415,425})
            cv::line(frame,{x,160},{x,300},cv::Scalar(240,240,240),3);
        // Adjacent-lane and transverse distractors must not become our centerline.
        for (int x : {25,615}) cv::line(frame,{x,160},{x,300},cv::Scalar(240,240,240),3);
        cv::line(frame,{30,240},{610,240},cv::Scalar(240,240,240),2);
        xtnetrc::visual_distance::LaneCorridorDetector detector(synthetic);
        auto lane = detector.process(frame);
        check(lane.valid && std::abs(lane.center_y_m) < 0.04 && std::abs(lane.width_m - 0.6) < 0.05,
              "synthetic own-lane geometry failed");
        rt::ControlPipeline pipeline(config);
        rt::RuntimeOutput output;
        for (int i=1; i<=10; ++i) {
            const double now = 1 + i*0.05;
            output = pipeline.update({static_cast<std::uint64_t>(i), now, lane.valid,
                lane.reference_x_m,lane.center_y_m,lane.heading_slope,lane.width_m,lane.confidence},
                {true,0,0,8,1,"mock GPS"},now,0.05,true);
        }
        check(output.drive && output.speed.enabled, "real perception did not feed unified controller");
        check(!detector.process(cv::Mat::zeros(480,640,CV_8UC3)).valid,"blank frame accepted as lane");
    }
    {
        rt::VisionWorker worker(std::make_unique<TimedSource>(),calibration);
        const auto start = cam::Clock::now();
        for (int i=0; i<100; ++i) (void)worker.snapshot();
        check(cam::Clock::now() - start < 100ms, "camera blocks control mailbox");
        rt::VisionSnapshot snapshot;
        const auto deadline = cam::Clock::now() + 2s;
        do { std::this_thread::sleep_for(20ms); snapshot = worker.snapshot(); }
        while (!snapshot.lane.sequence && snapshot.error.empty() && cam::Clock::now() < deadline);
        check(snapshot.error.empty() && snapshot.lane.sequence > 0, "worker failed to publish");
        check(rt::monotonic_seconds() - snapshot.lane.captured_s >= 2,
              "worker replaced capture timestamp with processing timestamp");
    }
    auto raw = calibration;
    raw.dist_coeffs[0] = 0.2;
    bool rejected = false; try { raw.validate(); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected,"raw projection accepted nonzero distortion");
    {
        // Strict configuration: reject typos/duplicates/bad types; paths are relative to the
        // config file, not the current process working directory.
        std::ifstream input(XTNETRC_RUNTIME_CONFIG);
        const std::string original((std::istreambuf_iterator<char>(input)), {});
        const auto dir = std::filesystem::path(XTNETRC_RUNTIME_CONFIG).parent_path();
        // Test fixtures live in a unique temporary directory; point referenced
        // files at absolute paths so malformed-key tests reach schema checking.
        const auto temp = std::filesystem::temp_directory_path() /
            ("xtnetrc-config-test-" + std::to_string(cam::Clock::now().time_since_epoch().count()));
        std::filesystem::create_directory(temp);
        struct Cleanup { std::filesystem::path path; ~Cleanup() { std::error_code e; std::filesystem::remove_all(path,e); } } cleanup{temp};
        auto base = original;
        for (const std::string relative : {"../motion_control/config/motion_default.json", "../camera_calibration/output/ground_projection.json"}) {
            auto position = base.find(relative);
            check(position != std::string::npos,"test config path missing");
            base.replace(position,relative.size(),(dir / relative).lexically_normal().string());
        }
        for (int test_case = 0; test_case < 3; ++test_case) {
            auto text = base;
            const auto position = text.find("target_speed_mps");
            if (test_case == 0) text.replace(position,std::string("target_speed_mps").size(),"target_speed_typo");
            if (test_case == 1) text.insert(position-1,"\"target_speed_mps\": 0.1,\n    ");
            if (test_case == 2) {
                const auto value_position = text.find("0.20",position);
                check(value_position != std::string::npos,"test numeric fixture missing");
                text.replace(value_position,4,"\"NaN\"");
            }
            const auto fixture = temp / (std::to_string(test_case) + ".json");
            std::ofstream(fixture) << text;
            bool failed = false;
            try { (void)rt::load_runtime_config(fixture.string()); } catch (const std::exception&) { failed = true; }
            check(failed,"unknown/duplicate/bad-type config accepted");
        }
    }
    std::cout << "config/camera metadata/async perception/timestamp tests passed\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
