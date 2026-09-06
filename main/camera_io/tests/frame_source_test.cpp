#include "xtnetrc_camera/frame_source.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
namespace fs = std::filesystem;
namespace cam = xtnetrc::camera;
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
template<class F> void rejects(F fn, const char* message) {
    bool threw = false; try { fn(); } catch (const std::exception&) { threw = true; }
    check(threw, message);
}
struct Temp {
    fs::path path = fs::temp_directory_path() / ("xtnetrc-camera-test-" +
        std::to_string(cam::Clock::now().time_since_epoch().count()));
    Temp() { if (!fs::create_directory(path)) throw std::runtime_error("test temp create failed"); }
    ~Temp() { std::error_code error; fs::remove_all(path, error); }
};
int main() try {
    Temp temp;
    const auto h65 = temp.path / "h65", gimbal = temp.path / "gimbal", byid = temp.path / "fixed-by-id";
    std::ofstream(h65) << "mock physical camera";
    std::ofstream(gimbal) << "mock other camera";
    fs::create_symlink(h65, byid);
    fs::create_symlink(h65, temp.path / "video0");
    fs::create_symlink(gimbal, temp.path / "video2");
    xtnetrc::vision::GroundProjectionConfig config;
    config.camera_role = "front_fixed"; config.camera_device_by_id = byid.string();
    check(cam::check_identity(config, (temp.path / "video0").string()).matched, "same camera numbered 0 rejected");
    rejects([&] { cam::check_identity(config, (temp.path / "video2").string()); }, "gimbal accepted silently");
    const auto override = cam::check_identity(config, (temp.path / "video2").string(), true);
    check(!override.matched && override.device == (temp.path / "video2").string(), "explicit mismatch override not reported");
    // Swap enumeration while retaining the physical by-id identity.
    fs::remove(temp.path / "video0"); fs::remove(temp.path / "video2");
    fs::create_symlink(gimbal, temp.path / "video0"); fs::create_symlink(h65, temp.path / "video2");
    check(cam::check_identity(config, (temp.path / "video2").string()).matched, "renumbered fixed camera rejected");
    rejects([&] { cam::check_identity(config, (temp.path / "video0").string()); }, "renumbered wrong camera accepted");
    config.camera_device_by_id.clear();
    rejects([&] { cam::check_identity(config, ""); }, "missing metadata silently selected camera0");
    const auto image = temp.path / "frame.png";
    cv::imwrite(image.string(), cv::Mat::zeros(480, 640, CV_8UC3));
    rejects([&] { cam::open_replay_image(image.string(), {320, 240}); }, "resolution mismatch resized silently");
    auto replay = cam::open_replay_image(image.string(), {640, 480});
    cam::Frame frame;
    check(replay->read(frame, std::chrono::milliseconds(1)) && frame.image.size() == cv::Size(640,480), "replay failed");
    std::cout << "camera identity swap/override/resolution tests passed\n";
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
