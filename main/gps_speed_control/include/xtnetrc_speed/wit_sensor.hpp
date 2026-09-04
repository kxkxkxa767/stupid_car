#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace xtnetrc::speed {

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct WitSensorData {
    std::optional<Vector3> acceleration_mps2;
    std::optional<Vector3> angular_velocity_rad_s;
    std::optional<Vector3> euler_deg;

    std::optional<double> latitude_deg;
    std::optional<double> longitude_deg;
    std::optional<double> gps_altitude_m;
    std::optional<double> gps_course_deg;
    std::optional<double> gps_speed_mps;
    std::optional<int> satellites;
    std::optional<double> pdop;
    std::optional<double> hdop;
    std::optional<double> vdop;

    std::uint64_t valid_frames{0};
    std::uint64_t bad_checksums{0};
};

// 解析 WIT 标准 11 字节主动输出协议。该类只处理字节，不打开串口。
class WitFrameParser {
public:
    // 接受一个字节。完整且校验正确的帧返回帧类型，否则返回 nullopt。
    [[nodiscard]] std::optional<std::uint8_t> feed(std::uint8_t byte) noexcept;

    [[nodiscard]] const WitSensorData& data() const noexcept { return data_; }
    void reset() noexcept;

private:
    void decode_frame() noexcept;
    void recover_after_bad_checksum() noexcept;

    std::array<std::uint8_t, 11> frame_{};
    std::size_t frame_size_{0};
    WitSensorData data_{};
};

}  // namespace xtnetrc::speed
