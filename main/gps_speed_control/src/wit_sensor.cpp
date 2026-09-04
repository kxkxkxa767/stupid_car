#include "xtnetrc_speed/wit_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace xtnetrc::speed {
namespace {

constexpr double kGravityMps2 = 9.80665;
constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] std::int16_t read_i16(const std::uint8_t* bytes) noexcept {
    const std::uint16_t value = static_cast<std::uint16_t>(bytes[0]) |
                                (static_cast<std::uint16_t>(bytes[1]) << 8U);
    return static_cast<std::int16_t>(value);
}

[[nodiscard]] std::uint16_t read_u16(const std::uint8_t* bytes) noexcept {
    return static_cast<std::uint16_t>(bytes[0]) |
           (static_cast<std::uint16_t>(bytes[1]) << 8U);
}

[[nodiscard]] std::int32_t read_i32(const std::uint8_t* bytes) noexcept {
    const std::uint32_t value = static_cast<std::uint32_t>(bytes[0]) |
                                (static_cast<std::uint32_t>(bytes[1]) << 8U) |
                                (static_cast<std::uint32_t>(bytes[2]) << 16U) |
                                (static_cast<std::uint32_t>(bytes[3]) << 24U);
    return static_cast<std::int32_t>(value);
}

[[nodiscard]] double degrees_minutes_to_degrees(const std::int32_t raw) noexcept {
    const double signed_value = static_cast<double>(raw) / 100000.0;
    const double value = std::abs(signed_value);
    const double degrees = std::floor(value / 100.0);
    const double minutes = value - degrees * 100.0;
    const double decimal = degrees + minutes / 60.0;
    return std::copysign(decimal, signed_value);
}

}  // namespace

std::optional<std::uint8_t> WitFrameParser::feed(const std::uint8_t byte) noexcept {
    if (frame_size_ == 0 && byte != 0x55U) {
        return std::nullopt;
    }
    frame_[frame_size_++] = byte;
    if (frame_size_ < frame_.size()) {
        return std::nullopt;
    }

    std::uint8_t checksum = 0;
    for (std::size_t index = 0; index < frame_.size() - 1; ++index) {
        checksum = static_cast<std::uint8_t>(checksum + frame_[index]);
    }
    if (checksum != frame_.back()) {
        ++data_.bad_checksums;
        recover_after_bad_checksum();
        return std::nullopt;
    }

    const std::uint8_t type = frame_[1];
    decode_frame();
    ++data_.valid_frames;
    frame_size_ = 0;
    return type;
}

void WitFrameParser::reset() noexcept {
    frame_.fill(0);
    frame_size_ = 0;
    data_ = {};
}

void WitFrameParser::recover_after_bad_checksum() noexcept {
    const auto begin = frame_.begin() + 1;
    const auto next_header = std::find(begin, frame_.end(), 0x55U);
    if (next_header == frame_.end()) {
        frame_size_ = 0;
        return;
    }
    const std::size_t remaining = static_cast<std::size_t>(frame_.end() - next_header);
    std::memmove(frame_.data(), &*next_header, remaining);
    frame_size_ = remaining;
}

void WitFrameParser::decode_frame() noexcept {
    const std::uint8_t* payload = frame_.data() + 2;
    switch (frame_[1]) {
        case 0x51: {
            constexpr double scale = 16.0 * kGravityMps2 / 32768.0;
            data_.acceleration_mps2 = Vector3{read_i16(payload) * scale,
                                              read_i16(payload + 2) * scale,
                                              read_i16(payload + 4) * scale};
            break;
        }
        case 0x52: {
            constexpr double scale = 2000.0 * kPi / (32768.0 * 180.0);
            data_.angular_velocity_rad_s = Vector3{read_i16(payload) * scale,
                                                   read_i16(payload + 2) * scale,
                                                   read_i16(payload + 4) * scale};
            break;
        }
        case 0x53: {
            constexpr double scale = 180.0 / 32768.0;
            data_.euler_deg = Vector3{read_i16(payload) * scale,
                                      read_i16(payload + 2) * scale,
                                      read_i16(payload + 4) * scale};
            break;
        }
        case 0x57: {
            const std::int32_t longitude = read_i32(payload);
            const std::int32_t latitude = read_i32(payload + 4);
            if (longitude != 0 && latitude != 0) {
                data_.longitude_deg = degrees_minutes_to_degrees(longitude);
                data_.latitude_deg = degrees_minutes_to_degrees(latitude);
            }
            break;
        }
        case 0x58: {
            data_.gps_altitude_m = read_i16(payload) / 10.0;
            data_.gps_course_deg = read_u16(payload + 2) / 100.0;
            const std::int32_t speed_raw = read_i32(payload + 4);
            if (speed_raw >= 0) {
                // WIT GPSV 的缩放值为 0.001 km/h，控制模块统一换算为 m/s。
                data_.gps_speed_mps = speed_raw / 1000.0 / 3.6;
            }
            break;
        }
        case 0x5A: {
            data_.satellites = static_cast<int>(read_i16(payload));
            data_.pdop = read_i16(payload + 2) / 100.0;
            data_.hdop = read_i16(payload + 4) / 100.0;
            data_.vdop = read_i16(payload + 6) / 100.0;
            break;
        }
        default:
            break;
    }
}

}  // namespace xtnetrc::speed
