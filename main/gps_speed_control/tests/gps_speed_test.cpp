#include "xtnetrc_speed/speed_loop.hpp"
#include "xtnetrc_speed/wit_sensor.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

namespace speed = xtnetrc::speed;

namespace {

int failures = 0;

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "失败: " << message << '\n';
    }
}

void put_i16(std::array<std::uint8_t, 11>& frame, const std::size_t offset,
             const std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    frame[offset] = static_cast<std::uint8_t>(raw & 0xffU);
    frame[offset + 1] = static_cast<std::uint8_t>((raw >> 8U) & 0xffU);
}

void put_i32(std::array<std::uint8_t, 11>& frame, const std::size_t offset,
             const std::int32_t value) {
    const auto raw = static_cast<std::uint32_t>(value);
    for (std::size_t index = 0; index < 4; ++index) {
        frame[offset + index] = static_cast<std::uint8_t>((raw >> (8U * index)) & 0xffU);
    }
}

std::array<std::uint8_t, 11> make_frame(const std::uint8_t type) {
    std::array<std::uint8_t, 11> frame{};
    frame[0] = 0x55;
    frame[1] = type;
    return frame;
}

void finish_checksum(std::array<std::uint8_t, 11>& frame) {
    std::uint8_t sum = 0;
    for (std::size_t index = 0; index < 10; ++index) {
        sum = static_cast<std::uint8_t>(sum + frame[index]);
    }
    frame[10] = sum;
}

void feed(speed::WitFrameParser& parser, const std::array<std::uint8_t, 11>& frame) {
    for (const std::uint8_t byte : frame) {
        static_cast<void>(parser.feed(byte));
    }
}

void test_wit_decoding() {
    speed::WitFrameParser parser;
    auto quality = make_frame(0x5A);
    put_i16(quality, 2, 8);
    put_i16(quality, 4, 150);
    put_i16(quality, 6, 170);
    put_i16(quality, 8, 220);
    finish_checksum(quality);
    feed(parser, quality);
    expect(parser.data().satellites == 8, "应解析卫星数");
    expect(parser.data().hdop && std::abs(*parser.data().hdop - 1.7) < 1e-12,
           "应解析 HDOP");

    auto navigation = make_frame(0x58);
    put_i16(navigation, 2, 512);
    put_i16(navigation, 4, 12345);
    put_i32(navigation, 6, 3600);
    finish_checksum(navigation);
    feed(parser, navigation);
    expect(parser.data().gps_speed_mps &&
               std::abs(*parser.data().gps_speed_mps - 1.0) < 1e-12,
           "GPSV=3600 应换算为 1 m/s");
    expect(parser.data().gps_course_deg &&
               std::abs(*parser.data().gps_course_deg - 123.45) < 1e-12,
           "应解析 GPS 航向");

    auto position = make_frame(0x57);
    put_i32(position, 2, 1172051088);
    put_i32(position, 6, 315633743);
    finish_checksum(position);
    feed(parser, position);
    expect(parser.data().latitude_deg &&
               std::abs(*parser.data().latitude_deg - 31.9389571667) < 1e-8,
           "应把纬度的度分格式转换为十进制度");

    navigation[10] ^= 0xffU;
    feed(parser, navigation);
    expect(parser.data().bad_checksums == 1, "坏校验帧必须计数并丢弃");
}

void test_speed_estimator_quality_and_timeout() {
    speed::GpsImuSpeedEstimator estimator;
    expect(!estimator.update_gps(0.5, 4, 1.0, 0.0), "卫星不足应拒绝");
    expect(!estimator.update_gps(0.5, 8, 4.0, 0.0), "HDOP 过高应拒绝");
    expect(estimator.update_gps(0.5, 8, 1.5, 0.0), "合格 GPS 应接收");
    expect(estimator.estimate(0.1).valid, "新鲜 GPS 估计应有效");
    expect(!estimator.estimate(3.0).valid, "GPS 超时必须失效");
}

void test_speed_estimator_filter_and_imu_prediction() {
    speed::SpeedEstimatorConfig config;
    config.gps_correction_gain = 1.0;
    config.acceleration_deadband_mps2 = 0.0;
    speed::GpsImuSpeedEstimator estimator(config);
    expect(estimator.update_gps(0.4, 8, 1.5, 0.0), "首个 GPS 应接收");
    expect(estimator.update_gps(2.0, 8, 1.5, 0.2), "离群 GPS 应进入窗口");
    expect(estimator.update_gps(0.5, 8, 1.5, 0.4), "第三个 GPS 应接收");
    expect(std::abs(estimator.estimate(0.4).speed_mps - 0.5) < 1e-12,
           "中值滤波应压制单点离群值");
    estimator.update_acceleration(1.0, 0.4);
    estimator.update_acceleration(1.0, 0.5);
    expect(std::abs(estimator.estimate(0.5).speed_mps - 0.6) < 1e-12,
           "两次 GPS 之间应积分前向加速度");
}

void test_pi_safety_and_anti_windup() {
    speed::SpeedPiController controller;
    speed::SpeedEstimate invalid;
    expect(!controller.update(0.5, invalid, 0.2).enabled,
           "无有效速度反馈时 PI 必须禁用");
    speed::SpeedEstimate valid{true, 0.0, 0.0, 8, 1.5, "ok"};
    const auto output = controller.update(1.0, valid, 0.2);
    expect(output.enabled && output.pwm_trim == 8.0, "PWM 修正应受上限约束");
    expect(!controller.update(0.0, valid, 0.2).enabled,
           "目标为零时 PI 必须清零并禁用");

    bool threw = false;
    try {
        static_cast<void>(controller.update(1.0, valid,
                                             std::numeric_limits<double>::quiet_NaN()));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "非法控制周期必须拒绝");
}

}  // namespace

int main() {
    test_wit_decoding();
    test_speed_estimator_quality_and_timeout();
    test_speed_estimator_filter_and_imu_prediction();
    test_pi_safety_and_anti_windup();
    if (failures != 0) {
        std::cerr << failures << " 项 GPS 速度控制测试失败\n";
        return 1;
    }
    std::cout << "GPS 速度控制测试全部通过\n";
    return 0;
}
