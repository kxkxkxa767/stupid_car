#include "xtnetrc_motion/controller.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace motion = xtnetrc::motion;

namespace {
int failures = 0;

void expect(const bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "失败: " << message << '\n';
    }
}

void expect_lost(const motion::ControlCommand& command, const std::string& message) {
    expect(command.state == motion::ControlState::Lost &&
               command.target_speed_mps == 0.0 && command.steering_rad == 0.0,
           message);
}

void expect_invalid_config(motion::MotionConfig config, const std::string& message) {
    bool threw = false;
    try {
        motion::MotionController controller(config);
        static_cast<void>(controller);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, message);
}

void test_straight_and_curve() {
    motion::MotionController controller;
    const auto straight = controller.update({{0.2, 0.0}, {0.5, 0.0}}, 0.9);
    const auto left = controller.update({{0.20, 0.03}, {0.42, 0.14}}, 0.9);
    const auto right = controller.update({{0.20, -0.03}, {0.42, -0.14}}, 0.9);
    expect(straight.state == motion::ControlState::Tracking, "直道应正常跟踪");
    expect(std::abs(straight.steering_deg()) < 1e-9, "直道转角应为零");
    expect(left.steering_deg() > 0.0, "左弯转角应为正");
    expect(right.steering_deg() < 0.0, "右弯转角应为负");
    expect(left.target_speed_mps < straight.target_speed_mps, "弯道应比直道慢");
}

void test_target_interpolation() {
    motion::MotionController controller;
    const auto command = controller.update({{0.10, 0.0}, {0.80, 0.0}}, 0.9);
    expect(command.target_point_m.has_value(), "应返回前视点");
    if (command.target_point_m) {
        expect(std::abs(std::hypot(command.target_point_m->x_m,
                                   command.target_point_m->y_m) -
                        controller.config().lookahead_m) < 1e-9,
               "前视点应插值到前视圆，而非跳到离散点");
    }
}

void test_path_validation_and_curvature_cap() {
    motion::MotionController controller;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    expect_lost(controller.update({{0.5, nan}}, 0.9), "NaN 路径必须停车");
    expect_lost(controller.update({{-0.5, 0.0}}, 0.9), "车后路径必须停车");
    expect_lost(controller.update({{0.5, 0.0}, {0.2, 0.0}}, 0.9),
                "未按由近到远排列的路径必须停车");
    expect_lost(controller.update({{1e200, 1e200}}, 0.9),
                "巨大坐标不能因平方溢出而按直道行驶");
    motion::MotionController tight_curve_controller;
    expect_lost(tight_curve_controller.update({{0.10, 0.44}}, 0.9),
                "超过机械转向能力的曲率必须停车");
}

void test_confidence_hysteresis_and_recovery() {
    motion::MotionController controller;
    expect(controller.update({{0.5, 0.0}}, 0.61).state == motion::ControlState::Tracking,
           "高置信度应进入正常跟踪");
    expect(controller.update({{0.5, 0.0}}, 0.52).state == motion::ControlState::Tracking,
           "正常状态在迟滞带内不应抖动");
    expect(controller.update({{0.5, 0.0}}, 0.49).state == motion::ControlState::Degraded,
           "低于正常退出阈值应降级");
    expect(controller.update({{0.5, 0.0}}, 0.58).state == motion::ControlState::Degraded,
           "降级状态在迟滞带内不应抖动");
    expect(controller.update({{0.5, 0.0}}, 0.61).state == motion::ControlState::Tracking,
           "超过正常进入阈值才恢复正常");
    expect_lost(controller.update({{0.5, 0.0}}, 0.1), "低置信度必须立即停车");
    expect_lost(controller.update({{0.5, 0.0}}, 0.9), "恢复第 1 帧不得起步");
    expect_lost(controller.update({{0.5, 0.0}}, 0.9), "恢复第 2 帧不得起步");
    expect(controller.update({{0.5, 0.0}}, 0.9).state == motion::ControlState::Tracking,
           "连续确认达到配置帧数后才可恢复");
}

void test_rate_limits_and_emergency_stop() {
    motion::MotionConfig config;
    config.max_steering_rate_deg_s = 100.0;
    config.max_accel_mps2 = 0.8;
    config.max_decel_mps2 = 0.1;
    motion::MotionController controller(config);
    const auto first = controller.update({{0.5, 0.0}}, 0.9, 0.1);
    const auto second = controller.update({{0.5, 0.0}}, 0.9, 0.1);
    expect(std::abs(first.target_speed_mps - 0.08) < 1e-9,
           "起步速度必须服从加速度限制");
    expect(std::abs(second.target_speed_mps - 0.16) < 1e-9,
           "连续周期应平滑加速");
    motion::MotionController steering_controller(config);
    const auto steering =
        steering_controller.update({{0.20, 0.03}, {0.42, 0.14}}, 0.9, 0.05);
    expect(std::abs(steering.steering_deg()) <= 5.0 + 1e-9,
           "转角变化率限制失效");

    motion::MotionConfig decel_config = config;
    decel_config.max_accel_mps2 = 10.0;
    motion::MotionController decel_controller(decel_config);
    const auto fast = decel_controller.update({{0.5, 0.0}}, 0.9, 0.1);
    const auto slowing =
        decel_controller.update({{0.20, 0.03}, {0.42, 0.14}}, 0.9, 0.1);
    expect(fast.target_speed_mps == decel_config.max_speed_mps,
           "测试前应达到最大速度");
    expect(slowing.target_speed_mps >= fast.target_speed_mps - 0.011,
           "正常减速应服从减速度限制");
    expect_lost(decel_controller.update({{0.5, 0.0}}, 0.1, 0.1),
                "安全停车不能被减速度斜坡延迟");
}

void test_invalid_config_and_dt() {
    motion::MotionConfig config;
    config.max_speed_mps = std::numeric_limits<double>::infinity();
    expect_invalid_config(config, "无穷速度配置必须被拒绝");
    config = {};
    config.curvature_speed_gain = std::numeric_limits<double>::quiet_NaN();
    expect_invalid_config(config, "NaN 曲率增益必须被拒绝");
    config = {};
    config.max_steering_deg = 90.0;
    expect_invalid_config(config, "90 度转向配置必须被拒绝");
    config = {};
    config.confidence_hysteresis = 0.20;
    expect_invalid_config(config, "重叠的置信度迟滞带必须被拒绝");

    motion::MotionController controller;
    bool threw = false;
    try {
        static_cast<void>(controller.update({}, 0.0,
            std::numeric_limits<double>::quiet_NaN(), false));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "即使控制器关闭，无效周期也必须被一致拒绝");
}

void test_json_configuration() {
    const motion::MotionConfig config =
        motion::load_motion_config_json(XTNETRC_TEST_CONFIG_PATH);
    expect(std::abs(config.wheelbase_m - 0.23861) < 1e-9, "JSON 应实际载入轴距");
    expect(config.confidence_reacquire_frames == 3, "JSON 应载入恢复确认帧数");
    const std::string bad_path = "/tmp/xtnetrc_bad_motion_config.json";
    {
        std::ofstream output(bad_path);
        output << "{\"motion\":{\"wheelbase_m\":0.2}}";
    }
    bool threw = false;
    try {
        static_cast<void>(motion::load_motion_config_json(bad_path));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "缺少参数的 JSON 必须被拒绝");
    std::remove(bad_path.c_str());
}
}  // namespace

int main() {
    test_straight_and_curve();
    test_target_interpolation();
    test_path_validation_and_curvature_cap();
    test_confidence_hysteresis_and_recovery();
    test_rate_limits_and_emergency_stop();
    test_invalid_config_and_dt();
    test_json_configuration();
    if (failures != 0) {
        std::cerr << failures << " 项运动控制测试失败\n";
        return 1;
    }
    std::cout << "运动控制测试全部通过\n";
    return 0;
}
