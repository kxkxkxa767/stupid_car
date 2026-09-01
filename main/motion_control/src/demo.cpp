#include "xtnetrc_motion/controller.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace motion = xtnetrc::motion;

int main(int argc, char* argv[]) {
    // 合成路径采用车辆坐标系，按由近到远排列，单位为米。
    const std::map<std::string, std::vector<motion::PathPoint>> scenarios{
        {"straight", {{0.20, 0.00}, {0.45, 0.00}, {0.70, 0.00}}},
        {"left", {{0.20, 0.03}, {0.42, 0.14}, {0.65, 0.30}}},
        {"right", {{0.20, -0.03}, {0.42, -0.14}, {0.65, -0.30}}},
        {"lost", {}},
    };

    std::string config_path;
    int argument = 1;
    if (argc >= 3 && std::string(argv[1]) == "--config") {
        config_path = argv[2];
        argument = 3;
    }
    const std::string scenario_name = argc > argument ? argv[argument] : "straight";
    const double confidence = argc > argument + 1
                                  ? std::strtod(argv[argument + 1], nullptr)
                                  : 0.90;
    const auto scenario = scenarios.find(scenario_name);
    if (scenario == scenarios.end()) {
        std::cerr << "未知场景。可选：straight、left、right、lost\n";
        return 2;
    }

    // 演示只调用算法库，永远不会向舵机或电调发送命令。
    const motion::MotionConfig config = config_path.empty()
                                            ? motion::MotionConfig{}
                                            : motion::load_motion_config_json(config_path);
    motion::MotionController controller(config);
    const motion::ControlCommand command =
        controller.update(scenario->second, confidence);

    std::cout << std::fixed << std::setprecision(3)
              << "场景: " << scenario_name << '\n'
              << "状态: " << motion::to_string(command.state) << '\n'
              << "前轮目标转角: " << command.steering_deg() << " deg\n"
              << "路径曲率: " << command.curvature_inv_m << " 1/m\n"
              << "目标速度: " << command.target_speed_mps << " m/s\n"
              << "置信度: " << command.confidence << '\n'
              << "状态原因: " << command.reason << '\n'
              << "硬件输出: disabled\n";
    return 0;
}
