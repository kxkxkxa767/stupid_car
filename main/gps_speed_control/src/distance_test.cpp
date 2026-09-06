#include <iostream>
int main() {
    std::cerr << "旧 GPS 固定 PWM 行驶入口已停用；没有输出任何电机信号。\n"
                 "请使用 xtnetrc_car_runtime --config main/config/runtime_2023.json，\n"
                 "统一运行循迹、运动控制、GPS 速度 PI 和独立超时保护。\n"
                 "默认只读；先完成 dry-run。旧实验可从 Git 历史查看。\n";
    return 2;
}
