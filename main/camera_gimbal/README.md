# XT-NetRC 双轴摄像头云台控制

本目录只控制带舵机的第二颗摄像头，不控制固定循迹摄像头，也不会访问车辆转向
GPIO 12 或电调 GPIO 13。代码使用 BCM 编号：GPIO 22 为水平轴，GPIO 23 为俯仰轴。

## 实车测试结果（2026-08-31）

| 轴 | 中心 | 实测边界 | 程序安全范围 | 数值方向 |
|---|---:|---:|---:|---|
| 水平 X | 72 | 25～125 | 28～122 | 减小向车辆右侧，增大向左侧 |
| 俯仰 Y | 73 | 约 55～125 | 58～122 | 减小向下，增大向上 |

俯仰从 55 继续降到 50 时，摄像头画面移动不足 1 像素，说明机构已经到达机械
下限。程序因此保留 3 个 PWM 单位余量，严禁把 50 当作可用位置。

## 代码调用

主程序长期持有控制器时使用：

```cpp
#include "xtnetrc_gimbal/gimbal.hpp"
#include "xtnetrc_gimbal/pigpio_driver.hpp"

xtnetrc::gimbal::PigpioDriver gpio;
auto config = xtnetrc::gimbal::load_gimbal_config_json(
    "config/gimbal_default.json");
xtnetrc::gimbal::GimbalController gimbal(gpio, config);

gimbal.initialize();                    // 回到 X=72、Y=73
gimbal.move_to(90, 80);                 // 自动平滑移动并强制限位
gimbal.move_relative(-5, 0);            // 水平向车辆右侧移动
gimbal.center();                         // 回中
gimbal.release();                        // 停止 GPIO 22/23 PWM
```

`move_to()` 即使收到 `1000` 或负数也只会写入安全范围。控制器析构时也会自动
调用 `release()`。

## 编译和测试

Mac 单元测试完全使用模拟 GPIO，不会移动实车：

```bash
cmake -S . -B build/mac-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/local/opencv"
cmake --build build/mac-release --parallel
ctest --test-dir build/mac-release --output-on-failure
```

树莓派上可以单独编译：

```bash
cd /home/5G/5G/main/camera_gimbal
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build --parallel
./build/xtnetrc_gimbal_cli --config config/gimbal_default.json center
./build/xtnetrc_gimbal_cli --config config/gimbal_default.json move 82 73
```

命令行程序移动完成后会短暂保持并释放 PWM，适合人工验证；正式视觉主函数应直接
链接 `xtnetrc_camera_gimbal` 并长期持有控制器。不得与另一个正在调用
`gpioInitialise()` 的原车运动程序同时运行。
