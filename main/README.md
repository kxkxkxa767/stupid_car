# main 主程序目录

这里保留今年的 C++ 框架与其依赖。自动行驶唯一入口是 `car_runtime/` 中的
`xtnetrc_car_runtime`；子模块、只读诊断和测试不是另一套旧自动行驶框架。
2026-09-07 移除旧 GPS 提示入口和重复循迹兼容入口，保留 Git 历史以便恢复。

- `vision/`：C++17 + OpenCV 车道视觉程序。
- `car_runtime/`：统一循迹、运动控制、GPS PI 和安全输出入口；默认不驱动。
- `hardware/`：共享串口/车辆驱动接口、pigpio 后端和独立命令看门狗。
- `camera_io/`：按标定身份选相机、有限等待采集、原图分辨率校验与离线源。
- `gps_speed_control/`：WIT 解析、GPS 估计、速度 PI 与只读诊断。
- `motion_control/`：C++17 Pure Pursuit 运动控制核心，当前不输出 GPIO/PWM。
- `camera_gimbal/`：C++17 双轴摄像头云台控制，含实测限位、平滑移动和 pigpio 后端。
- `config/`：实车几何参数和后续标定记录。
- `ground_projection/`：固定 XWF 的像素到地面坐标投影及地面区域掩码。
- `visual_distance_test/`：新入口使用的双边界检测库、光流库、只读诊断和离线测试；
  虽然目录名保留 test，但包含新框架依赖，不能当作旧工程整目录删除。
- `camera_calibration/`：当前标定、原始照片、身份记录和历史参数备份。
- `camera_motion_validation/`：独立的相机/运动模型验证工具。

联调从 `car_runtime/README.md` 开始；旧固定 PWM 实验已收敛到统一入口。
`main/` 没有 Python 文件。`reference/` 原车参考、`backup/` 完整备份和今年的 Python
标定工具全部保留；不修改树莓派 `/home/5G/5G`，不把构建产物加入 Git。
