# main 主程序目录

该目录对应原车 `/home/5G/5G/main/`，每个子目录尽量保持可单独编译。

- `vision/`：C++17 + OpenCV 车道视觉程序。
- `car_runtime/`：统一循迹、运动控制、GPS PI 和安全输出入口；默认不驱动。
- `hardware/`：共享串口/车辆驱动接口、pigpio 后端和独立命令看门狗。
- `camera_io/`：按标定身份选相机、有限等待采集、原图分辨率校验与离线源。
- `gps_speed_control/`：WIT 解析、GPS 估计、速度 PI 与只读诊断。
- `motion_control/`：C++17 Pure Pursuit 运动控制核心，当前不输出 GPIO/PWM。
- `camera_gimbal/`：C++17 双轴摄像头云台控制，含实测限位、平滑移动和 pigpio 后端。
- `config/`：实车几何参数和后续标定记录。

联调从 `car_runtime/README.md` 开始；旧固定 PWM 实验已收敛到统一入口。
不要修改原车 `studyroad` 备份，也不要将本地构建产物加入 Git。
