# main 主程序目录

该目录对应原车 `/home/5G/5G/main/`，每个子目录尽量保持可单独编译。

- `vision/`：C++17 + OpenCV 车道视觉程序。
- `motion_control/`：C++17 Pure Pursuit 运动控制核心，当前不输出 GPIO/PWM。
- `camera_gimbal/`：C++17 双轴摄像头云台控制，含实测限位、平滑移动和 pigpio 后端。
- `config/`：实车几何参数和后续标定记录。

后续视觉与运动控制联调时，在 `main/`下新建独立的集成子工程，不直接修改原车 `studyroad`备份。
