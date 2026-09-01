# XT-NetRC 独立 C++ 运动控制模块

本目录暂时与视觉主函数分离，使用 C++17 实现 Pure Pursuit 路径跟踪核心、离线演示和单元测试。模块只计算目标转角与目标速度，不包含 OpenCV、GPIO、PWM 或电调输出。

坐标约定：`x` 向车辆前方、`y` 向车辆左侧，距离单位为米，正转角表示左转。

目录结构：

```text
main/motion_control/
├── include/xtnetrc_motion/controller.hpp  对外接口和参数
├── src/controller.cpp                     Pure Pursuit 与安全策略
├── src/config_loader.cpp                  严格读取 JSON 运动参数
├── src/demo.cpp                           不接硬件的合成路径演示
├── tests/controller_test.cpp              独立算法测试
├── config/motion_default.json             参数记录与标定状态
└── CMakeLists.txt                          独立构建入口
```

编译和测试：

```bash
cd /Users/yuhaojin/Documents/5G_opencv
cmake -S main/motion_control -B build/motion-control -DCMAKE_BUILD_TYPE=Release
cmake --build build/motion-control --parallel
ctest --test-dir build/motion-control --output-on-failure
./build/motion-control/xtnetrc_motion_demo \
  --config main/motion_control/config/motion_default.json left
```

在 Mac 上交叉编译树莓派 ARM64 版本：

```bash
cd /Users/yuhaojin/Documents/5G_opencv
./scripts/build_pi_motion_cross.sh
```

结果位于 `build/pi-aarch64-motion/xtnetrc_motion_demo`。上传时使用独立测试目录，
不会覆盖树莓派已有视觉识别程序：

```bash
./scripts/deploy_pi_motion_staging.sh 5G@192.168.124.5
ssh 5G@192.168.124.5 \
  'cd /home/5G/xtnetrc_staging/motion_control && \
   ./xtnetrc_motion_demo --config ./motion_default.json left'
```

后续主程序只需链接 `xtnetrc_motion` 静态库并包含
`xtnetrc_motion/controller.hpp`，目前尚未建立该链接。

已实现的安全边界包括：全部配置有限值检查、路径米制范围与排序检查、
前视圆插值、车辆最大可实现曲率检查、置信度迟滞、丢线后连续 3 帧恢复确认、
正常速度斜坡，以及失线/关闭时不经斜坡的立即零输出。

`main/camera_motion_validation` 是单独的树莓派本地验证程序。它读取真实摄像头
置信度，但只配合明确标注的合成直线路径检查安全门控，不访问 GPIO/PWM，
也不会把尚未标定的像素误差当成米制路径。

后续接入主程序前，还需要完成：

1. 相机标定和鸟瞰变换，将像素车道线转换为车辆坐标系下的米制路径。
2. 舵机 PWM 与前轮转角的对应关系标定。
3. 电调中位、前进方向与目标车速的对应关系标定。
4. 负载状态下滚动周长或其他车速反馈标定。
