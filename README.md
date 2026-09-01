# XT-NetRC Python + C++ / OpenCV 工程

这个目录用于 XT-NetRC 智能车的视觉开发。当前以 Python + OpenCV 作为主要视觉入口，同时保留 C++ 版本用于性能对比和未来硬件控制。第一阶段只做摄像头或视频的车道检测，并输出“模拟舵机角度”；默认不会给车辆发送 PWM，避免在 GPIO 编号和电调参数尚未核实前误启动车轮。

## 已确认的原车信息

- 原资料使用树莓派、OpenCV 4.2、CMake 和 pigpio。
- 原厂网页上位机由 `network-rc.service` 提供，本地端口为 8080；旧资料还使用 FRP 做公网转发。
- 电机 PWM 代码使用编号 `13`。
- 舵机代码实际写的是编号 `12`，但旁边注释写“GPIO17”，两者矛盾。接真实舵机前必须在车上确认采用的是 BCM 编号、物理引脚编号还是资料写错。
- 原 Python 案例会主动停止 `network-rc.service`，说明自动驾驶程序和原厂上位机可能争用摄像头或 PWM。当前工程不会自动停止任何系统服务。

## 目录

```text
OpenCV/python/        Python 视觉包与测试
PWM/                  后续 pigpio 执行器适配层
main/vision/          C++ OpenCV 视觉主程序和测试
main/motion_control/  C++ Pure Pursuit 运动控制核心
main/camera_gimbal/   C++ 双轴摄像头云台控制与实测安全限位
main/camera_calibration/ 相机标定照片、说明与输出参数
main/ground_projection/ C++ 像素去畸变及车辆米制坐标投影
main/config/          实车尺寸和标定参数
imu/                  后续 IMU/GPS 驱动与融合代码
tflite/               后续经验证的 TFLite 模型与推理适配层
voice/                语音功能预留目录
picsave/              调试截图输出目录
backup/               原车 /home/5G/5G 的部署前完整备份
.venv/                Mac 项目独立 Python 环境
scripts/              环境构建和树莓派部署脚本
reference/            学习资料和原车 SD 卡只读参考
third_party/          Mac 的 OpenCV 源码
build/                Mac/树莓派构建产物
local/opencv/         Mac 项目内 OpenCV 4.14
.vscode/              VS Code 编译、测试和调试配置
```

`OpenCV/`、`PWM/`、`main/`、`imu/`、`tflite/`、`voice/` 和 `picsave/` 与原车 `/home/5G/5G/` 顶层结构对应。Mac 专用环境与参考资料不会随部署脚本上传。

`reference/` 下分为：

- `01_整车与远程控制/`：原厂网页上位机手册和 5G 通信说明。
- `02_C++赛道案例/`：OpenCV/CMake、图像处理、循迹、PID、避障和触发停止案例。
- `03_电机舵机/`：原始 PWM 说明与 Python 参考代码，仅用于核对硬件参数。
- `04_Python视觉案例/`：筛选后的原厂 Python/OpenCV 示例，不含泄露密钥的云端案例。
- `05_XT-NetRC车辆规格/`：车辆参数、版本差异、轮胎/轴距实测表、控制建模说明及保存的公开 PDF。
- `06_原车SD卡程序_2025/`：从实车 SD 卡只读筛选出的循迹/识别/执行器源码和 TFLite 模型，不含视频或网络凭据。

## 实车 SD 卡已确认信息

- 该卡为 Debian 12 `bookworm` 64 位系统，程序更新到 2025-12-28。
- 实际登录用户是 `5G`，主机名是 `pi`；系统中没有 `pi` 用户。
- SSH 已启用；`127.0.1.1` 是回环地址，不是局域网 IP。
- 实车卡在 `/usr/local` 中安装了 OpenCV 4.13.0，而老学习资料使用 OpenCV 4.2。
- 原车最新集成程序使用 BCM GPIO 12 控制转向舵机、GPIO 13 控制电调，GPIO 22/23 控制云台。详细参数只用于核对，不作为未标定实车的默认输出。

## Python 视觉环境（推荐入口）

本机使用项目内 `.venv`，不会向系统 Python 安装依赖：

```bash
cd /Users/yuhaojin/Documents/5G_opencv
./scripts/setup_python_macos.sh
.venv/bin/python -m unittest discover -s OpenCV/python/tests -v
```

启动摄像头：

```bash
PYTHONPATH=OpenCV/python .venv/bin/python -m xtnetrc_vision --camera 0
```

读取视频：

```bash
PYTHONPATH=OpenCV/python .venv/bin/python -m xtnetrc_vision --video /绝对路径/测试视频.mp4
```

也可以直接在 VS Code 的“运行任务”中选择 `Python: 启动摄像头 0` 或 `Python: 运行自检`。
当前环境固定 `opencv-python 5.0.0.93`；算法只使用 OpenCV 4/5 都支持的基础接口，方便以后部署到仍使用 OpenCV 4.x 的树莓派。

## 运动控制核心（Pure Pursuit）

独立目录 `main/motion_control/` 已使用 C++17 实现不接硬件的 Pure Pursuit 路径跟踪、按曲率降速、置信度迟滞与恢复确认、失线立即停车、转角/速度变化率限制、路径输入检查和车辆可实现曲率检查。目前不与正式视觉主函数连接，也不依赖 Python 或 OpenCV。车辆坐标约定为 `x` 向前、`y` 向左，单位为米；正转角代表左转。

运行合成路径演示：

```bash
cd /Users/yuhaojin/Documents/5G_opencv
cmake -S main/motion_control -B build/motion-control -DCMAKE_BUILD_TYPE=Release
cmake --build build/motion-control --parallel
ctest --test-dir build/motion-control --output-on-failure
./build/motion-control/xtnetrc_motion_demo \
  --config main/motion_control/config/motion_default.json left
```

C++ 默认参数定义在 `main/motion_control/include/xtnetrc_motion/controller.hpp` 的 `MotionConfig` 中，`main/motion_control/config/motion_default.json` 会由演示和验证程序严格读取；缺项、重复项或非法值会拒绝启动。其中轴距 `0.23861 m` 来自实测；`25°` 最大转角和速度值目前是保守的软件初值，不是已标定的实车参数。控制器只输出前轮目标转角与目标速度，不会输出 GPIO/PWM。

视觉模块目前给出像素误差，运动控制器要求米制路径点。二者正式连接前，需要先完成相机内参、透视变换和像素到地面的尺度标定，不能把图像像素直接作为路径坐标。

## 双轴摄像头云台

`main/camera_gimbal/` 是独立的 C++17 模块。实车确认 BCM GPIO 22 为水平轴、
GPIO 23 为俯仰轴，中心值为 `X=72、Y=73`。模块内置本车实测的软件限位
`X=28～122、Y=58～122`，所有目标在写入 GPIO 前都会被限制，并按小步长平滑移动。
Mac 上的单元测试使用模拟驱动，不会访问 GPIO；树莓派后端在运行时复用系统已有的
`libpigpio`，不会要求 Mac 再安装一份依赖。详细调用方法见
`main/camera_gimbal/README.md`。

相机标定工具、可打印 ChArUco 板和 C++ 地面投影模块已经建立，操作顺序见
`main/camera_calibration/README.md`。标定 PDF 位于
`output/pdf/XT-NetRC_Charuco相机标定板_A4.pdf`。

## VS Code

在 Mac 终端运行：

```bash
code /Users/yuhaojin/Documents/5G_opencv
```

已配置 VS Code 的 `Python`、`Pylance`、`C/C++` 和 `CMake Tools` 扩展。可以使用：

- `Cmd+Shift+B`：并行生成 Mac Release、树莓派 ARM64 运动控制、视觉程序和摄像头运动安全验证器。
- “运行任务” → `Python: 运行自检`：测试 Python 视觉算法。
- “运行任务” → `Python: 启动摄像头 0`：运行当前推荐的 Python 入口。
- “运行任务” → `C++: 编译运动控制`：独立编译运动控制静态库。
- “运行任务” → `C++: 交叉编译树莓派 ARM64 运动控制`：在 Mac 生成 Linux ARM64 文件。
- “运行任务” → `C++: 交叉编译树莓派 ARM64 视觉程序`：复用实车 OpenCV sysroot 生成 Linux ARM64 文件。
- “运行任务” → `树莓派: 上传运动控制到独立测试目录`：只上传到 staging，不覆盖原视觉程序。
- “运行任务” → `树莓派: 上传交叉编译视觉程序到独立测试目录`：只验证新文件，不覆盖原程序。
- “运行任务” → `C++: 运动控制演示`：运行不接硬件的左弯路径跟踪演示。
- “运行任务” → `C++: 运动控制自检`：单独验证 C++ 运动控制模块。
- “运行任务” → `CTest: 运行自检`：执行不需要摄像头的算法测试。
- “运行任务” → `运行：Mac 摄像头应用`：启动带 macOS 相机权限声明的应用。
- “运行任务” → `树莓派: 合并部署到 5G@pi.local`：按原 SD 卡目录结构增量上传，不删除原车文件。
- CMake Tools 状态栏：在 `mac-debug` 与 `mac-release` 预设之间切换。

## macOS：安装项目内 OpenCV 4.14

```bash
cd /Users/yuhaojin/Documents/5G_opencv
./scripts/build_opencv_macos.sh
```

环境会保存在当前目录的 `third_party/`、`build/` 和 `local/` 中，不依赖 Homebrew 的 OpenCV。

## macOS：编译工程

```bash
cd /Users/yuhaojin/Documents/5G_opencv
cmake --preset mac-release
cmake --build --preset mac-release
ctest --test-dir build/mac-release --output-on-failure
```

打开 Mac 摄像头：

```bash
open "./build/mac-release/XT-NetRC Vision.app"
```

第一次启动会弹出 macOS 摄像头权限提示，请允许 `XT-NetRC Vision`。项目保留了
`xt_netrc_vision` 命令行程序，但 macOS 通常不会为未封装的裸命令行程序登记相机权限，
所以摄像头实测请优先启动上面的 `.app`。

读取视频文件：

```bash
./build/mac-release/xt_netrc_vision --video /绝对路径/测试视频.mp4
```

按 `Q` 或 `Esc` 退出。首次打开摄像头时，macOS 可能要求给终端或 VS Code 摄像头权限。

## 树莓派：使用系统 OpenCV 4

不建议照旧文档重新编译 OpenCV 4.2。先确认原车系统已有环境：

```bash
pkg-config --modversion opencv4
python3 -c 'import cv2; print(cv2.__version__)'
g++ --version
cmake --version
```

Python 视觉入口优先使用树莓派系统包，避免在车上编译大型 wheel：

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy
cd /home/5G/5G
PYTHONPATH=OpenCV/python python3 -m xtnetrc_vision --camera 0
```

如果没有 OpenCV，再安装树莓派系统提供的 4.x：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libopencv-dev
```

在 Mac 上按原 SD 卡结构合并部署：

```bash
cd /Users/yuhaojin/Documents/5G_opencv
./scripts/deploy_to_raspberry_pi.sh 5G@pi.local /home/5G/5G
```

部署脚本不使用 `--delete`，只合并新模块，不会删除原车 `main/studyroad` 等历史程序。然后在树莓派上单独编译视觉子工程：

```bash
cd /home/5G/5G/main/vision
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/xt_netrc_vision --camera 0
```

有两只 USB 摄像头时，也要尝试 `--camera 1` 或 `--camera 2`。

## Mac：交叉编译树莓派程序

树莓派是 Debian 12 ARM64。Mac 上使用 `aarch64-unknown-linux-gnu` 工具链，
可以直接生成 Linux ARM64 可执行文件；这与编译 Mac 本机程序不同。

```bash
brew tap messense/macos-cross-toolchains
brew install messense/macos-cross-toolchains/aarch64-unknown-linux-gnu
cd /Users/yuhaojin/Documents/5G_opencv
./scripts/build_pi_motion_cross.sh
```

为了保护原车已验证的视觉代码，测试程序只上传到独立 staging 目录：

```bash
./scripts/deploy_pi_motion_staging.sh 5G@192.168.124.5
ssh 5G@192.168.124.5 \
  'cd /home/5G/xtnetrc_staging/motion_control && \
   ./xtnetrc_motion_demo --config ./motion_default.json left'
```

运动控制模块可以直接交叉编译；视觉模块需要先同步树莓派 `/usr/local` 的 OpenCV
4.13.0 及其实际依赖到项目内专用 sysroot。同步只需首次
执行，读取树莓派已有文件，不会在车上安装或修改软件：

```bash
./scripts/sync_pi_opencv_sysroot.sh 5G@192.168.124.5
./scripts/build_pi_vision_cross.sh
```

输出文件：

```text
build/pi-aarch64-motion/xtnetrc_motion_demo
build/pi-aarch64-vision/xt_netrc_vision
```

视觉文件已在树莓派独立 staging 目录完成 `--help` 和动态库加载验证。上传新测试文件：

```bash
./scripts/deploy_pi_vision_staging.sh 5G@192.168.124.5
```

默认构建不会自动上传，两份程序都不会替换树莓派现有视觉程序。

## 安全要求

1. 第一次测试电机时让驱动轮离地，并准备物理断电。
2. 未确认 PWM 中位值、前进值、后退值和 GPIO 编号前，不启用真实电机输出。
3. 不要让 `network-rc.service` 与自动驾驶程序同时控制同一组 PWM。
4. 旧资料里的 FRP 服务器、token 和网址属于 2022–2023 年配置，不应直接用于新工程。
5. 断线、摄像头读取失败或识别置信度过低时，后续电机模块必须自动回到安全停止值。

## 与原厂案例相比已修正的明显问题

- 原 `循迹.cpp` 右边界循环使用 `j <= cols`，可能越界；本工程使用严格的 `< cols`。
- 原代码在右边线丢失时把列坐标设为图像行数；本工程不混用 rows 和 cols。
- 原代码固定扫描第 250–350 行，只适用于特定分辨率；本工程按图像比例选 ROI。
- 原案例文件 `luzhi.c` 实际包含 C++，正确扩展名应为 `.cpp`。
- 原 CMake 文件版本过旧且包含可疑的 `${PROJECT_SOURCE_DIR}pio` 拼接；本工程改用现代 CMake 和明确的 OpenCV 组件。
