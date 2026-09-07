# 统一运行框架与主控移植说明

更新：2026-09-06。此版本完成结构整合与离线验证，不等于已完成实车调参或比赛验收。
本轮没有部署、连接 GPIO 或启动电机；原车 `/home/5G/5G` 不得覆盖。

## 1. 模块边界

```text
camera_io (FrameSource / Linux V4L2 / 离线图片)
  → 异步 VisionWorker → 单槽最新观测（采集时间戳、米制中心与航向、置信度）
  → car_runtime / ControlPipeline → motion_control / Pure Pursuit
hardware / ByteSource → WIT 解析 → GpsInput → GPS 中值/低通 → speed PI
  → 车辆标定映射 → GuardedVehicle → VehicleDriver → Raspberry Pi pigpio
```

- `motion_control`、GPS 解析/估计/PI、`runtime_core` 不依赖 OpenCV。
- `hardware` 提供车辆输出、命令看门狗和串口接口，不再放在视觉测试目录。
- `camera_io` 封装平台采集；Linux 使用非阻塞 V4L2、有限时长 poll 和内核采集时间戳。
- `car_runtime` 是实车自动行驶唯一入口。控制线程 20 Hz；摄像头/识别在独立线程，
  默认速度 PI 每 1 秒更新。重复图像不会被计为新的置信度恢复帧。
- GPS 质量帧和速度帧都必须新鲜；IMU 的前向轴/偏置尚未标定，所以新入口没有启用
  加速度积分预测。旧只读监视程序可继续用于验证前向轴。
- GPS 时间采用串口接收时刻，不是传感器内部测量时间；收到满 256 字节积压批次
  会报错停车，不将积压串口数据重新标为“刚测得”。
- Python 保留作标定和研究，不要求移植到 C++。GUI、硬件后端均可按构建选项选择。

更换 Linux 主控时，实现 `FrameSource`、`ByteSource`、`VehicleDriver` 对应后端，
更新板卡配置及目标工具链/sysroot，保留控制算法。`PwmCommand` 是当前驱动边界的
电气命令；若将来改用 CAN 速度控制器，应增加对应的物理量适配器，不应将 CAN 协议
塞进视觉算法。单片机只能复用经资源适配的算法，不支持原样搬运 Linux/OpenCV。

## 2. camera 0 到底是什么

`camera(0)` 通常对应 Linux `/dev/video0`，这是枚举号，不是永久的“固定摄像头身份”。
2026-09-06 室内现场纠正：**XWF 是固定循迹镜头，H65 是云台模型识别镜头**。
此前 H65 r2 的固定镜头身份记录有误，已备份，不再作为当前循迹外参。
2026-09-07 新角度标定时 XWF → `/dev/video0`、H65 → `/dev/video2`；昨日室内曾相反。

当前标定绑定：

```text
/dev/v4l/by-id/usb-XWF_1080P_PC_Camera_XWF_1080P_PC_Camera_240122004-video-index0
```

运行时解析此链接并核对实际打开的设备。若 XWF 重新枚举到 video0，仍然正常运行，
不需要修改外参。不会自动使用云台镜头，也不会依次尝试 0/1/2 猜测摄像头。
现场可只读检查 `ls -l /dev/v4l/by-id/`，结合遮挡固定镜头确认物理身份。
若设备没有唯一序列号导致 by-id 不唯一，应配置 by-path 并重新确认安装位置。

## 3. 默认与显式降级模式

默认不打开 pigpio、不输出任何舵机/电机信号，只观察视觉/GPS并打印拟议命令。

| 情况 | 行为 |
|---|---|
| 编号改变但 by-id 指向同一设备 | 正常运行，不算身份不匹配 |
| 设备身份不一致/元数据缺失 | 默认报错；显式 `--allow-camera-mismatch` 后可降级运行并持续标记警告 |
| 执行器/地面标定未独立验收 | 默认禁止实车输出；显式 `--allow-unverified-calibration` 可作受监督的短时调试 |
| 任一降级条件 | 目标速度最多 0.10 m/s，仍受 PWM、数据时效、距离和时长限制 |
| 图像大小不同、解码错误、采集时间戳无效 | 拒绝套用旧外参；不自动缩放或裁剪 |
| 丢线、GPS 失效、控制周期过长、急停 | 回中性，运动后的故障锁存；不会自动恢复给油 |

“限速”是软件目标，不是已验证的物理速度上限。当前前馈和 GPS 噪声尚未标定，
可能不动，也可能实际速度不符合目标。不能把降级开关当作完成比赛的保证。
不同镜头继续套用旧外参时，米制距离和转角不可信；显式降级只适用于现场监管的
短时调试。正式比赛需提前验证备用配置/相机，并补全人工接管流程；当前没有实现
遥控器接管协议，也不会自动停止原厂服务来抢占硬件。

## 4. 配置与安全边界

统一配置：`main/config/runtime_2023.json`。相对文件路径以配置文件目录为基准，
未知字段、缺项、重复字段、非法数值会报错。

- `board`：引脚、频率、PWM 范围、中点、串口路径；安全包络在 pigpio 后端再次校验。
- `control`：目标速度、最大行驶时间、GPS 积分距离门限、帧/GPS 时效、PI 周期及增益。
- `files`：运动参数与当前外参。没有修改本次四点坐标、单应矩阵或历史内参。
- `steering_pwm_per_rad=122.45` 是根据旧的约 0.4679°/PWM 估计的临时斜率，
  **不是使用新外参完成的新测量**；中点仍是整数 72，输出仅限 70～73。
- `feedforward_pwm=11100` 是历史可起步测试值，**不是已证实的 0.20 m/s 映射**。
  PI ±8 PWM 也只是原模块初值。两个 verified 标志保持 0，必须实测后才能更新。
- 地面外参只有 XWF 的 1.20～1.70 m 四点拟合；控制路径近端有外推，需实测验证。
- GPS 积分门限是估计距离，不是精确的最终停车距离，未补偿滑行。静止噪声仍可能
  使预检失败或提前停车，不能承诺精确 1 m / 3 m。

`GuardedVehicle` 默认命令租约为 300 ms，独立线程约每 10 ms 检查；首次给油后还有
不可被刷新延长的总时限。提交时也检查是否过期，避免调度延迟后重新给油。
停止、越界和写驱动失败均回中性。新入口当前默认 5 秒，配置绝对上限 10 秒。

这是**进程级**保护，受操作系统调度和底层驱动行为影响；不承诺硬实时，也不能覆盖
SIGKILL、整个进程被暂停、内核死锁或断电。仍需物理急停/外部硬件看门狗。
新车辆驱动用 `/run/lock/xtnetrc-actuators.lock` 防止合作进程争抢；这不能阻止不使用
该锁的原厂程序，运行前必须人工确认没有其他 PWM 控制者。

## 5. 构建与只读检查

Mac 全量：

```bash
cmake --preset mac-release
cmake --build --preset mac-release --parallel 6
ctest --test-dir build/mac-release --output-on-failure
```

不需要 OpenCV 的控制核心：

```bash
cmake --preset core-only
cmake --build --preset core-only --parallel 6
ctest --test-dir build/core-only --output-on-failure
```

新主控可通过 `-DCMAKE_TOOLCHAIN_FILE=...` 指定对应工具链。树莓派只在 Mac 交叉编译：

```bash
bash scripts/build_pi_runtime_cross.sh
```

产物在 `build/pi-aarch64-runtime/`，不提交 Git。脚本不连接或修改树莓派；部署只能
放入 `/home/5G/xtnetrc_staging/`，保留配置间相对目录关系，不覆盖原车工程。

在工程根目录离线检查当前标定图片（只验证读取/识别链路，不保证沙包图含双实线）：

```bash
./build/mac-release/xtnetrc_car_runtime \
  --config main/config/runtime_2023.json \
  --replay-image main/camera_calibration/captures/ground_reference_fixed_xwf_120_170_2026-09-07_640x480.jpg
```

预检未通过返回 3，解析/设备错误返回 2，正常观察/正常行驶限时结束返回 0。
离线回放使用明确标记的合成零速度，并绝对禁止电机使能。

在 staging 中运行 Linux 二进制只读预检：

```bash
./xtnetrc_car_runtime --config main/config/runtime_2023.json
```

只读查看指定摄像头的降级链路：

```bash
./xtnetrc_car_runtime --config main/config/runtime_2023.json \
  --camera-device /dev/video0 --allow-camera-mismatch
```

不要因为示例写了 video0 就认定它是固定镜头；应先检查 by-id/遮挡画面。
电机输出仍需同时指定 `--enable-motor` 与 `--i-understand-vehicle-will-move`。
未验收的配置还需上述显式调试开关，且须先只读、再架空、最后清空场地短测。

## 6. 旧入口迁移与验证范围

- `xtnetrc_lane_follow_test` 兼容入口已移除，只使用 `xtnetrc_car_runtime`。
- `xtnetrc_visual_distance_test` 现在只读，移除了固定 PWM 驱动；继续输出 VO 估计。
- `xtnetrc_gps_distance_test` 旧迁移提示入口已移除；历史实现可从 Git 恢复。
- 旧的 `--steering-check-only`、`--motor-pwm`、`--distance-scale`、`--debug-video`
  不再作为自动行驶入口参数；旧完整实验留在 Git 历史，不应误当新版命令执行。
- `--save-overlay` 在停车后保存最终叠加图，不把视频编码写盘放入控制线程。
- 根构建已纳入 GPS 和摄像头运动验证模块，子项目不再全局关闭测试。
- 新测试包括设备枚举互换、显式降级、分辨率拒绝、真实 OpenCV 合成跑道链路、
  GPS PI 对输出的影响、质量帧过期、重复帧、旧帧/未来时间、失线、超速、超时、
  停车锁存、驱动异常和独立看门狗。全部使用模拟输出，不碰实车。
- 已补充上传所需的 `workflow` 权限，GitHub CI 配置纳入本次提交。
  每次 push / pull request 在 Ubuntu 24.04 上构建有视觉/无视觉两套并执行 CTest；
  关闭 GUI 与 pigpio 后端，执行器测试使用模拟输出，不连接实车。

Linux 采集适配依据：[V4L2 官方采集示例](https://docs.kernel.org/userspace-api/media/v4l/capture.c.html)、
[缓冲区与时间戳语义](https://docs.kernel.org/userspace-api/media/v4l/buffer.html)。

## 7. 验证记录

### 2026-09-07：新外参及入口清理

- Mac 视觉版构建成功，CTest 12/12；无 OpenCV 核心构建成功，CTest 5/5；Python 3/3。
- Linux ARM64 根工程交叉编译成功，生成 `xtnetrc_car_runtime`；编译产物不上传 Git。
- 移除旧 GPS 提示壳和重复循迹兼容入口后重新配置、构建并测试，未删除依赖库。
- XWF 新角度地面外参已更新，历史内参原件不变；新参数未部署，未启动任何实车执行器。

### 2026-09-06：此前框架整合

- Mac Release：12/12 CTest 通过；原 Python 视觉 3/3 通过。
- 不含 OpenCV 的 `core-only`：5/5 通过。
- 视觉子目录独立构建：10/10 通过。
- ARM64：完整根工程交叉编译成功，生成 Linux aarch64 `xtnetrc_car_runtime`。
- H65 r2 沙包参考图只读回放：没有可信双边界，未通过行驶预检，电机输出从未启用。
  这张图用于标定，不是比赛跑道检测通过的证据。
- `git diff --check` 通过；当前内参和地面外参 JSON 的数值文件未修改。
- 尚未进行新入口实机采集/架空/地面测试。以上是本地验证结果；云端构建与测试
  以 [GitHub Actions](https://github.com/kxkxkxa767/stupid_car/actions/workflows/ci.yml)
  对应提交的运行结果为准，不代表实车验收。
