# XT-NetRC 工程接手与协作约定

适用于本仓库及其子目录，供后续 AI 助手和协作者使用，不绑定特定模型。
维护日期：2026-09-07。本文件规定工作方式，不代替实时设备检查和实车验收。
用户当前明确要求优先于这里的默认工作约定；系统安全边界仍须遵守。

## 1. 接手先读与任务边界

- 先确认工作目录与 Git 根目录。本机工程为 `/Users/yuhaojin/Documents/5G_opencv`，
  但队友机器路径可能不同；不要把 Codex 的临时任务目录误当工程目录。
- 读取 [README](README.md)、[上传规范](上传规范.md)、
  [当前调试进度](当前调试进度.md) 和 [统一运行说明](main/car_runtime/README.md)。
  相机任务再读 [标定说明](main/camera_calibration/README.md) 及
  [摄像头身份](main/camera_calibration/摄像头身份与用途.md)。
- 先运行 `git status --short --branch`、`git log -5 --oneline`，检查相关源码与配置。
  文档中的历史结论不能当作当前实测；配置、代码和记录冲突时先核实，不靠改文字掩盖。
- 用户说“看看、诊断、审查”时先只读并给出证据；说“修改、实现”才实施对应变更。
  “上传”指推送 GitHub，“部署”指复制到树莓派；两者都不自动授权启动电机或云台。
- 默认中文沟通，说明改了什么、如何验证、还有哪些限制。明确区分实际执行与建议命令。
  本文件不自动授权并行代理、后台监控或创建其他任务。

## 2. Git 与文件保护

- 仓库为 `https://github.com/kxkxkxa767/stupid_car.git`；日常直接在 `main` 小步修改，
  不默认另建功能分支，不重新初始化仓库。先核对当前分支，不能盲目切走队友的工作。
- 每次开始修改前按 [上传规范](上传规范.md) 拉取最新 `origin/main`。
  工作区干净且位于 main 时执行 `git pull --ff-only origin main`；有改动时先确认归属，
  保留全部未提交内容，不能擅自提交、stash 或覆盖不明改动。无法安全同步时报告原因。
- 只有用户明确要求提交/上传时才执行相应操作；上一轮上传不代表以后每轮都可上传。
  推送前重新同步远端，整合他人提交后重新验证；禁止强推共享 main。
- 使用 `git add <明确路径>`，检查暂存差异；禁止 `git add .` 和为构建产物 `git add -f`。
  `.venv/`、`build/`、`local/`、`third_party/`、`backup/`、`tmp/`、工具链、sysroot、
  完整 SD 卡镜像、日志和大型录像不提交。密码、Token、私钥不能写入仓库或输出日志。
- 不使用 `git reset --hard`、`git checkout --` 丢弃改动，不执行范围不明确的递归删除。
  删除前确认文件用途、调用关系和恢复方式，完成后说明删除范围。
- 用户已确认“只整理 main”：保留 `reference/`、原车 Python 副本、完整备份及
  `scripts/`、`OpenCV/python/` 中今年使用的 Python 工具。不要因“今年用 C++”就删除它们。
- GitHub 需要代理时先读 `scutil --proxy`，按实际启用的主机/端口临时配置。
  优先使用单条 Git 命令的 `-c http.proxy=... -c https.proxy=...`；不硬编码旧端口，
  不修改全局代理，若设置了仓库临时代理，结束后清除自己设置的项。

## 3. 当前框架与移植边界

自动行驶统一入口是 `xtnetrc_car_runtime`，源码在 `main/car_runtime/`。

| 路径 | 职责 |
|---|---|
| `main/car_runtime/` | 统一控制流程、视觉工作线程、运行配置与预检 |
| `main/camera_io/` | 图像来源抽象、Linux V4L2 与离线采集 |
| `main/hardware/` | 车辆输出、串口接口、独立命令看门狗与 pigpio 后端 |
| `main/motion_control/` | Pure Pursuit 等运动控制算法 |
| `main/gps_speed_control/` | WIT 解析、GPS 估计/速度 PI、只读监视 |
| `main/ground_projection/` | 固定相机像素到地面坐标投影 |
| `main/visual_distance_test/` | 新框架依赖的双边界检测、光流、只读诊断及测试 |
| `main/camera_gimbal/` | 云台控制模块 |
| `main/vision/`、`main/camera_motion_validation/` | 视觉与运动模型诊断/验证 |
| `main/config/`、`main/camera_calibration/` | 运行参数、标定与来源证据 |

- 目录带 `test` 不代表废弃。旧 `xtnetrc_lane_follow_test` 与
  `xtnetrc_gps_distance_test` 入口已移除；历史命令仅供追溯，不重新恢复固定 PWM 驱动流程。
- 使用 C++17；控制核心、GPS 解析与 PI 不应引入 OpenCV 或树莓派专用头文件。
  平台变化优先适配 `FrameSource`、`ByteSource`、`VehicleDriver` 和工具链，
  不把串口、GPIO、CAN 协议塞进视觉/控制算法。C++ 不意味着可无条件移植到单片机。
- 运行入口读取 `main/config/runtime_2023.json`；相对配置路径以该文件所在目录为基准。
  `vehicle_2023.json` 还承载车辆与标定记录，不能假定修改它就修改了全部运行参数。
- 保留异步视觉、采集时间戳、最新观测、控制超时和执行器看门狗；避免阻塞控制线程。
  新功能优先增加模拟输出与离线回归测试，不用实车代替单元测试。

## 4. 摄像头与标定：不要再次混用

截至 2026-09-07 的确认身份：

- 固定循迹：**XWF 1080P PC Camera**，设备链接为
  `/dev/v4l/by-id/usb-XWF_1080P_PC_Camera_XWF_1080P_PC_Camera_240122004-video-index0`。
- 云台模型障碍物/目标识别：**H65 USB CAMERA**，链接为
  `/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0`。
- `camera 0`、`/dev/video0` 和 HTTP `8080` 都不是物理身份；枚举号会变，
  8080 的来源由预览进程参数决定。先查 by-id、进程实际来源和原图，必要时请用户遮挡确认。
  未采到完整遮挡过程不能记录为“遮挡验证通过”。
- 当前有效外参只有 `main/camera_calibration/output/ground_projection.json`，
  为 2026-09-07 加固后 XWF 的 `raw_pixel_homography`，原生分辨率必须为 **640×480**。
  历史 `before_*`、`superseded_*` 与误标为 fixed 的 H65 参数不能直接用于当前固定镜头。
- 车辆坐标原点为后轴中心，x 向前、y 向左，单位米。当前四点顺序为近左、近右、远左、远右，
  近/远为 1.20/1.70 m，左右为 ±0.25 m；像素点、照片、哈希与选点定义以当前标定文档和元数据为准。
- 姿态、位置、焦距/对焦、分辨率、裁剪变化后须复核或重做；不自动缩放图像凑旧参数，
  不把 XWF 的矩阵套到 H65，也不只改设备标签假装完成了新标定。
- 四点回代误差很小仅证明拟合一致，不代表真实测距精度。四边形外是外推；
  保留循迹搜索区域不意味着区域内全部经过标定验收。需独立地面点和真实跑道验证。
- 历史 `camera_intrinsics.json` 的设备关联尚未可靠确认；当前 raw 拟合不使用其畸变参数。
  不把历史 RMS 当作新 XWF 内参实测，不无记录覆盖原件。
- 重标定保留旧参数、未加工原图、身份/测量元数据、点位预览、鸟瞰预览及生成命令。
  同步相关配置、测试、身份文档和进度记录；数学计算使用仓库标定脚本，不用生成式图片替代证据。

## 5. 实车、GPS 与安全输出

- 不修改树莓派原工程 `/home/5G/5G`。本机开发和交叉编译，明确授权部署后只复制到
  `/home/5G/xtnetrc_staging/<本次测试目录>/`，不在原工程编译或覆盖原有可用视觉程序。
- IP、video 编号、ttyUSB 编号均须现场核对，不从历史聊天推断仍有效。
  GPS 历史 by-id 为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`；
  默认运行配置仍写 `/dev/ttyUSB0`，现场应核实并用受支持的配置/参数指定，勿误读蜂窝模块端口。
- 当前 GPS 低速噪声明显，不能承诺精确 1 m/3 m 停车；积分估计不是实际后轴位移。
  IMU 前向轴和偏置未验收，不把加速度积分当作可用里程计；编码器能力以接线/协议证据为准。
- 默认无电机输出。只有用户对当前测试明确授权，且现场条件已确认，才可进入实车动作。
  先离线/只读预检，再架空短测，最后清场地面短测；确认物理断电手段及旁人安全。
  用户离场且只允许读传感器时，不能自行启动电机、舵机或云台。
- 执行前核对当前二进制 `--help`、配置、相机/GPS来源、PWM 控制者和测试上限。
  不复制历史实验参数盲跑，不为抢占设备停止无关服务；8080 预览可能与运行入口争用相机。
- 不因车辆不动就自动提高 PWM、延长时间或取消保护。先区分电源、预检、数据质量、
  中性/死区与硬件输出原因，新的运动参数需在当前授权范围内。
- 既有显式降级开关仅用于受监督的短测，不默认加上；不得为通过测试修改验收标志，
  不绕过分辨率、新鲜度、失线、GPS 失效、急停、PWM 包络和命令超时保护。
  软件降级目标速度不是实测限速保证，进程看门狗不是硬件急停。
- 动作结束检查停车、中性输出、设备释放与残留进程；不要仅因 SSH 断开或进程退出就声称车已停。

## 6. 验证命令与完成标准

以下命令在仓库根目录执行，按改动范围选择，不为纯文档修改强行跑全部构建。
依赖路径来自当前机器；缺少 `local/opencv`、虚拟环境或交叉工具链时先报告/按现有说明配置，
不为“编译通过”随意更换依赖版本或把整套工具链纳入 Git。

无真实执行器的 Mac 视觉构建：

```bash
cmake -S . -B build/agent-vision -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/local/opencv" \
  -DXTNETRC_BUILD_GUI=OFF -DXTNETRC_BUILD_VISION=ON -DXTNETRC_WITH_PIGPIO=OFF
cmake --build build/agent-vision --parallel 4
ctest --test-dir build/agent-vision --output-on-failure
```

控制核心、Python 与树莓派交叉编译：

```bash
cmake --preset core-only
cmake --build --preset core-only --parallel 4
ctest --test-dir build/core-only --output-on-failure
.venv/bin/python -m unittest discover -s OpenCV/python/tests -v
bash scripts/build_pi_runtime_cross.sh
```

- 每条失败先处理，不能只看最后一条退出码；构建/测试通过不能沿用上一提交的结果。
- C++ 或 CMake 改动执行相关构建与测试；共用控制/配置修改覆盖有视觉和无视觉两种模式。
  面向树莓派的 C++ 改动执行 ARM64 交叉编译并确认架构，编译脚本成功不代表依赖已在目标机验收。
- 标定/视觉改动验证 JSON 加载、点序/单位、设备身份、原图分辨率及相关投影/光流/控制测试。
  不通过放宽断言或安全门限消除失败；确需变更测试夹具时说明依据。
- 文档修改检查路径、命令、当前/历史表述与空白差异。提交前执行 `git diff --check`，
  新文件也需检查；提交后核对工作区和远端。GitHub CI 成功须查看对应提交，不能用本地测试替代。
- 完成后报告修改范围、实际测试结果、未验证事项、是否提交/推送/部署以及是否发生实车动作。
  重要功能、标定和实测更新 [当前调试进度](当前调试进度.md)，注明日期与证据来源。

## 7. 维护本文件

优先记录稳定规则与文档入口，不复制所有调参历史。结构、入口、身份或安全流程变化时同步维护；
仅改变 IP 或一次试跑结果通常写进进度记录，不新增永久约束。
使用标准文件名 `AGENTS.md`，不再维护内容重复的 `agent.md`。
文件发现规则参考 [OpenAI 官方说明](https://learn.chatgpt.com/docs/agent-configuration/agents-md)。
从本仓库或其子目录启动任务；若任务在其他目录，主动完整读取本文件，不假定已自动加载。
