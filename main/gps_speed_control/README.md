# XT-NetRC GPS/IMU 速度估计与速度环

本模块读取车载 WIT 组合导航模块，将 GPS 地速统一换算为 m/s，并用三轴加速度在
两次 GPS 更新之间做短时预测。速度 PI 只输出应叠加到已标定前馈 PWM 上的修正量，
本目录不包含 GPIO、pigpio、电机或舵机输出。

## 当前实车结论（2026-09-04，室外静止测试）

- 设备：`/dev/ttyUSB0`，9600 baud，WIT 11 字节协议。
- 3 分钟收到 14,496 个有效帧，校验错误为 0。
- 可见卫星 7～9 颗，HDOP 约 1.5～2.9，定位有效。
- 串口帧约 10 Hz；实测 GPS 地速/位置内容约 5 Hz 变化，但车辆资料标称 GPS
  定位更新率为 1 Hz，因此速度 PI 建议先以 1 Hz 更新。
- 静止 GPSV 实测约 0.203～2.315 km/h，即约 0.056～0.643 m/s。不能把单帧 GPS
  地速直接反馈给快速 PID。
- 三轴加速度、角速度和角度均约 10 Hz。静止 30 秒时前两轴加速度标准差约
  0.002～0.003 m/s²，适合做短时预测，但长时间积分一定会漂移，必须由 GPS 校正。

## 安全结构

推荐控制链：

```text
路径规划目标速度(m/s)
        │
        ▼
已标定 PWM 前馈 ───────────────┐
                               ├─ 限幅后发给电调
GPS 中值/低通 + IMU 短时预测 ─ PI 修正量
        │
        └─ 卫星数、HDOP、2.5 s 超时门控；失效时 PI 修正归零
```

GPS 速度环只是低频微调，不替代电调中位、前进方向、PWM 上下限和急停逻辑。
在完成 PWM—车速标定前，不得把示例 PI 参数直接接到电机。

## 构建与只读运行

Mac 上构建和测试：

```bash
cmake -S main/gps_speed_control -B build/gps-speed -DCMAKE_BUILD_TYPE=Release
cmake --build build/gps-speed --parallel
ctest --test-dir build/gps-speed --output-on-failure
```

交叉编译并部署到树莓派独立目录：

```bash
./scripts/build_pi_gps_speed_cross.sh
./scripts/deploy_pi_gps_speed_staging.sh 5G@192.168.107.252
```

只读监视 60 秒（不会启动电机）：

```bash
ssh 5G@192.168.107.252 \
  '/home/5G/xtnetrc_staging/gps_speed_control/xtnetrc_gps_speed_monitor \
   --port /dev/ttyUSB0 --duration 60 --forward-axis x'
```

`--forward-axis` 支持 `x`、`-x`、`y`、`-y`。目前还需通过人工前后推车确定传感器
哪一轴对应车辆前方；确认前不得依赖加速度预测控制电机。

## 后续地面标定顺序

1. 电机保持关闭，人工沿车头方向快速推车再刹停，确定前向轴和符号。
2. 空旷直道上先禁用 PI，只记录固定 PWM 对应的稳态 GPS 速度，建立前馈表。
3. 目标速度不得落入 GPS 静态噪声范围；按现有运动配置先在约 0.6 m/s 档标定，
   `0.18 m/s` 等低速档不能期待 GPS 单独形成平稳闭环。
4. 在前馈已可稳定运行后，以 1 Hz 开启小增益 PI，只允许有限 PWM 修正。
5. 验证卫星不足、HDOP 超限、串口断开和 GPS 超时均不会继续增加 PWM。
