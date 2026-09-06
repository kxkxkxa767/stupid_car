# XT-NetRC 相机与地面坐标标定

本目录保存原始标定照片和最终参数。标定分辨率固定为 `640x480`；完成标定后，
相机安装角度、焦距、分辨率或画面裁剪方式发生变化都必须重新标定。

## 当前摄像头身份（2026-09-06）

用户确认本次展示的 **H65 USB CAMERA** 画面用于固定摄像头标定。采集设备为：

```text
/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0 -> /dev/video2
```

历史称呼 `camera 0` 不能当作固定 USB 编号。本次 `8080` 服务实际读取 XWF 1080P 的
`/dev/video0`，不是本次 H65 标定源。以下 `--camera 2` 只适用于本次枚举结果；重启或
拔插后先用 `readlink -f` 检查上述 by-id 路径，再使用对应编号。配置中的身份信息是记录，
当前 C++ 加载器不会自动核实连接的相机身份。

## 1. 打印 ChArUco 标定板

打开 `output/pdf/XT-NetRC_Charuco相机标定板_A4.pdf`，选择“实际大小”或 `100%`，
禁止“适合页面”。打印后用尺测量页面底部校验线，必须是 `100.0 mm`。
棋盘参数为 5×7 格、格长 35 mm、标记长 25 mm、`DICT_5X5_100`。

## 2. 在树莓派采集 20 张内参照片

把标定板贴在平整硬板上。运行下面命令后，在镜头前缓慢改变标定板的距离、
左右位置、俯仰和旋转角度；每张图都应看清棋盘，且不要只在画面中央。

```bash
mkdir -p /home/5G/xtnetrc_staging/calibration/intrinsics
/home/5G/xtnetrc_staging/vision_cross/xt_netrc_vision \
  --camera 2 --width 640 --height 480 \
  --capture-dir /home/5G/xtnetrc_staging/calibration/intrinsics \
  --capture-count 20 --capture-interval-ms 1500
```

下载到 Mac：

```bash
cd /Users/yuhaojin/Documents/5G_opencv
rsync -av 5G@192.168.124.5:/home/5G/xtnetrc_staging/calibration/intrinsics/ \
  main/camera_calibration/captures/intrinsics/
```

计算内参：

```bash
.venv/bin/python scripts/calibrate_camera_intrinsics.py \
  --images main/camera_calibration/captures/intrinsics \
  --output main/camera_calibration/output/camera_intrinsics.json
```

建议至少 12 张有效图，RMS 重投影误差最好小于 0.5 px；超过 1.0 px 应重新采集。

## 3. 标定地面到车辆坐标

车辆坐标原点取后轴中心，`x` 向前、`y` 向左。保持车辆和摄像头不动，在地面
标出以下四个点，并实际用卷尺核对。本次实车使用方形沙包，坐标点取每个沙包
“朝向车辆的面”和“朝向矩形内部的面”相交的接地角：

1. 近端左：`x=1.50 m, y=+0.25 m`
2. 近端右：`x=1.50 m, y=-0.25 m`
3. 远端左：`x=2.00 m, y=+0.25 m`
4. 远端右：`x=2.00 m, y=-0.25 m`

拍一张固定地面参考图：

```bash
/home/5G/xtnetrc_staging/vision_cross/xt_netrc_vision \
  --camera 2 --width 640 --height 480 \
  --snapshot /home/5G/xtnetrc_staging/calibration/ground_reference.jpg
```

下载后运行点选工具，严格按上述 1→4 顺序点击，按 `U` 撤销，按 Enter 保存：

```bash
.venv/bin/python scripts/calibrate_ground_plane.py \
  --image main/camera_calibration/captures/ground_reference.jpg \
  --intrinsics main/camera_calibration/output/camera_intrinsics.json \
  --output main/camera_calibration/output/ground_projection.json \
  --near-m 1.50 --far-m 2.00 --half-width-m 0.25 \
  --raw-pixel-homography
```

`main/ground_projection` 的 C++ 库会严格检查画面分辨率，并按配置的投影模型将视觉
模块输出的中心线像素点转换为车辆坐标系下的米制点。当前使用原始像素单应性，不再
额外去畸变；模块仍不包含 GPIO/PWM 输出。

## 当前实车标定结果（2026-09-06）

- 以用户要求重新拍摄的第二帧为准，使用 H65 原生 `640x480` MJPG 解码帧，PNG 保存，
  未裁剪或缩放：`captures/ground_reference_fixed_h65_150_200_2026-09-06_r2_640x480.png`。
- 四个内角由新图人工选取，地面坐标由用户提供，严格以**后轴中心**为原点：

| 点序 | 接地内角 | 原图像素 (u, v) | 车辆坐标 (x, y)，m |
|---|---|---|---|
| 1 | 近端左 | (260, 329) | (1.50, +0.25) |
| 2 | 近端右 | (386, 328) | (1.50, -0.25) |
| 3 | 远端左 | (277, 307) | (2.00, +0.25) |
| 4 | 远端右 | (369, 305) | (2.00, -0.25) |

- 正式投影参数：`output/ground_projection.json`，模型为 `raw_pixel_homography`。
- 点位预览：`output/ground_reference_points_fixed_h65_2026-09-06_preview.png`；鸟瞰：
  `output/ground_birdseye_preview.jpg`。
- 采集来源、相机身份、原图 SHA-256 和验证范围保存在
  `captures/ground_reference_fixed_h65_150_200_2026-09-06_r2_metadata.json`，同时嵌入正式
  配置的 `calibration_metadata` 字段。
- 四点回代小于 `0.000001 m` 只是拟合一致性；没有第五个独立实测点，尚未验证实际米制
  精度。以原图纵向偏差 ±2 px 作示例，在四角附近可造成约 3～7 cm 的投影距离变化；
  这不是实测误差界限或统计置信区间。
- 参考点覆盖 `x=1.50～2.00 m`、`y=±0.25 m`；范围外为外推。近车区域的光流里程、
  旧距离比例与旧视觉推算转角需要重新验证。鸟瞰仅对地面有效，沙包立面被拉长是正常的。
- `output/camera_intrinsics.json` 保持历史原件：15 张、RMS `0.2345 px`、最差单张
  `0.3666 px`。历史文件未记录设备 ID，尚不能独立确认它属于本次 H65；这些数值不是
  本次 H65 内参质量结论。本次直接拟合原始像素、`projection_dist_coeffs` 全为 0，
  不使用历史畸变参数；正式配置保留历史内参字段仅为兼容现有加载器。需要去畸变或
  三维姿态解算时，应先为该设备核实或重新采集内参。

在项目根目录重现本次结果：

```bash
.venv/bin/python scripts/calibrate_ground_plane.py \
  --image main/camera_calibration/captures/ground_reference_fixed_h65_150_200_2026-09-06_r2_640x480.png \
  --intrinsics main/camera_calibration/output/camera_intrinsics.json \
  --output main/camera_calibration/output/ground_projection.json \
  --near-m 1.50 --far-m 2.00 --half-width-m 0.25 \
  --image-points '260,329;386,328;277,307;369,305' \
  --raw-pixel-homography \
  --metadata main/camera_calibration/captures/ground_reference_fixed_h65_150_200_2026-09-06_r2_metadata.json \
  --points-preview main/camera_calibration/output/ground_reference_points_fixed_h65_2026-09-06_preview.png

.venv/bin/python scripts/render_ground_projection_preview.py \
  --image main/camera_calibration/captures/ground_reference_fixed_h65_150_200_2026-09-06_r2_640x480.png \
  --config main/camera_calibration/output/ground_projection.json \
  --output main/camera_calibration/output/ground_birdseye_preview.jpg \
  --x-min 1.20 --x-max 2.20 --half-width 0.40 --pixels-per-meter 600
```

## 历史文件（不用于当前姿态和摆放）

- `output/ground_projection_before_camera_fix_2026-08-31.json`：摄像头加固前。
- `output/ground_projection_before_distance_correction_2026-09-06.json`：历史加固后
  `0.70/1.20 m` 配置；本次移动沙包和重新采集后已不适用。
- `output/ground_projection_before_live_h65_2026-09-06.json`：本轮早期仅沿用旧图像素改为
  `0.75/1.00 m` 的中间结果，未核实当前实车，已废弃。
- `output/ground_projection_superseded_h65_first_capture_2026-09-06.json`：H65 第一帧
  `1.50/2.00 m` 候选；用户要求重做后由当前 r2 新图和新点位替代。

更换摄像头、镜头/对焦、安装姿态、分辨率或裁剪方式后，当前投影必须重新标定。
