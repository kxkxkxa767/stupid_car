# XT-NetRC 相机与地面坐标标定

本目录保存原始标定照片和最终参数。标定分辨率固定为 `640x480`；完成标定后，
相机安装角度、焦距、分辨率或画面裁剪方式发生变化都必须重新标定。

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
  --camera 0 --width 640 --height 480 \
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

1. 近端左：`x=0.70 m, y=+0.25 m`
2. 近端右：`x=0.70 m, y=-0.25 m`
3. 远端左：`x=1.20 m, y=+0.25 m`
4. 远端右：`x=1.20 m, y=-0.25 m`

拍一张固定地面参考图：

```bash
/home/5G/xtnetrc_staging/vision_cross/xt_netrc_vision \
  --camera 0 --width 640 --height 480 \
  --snapshot /home/5G/xtnetrc_staging/calibration/ground_reference.jpg
```

下载后运行点选工具，严格按上述 1→4 顺序点击，按 `U` 撤销，按 Enter 保存：

```bash
.venv/bin/python scripts/calibrate_ground_plane.py \
  --image main/camera_calibration/captures/ground_reference.jpg \
  --intrinsics main/camera_calibration/output/camera_intrinsics.json \
  --output main/camera_calibration/output/ground_projection.json \
  --near-m 0.70 --far-m 1.20 --half-width-m 0.25
```

`main/ground_projection` 的 C++ 库会严格检查画面分辨率，先去畸变，再将视觉模块
输出的中心线像素点转换为车辆坐标系下的米制点。当前仍不包含 GPIO/PWM 输出。

## 当前实车标定结果（2026-08-31）

- 内参有效图片：15 张，RMS 重投影误差 `0.2345 px`，最差单张 `0.3666 px`。
- 地面四点：`x=0.70/1.20 m`、`y=±0.25 m`。
- 四点最大回代误差：小于 `0.000001 m`（四点拟合的数值一致性）。
- 参数：`output/camera_intrinsics.json` 和 `output/ground_projection.json`。
- 预览：`output/ground_birdseye_preview.jpg`。
- 固定摄像头加固后重新确认了外参；车辆和四个沙包均未移动。新参考图为
  `captures/ground_reference_after_camera_fix_640x480.jpg`，新像素接地点依次为
  `(63.881,311.539)`、`(589.071,340.210)`、`(210.111,216.762)`、
  `(487.686,227.782)`。
- 旧外参保存在
  `output/ground_projection_before_camera_fix_2026-08-31.json`，不能再用于当前
  摄像头姿态；相机内参未改变。

内参照片主要覆盖画面中央，近端沙包位于广角边缘，因此当前地面模型明确采用
`raw_pixel_homography`，不向边缘外推不稳定的畸变逆解。相机位置、俯仰角或
640×480 分辨率改变后，本标定立即失效，必须重新执行。
