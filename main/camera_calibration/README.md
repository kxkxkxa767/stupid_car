# XT-NetRC 相机与地面坐标标定

更新：2026-09-07，固定镜头重新调整角度并加固后。当前完成的是 **XWF 固定镜头的四点地面外参拟合**，
不是新内参标定，也不是实车循迹或测距验收。

## 1. 两个摄像头的身份和用途

| 物理位置 | 设备 | 用途 | 本次室内枚举 |
|---|---|---|---|
| 车头固定镜头 | XWF 1080P PC Camera | 双实线循迹、固定姿态地面投影 | /dev/video0 |
| 双轴云台镜头 | H65 USB CAMERA | 接入模型识别障碍物等目标 | /dev/video2 |

用户查看当前 8080 实时画面后明确确认 XWF 是循迹镜头，SSH 核对预览进程确实
打开了 XWF。没有保存完整的遮挡前后序列，不将此描述为“遮挡验证通过”。
详见 [摄像头身份与用途](摄像头身份与用途.md)。

固定镜头唯一入口：

```text
/dev/v4l/by-id/usb-XWF_1080P_PC_Camera_XWF_1080P_PC_Camera_240122004-video-index0
```

云台镜头：

```text
/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0
```

同日室外与室内的 video0/video2 曾互换。**camera 0、camera 2 和端口 8080 都不是
固定的物理身份**；8080 显示哪个镜头取决于启动命令。统一运行入口会核对外参
元数据中的角色、by-id 和实际分辨率；不匹配必须显式选择降级，不能静默换镜头。

## 2. 当前四点外参

车辆原点：后轴中心；x 向前，y 向左；单位 m。用户最终确认近排 1.20 m、
远排 1.70 m，左右接地内角距车身中线各 0.25 m。
每点取沙包“朝向车辆的面”与“朝向矩形内部的面”相交的接地角，
不是顶部角、外轮廓最内侧点或地面倒影。

| 顺序 | 接地内角 | 原始像素 (u, v) | 车辆坐标 (x, y)，m |
|---|---|---|---|
| 1 | 近端左 | (166, 263) | (1.20, +0.25) |
| 2 | 近端右 | (465, 265) | (1.20, -0.25) |
| 3 | 远端左 | (217, 222) | (1.70, +0.25) |
| 4 | 远端右 | (427, 225) | (1.70, -0.25) |

- 当前参数：[ground_projection.json](output/ground_projection.json)，绑定固定 XWF。
- 原图：[640×480 原始 JPEG](captures/ground_reference_fixed_xwf_120_170_2026-09-07_640x480.jpg)。
  从已核实来源的 MJPEG 服务取得，无缩放、裁剪、去畸变或再次压缩。
- [来源与身份元数据](captures/ground_reference_fixed_xwf_120_170_2026-09-07_metadata.json)
  同时嵌入外参 JSON，记录原图 SHA-256、选点方法和适用范围。
- [点位预览](output/ground_reference_points_fixed_xwf_2026-09-07_preview.png)。
- [鸟瞰预览](output/ground_birdseye_fixed_xwf_2026-09-07_preview.jpg)。
- 旧 H65 当前文件已原样备份为
  [H65 身份纠正前备份](output/ground_projection_superseded_h65_role_correction_2026-09-06.json)，
  不得将其中“front_fixed”历史标签当作新的身份结论。

- 昨天 XWF 旧角度外参已备份为
  [XWF 调角前备份](output/ground_projection_superseded_xwf_before_tilt_2026-09-07.json)，
  不再适用于今天的新角度。昨天的原图与预览保留作历史证据。

## 3. 精度与适用限制

本次采用 `raw_pixel_homography`，直接拟合原始像素，投影畸变系数全为 0。
近端袋子有两个可见侧面，选取两侧面交线的接地顶点，不按轮廓极值盲选。
四点最大回代误差约 **0.00000018 m**，仅表示四点拟合一致性。

新角度使四点远离图像底边，近处地面明显增多；软袋边缘和阴影仍影响选点。固定当前单应矩阵，将各点沿像素轴偏移 ±3 px
作敏感性示例，四角的最大地面变化分别约 **2.7、2.8、5.4、5.6 cm**。
这不是实测误差界限，也不是置信区间。仍需用第五个独立地面点实测验证。

参考覆盖 x=1.20～1.70 m、y=±0.25 m；四边形外为外推。鸟瞰只对地面有效，
沙包和背景立面被拉长、图像覆盖不到的地方呈黑色/灰色都不能解释为地面测量。
循迹保留原先 0.55～2.20 m、左右 ±0.75 m 的搜索范围，由当前外参生成原图掩码；
不会把 1.20～1.70 m 标定范围硬当作循迹视野。超出控制点区域仍是外推。
光流采用标定覆盖范围，并排除投影黑边；置信度、宽度和停车门限没有放宽。
外参完成不代表真实跑道检测、GPS 速度环或 1 m 停车已经通过验证。
`runtime_2023.json` 中的地面与执行器验收标志继续保持未验收。

只要固定安装位置、俯仰/偏航、焦距/对焦、640×480 原始分辨率或裁剪方式变化，
本外参就必须复核或重做。H65 云台运动后不能沿用某一姿态的地面外参，更不能
共用 XWF 的矩阵。禁止为了换相机只改 by-id 而继续使用另一个镜头的数值。

## 4. 内参与重现方法

历史 `output/camera_intrinsics.json` 原件未改：15 张，RMS 0.2345 px，
最差单张 0.3666 px。文件未可靠绑定设备，这些数值**不是新测得的 XWF 内参质量**。
仅为现有加载器兼容保留相机矩阵等字段，本次原始像素拟合不使用历史畸变参数。
需要去畸变、三维姿态或模型测距时，应独立核实或重新采集该镜头的内参。

内参标定板：ChArUco 5×7，格长 35 mm、标记长 25 mm、DICT_5X5_100。
打印 `output/pdf/XT-NetRC_Charuco相机标定板_A4.pdf` 时使用实际大小，核对
底部 100 mm 标尺。用固定 XWF 在原生分辨率拍摄多个位置、角度的清晰照片，
再用 `scripts/calibrate_camera_intrinsics.py` 计算；不要覆盖历史原件来冒充新测量。

以下命令从已归档原图重现本次结果，只在 Mac 工程根目录执行：

```bash
.venv/bin/python scripts/calibrate_ground_plane.py \
  --image main/camera_calibration/captures/ground_reference_fixed_xwf_120_170_2026-09-07_640x480.jpg \
  --intrinsics main/camera_calibration/output/camera_intrinsics.json \
  --output main/camera_calibration/output/ground_projection.json \
  --near-m 1.20 --far-m 1.70 --half-width-m 0.25 \
  --image-points '166,263;465,265;217,222;427,225' \
  --raw-pixel-homography \
  --metadata main/camera_calibration/captures/ground_reference_fixed_xwf_120_170_2026-09-07_metadata.json \
  --points-preview main/camera_calibration/output/ground_reference_points_fixed_xwf_2026-09-07_preview.png

.venv/bin/python scripts/render_ground_projection_preview.py \
  --image main/camera_calibration/captures/ground_reference_fixed_xwf_120_170_2026-09-07_640x480.jpg \
  --config main/camera_calibration/output/ground_projection.json \
  --output main/camera_calibration/output/ground_birdseye_fixed_xwf_2026-09-07_preview.jpg \
  --x-min 1.00 --x-max 1.90 --half-width 0.40 --pixels-per-meter 600
```

## 5. 历史文件

`ground_projection_before_*.json`、`ground_projection_superseded_*.json` 以及
H65 1.50/2.00 m 原图、点位与元数据均保留作追溯；旧“fixed_h65”文件名和角色
文字反映的是当时错误判断，不代表现在允许用于循迹。
旧通用 `ground_birdseye_preview.jpg` 也是历史预览，当前只看上面的 XWF 专用文件。

本次只修改本地标定、记录及相关离线身份断言，不覆盖树莓派原工程，不启动电机，
不改舵机中点、云台位置或焦距。新参数尚未部署到实车测试目录。
