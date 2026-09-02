# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
from enum import Enum
import pygame
import os
import threading


# 硬件控制依赖
try:
    import pigpio as pio
    os.system("sudo killall pigpiod")
    os.system("sudo pigpiod")
    os.system("sudo systemctl stop network-rc.service")
    time.sleep(1)
    pi_io = pio.pi()
    HARDWARE_AVAILABLE = True
    print("[初始化] 硬件控制已启用")
except Exception as e:
    print(f"[警告] 初始化pigpio失败: {e}")
    pi_io = None
    HARDWARE_AVAILABLE = False

# OCR依赖
try:
    import pytesseract
    from PIL import Image
    PYTESSERACT_AVAILABLE = True
except ImportError:
    print("[警告] 未安装 pytesseract 或 PIL，OCR 功能禁用")
    PYTESSERACT_AVAILABLE = False

# 二维码识别依赖
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    print("[警告] pyzbar 未安装，二维码功能禁用")
    PYZBAR_AVAILABLE = False

# 二维码识别依赖
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    print("[警告] pyzbar 未安装，二维码功能禁用")
    PYZBAR_AVAILABLE = False


class ParkingZone(Enum):
    """停车区域"""
    NONE = 0
    A = 1
    B = 2


class RaceState(Enum):
    """比赛状态"""
    WAITING_START = "WAITING_START"
    NORMAL_DRIVING = "NORMAL_DRIVING"
    AVOIDING_CONE = "AVOIDING_CONE"
    ZEBRA_STOPPING = "ZEBRA_STOPPING"
    PARKING = "PARKING"
    QR_SCANNING = "QR_SCANNING"
    FINISHED = "FINISHED"


class AdvancedOptimizedRaceController:

    def __init__(self, school_name="安徽医科大学", team_name="杏林方舟队"):
        self.school_name = school_name
        self.team_name = team_name
        self.debug_mode = True

        self.zebra_stop_duration = 10

        # 硬件控制参数
        self.servo_pin = 12
        self.motor_pin = 13
        self.current_servo_angle = 88
        self.current_motor_speed = 0

        # PID控制参数
        self.kp = 0.25
        self.kd = 0.1
        self.ki = 0.002
        self.last_error = 0
        self.error_sum = 0
        self.angle_outmax = 20
        self.angle_outmin = -20

        # 速度控
        self.base_speed = 00
        self.min_speed = 00
        self.current_speed = 0
        self.speed_reduction_angle = 15

        # 循迹参数
        self.center_x = 350
        self.img_width = 700
        self.img_height = 450
        self.center_x_lock = threading.Lock()
        self.lane_lost_count = 0
        self.lane_lost_threshold = 10

        # 状态机
        self.state = RaceState.WAITING_START
        self.prev_state = RaceState.WAITING_START
        self.state_start_time = time.time()
        self.last_save_time = time.time()
        self.state_timeout = {
            RaceState.ZEBRA_STOPPING: 15,
            RaceState.PARKING: 5,
            RaceState.QR_SCANNING: 35
        }

        # 任务状态标志
        self.start_detected = False
        self.zebra_crossing_stopped = False
        self.parking_sign_detected = ParkingZone.NONE
        self.target_parking_zone = None
        self.parking_completed = False
        self.qr_code_scanned = False
        self.after_zebra_crossing = False

        # OCR尝试次数
        self.ocr_attempts = {}
        self.max_ocr_attempts = 3

        # 颜色检测阈值（HSV）——只保留红色
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([35, 255, 255])
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])

        # 形态学kernel（预创建）
        self.kernel_5x5 = np.ones((5, 5), dtype=np.uint8)
        self.kernel_rect_25x5 = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 5))
        self.kernel_rect_30x1 = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 1))

        # OCR控制
        self.last_ocr_time = 0
        self.ocr_min_interval = 0.6
        self.parking_letter_cached = None
        self.floor_letter_cached = None

        # 避障参数
        self.cone_detected_time = 0
        self.cone_avoid_cooldown = 3.0
        self.in_avoiding = False
        self.avoid_start_time = 0
        self.max_avoid_duration = 2.0

        # 计时器
        self.task_start_time = 0

        # 帧处理策略
        self.frame_counter = 0
        self.process_every_n_frames = 1  # 基础处理频率

        # 状态特定的检测频率（帧间隔）
        self.detection_intervals = {
            'barrier': 5,  # 红色挡板检测间隔
            'zebra': 2,  # 斑马线检测间隔
            'cone': 1,  # 锥桶检测间隔（安全优先）
            'sign': 4,  # 停车标志检测间隔
            'letter': 4,  # 字母识别间隔
            'area': 3,  # 区域检测间隔（已无用，但保留结构）
        }

        # HSV缓存
        self.hsv_cache = None
        self.hsv_frame_id = -1

        # 调试输出频率
        self.debug_update_interval = 5  # 每10帧更新一次调试显示

        self._init_hardware()

        print(f"[初始化] 就绪")

    def _init_hardware(self):
        """初始化舵机和电机"""
        if not HARDWARE_AVAILABLE or pi_io is None:
            print("[控制] 硬件控制已禁用（模拟模式）")
            return

        try:
            pi_io.set_PWM_frequency(self.servo_pin, 50)
            pi_io.set_PWM_range(self.servo_pin, 100)
            self.set_servo_angle(88)

            pi_io.set_mode(self.motor_pin, pio.OUTPUT)
            pi_io.set_PWM_frequency(self.motor_pin, 200)
            pi_io.set_PWM_range(self.motor_pin, 40000)
            self.set_motor_speed(0)

            print("[硬件] 舵机和电机初始化成功")
        except Exception as e:
            print(f"[错误] 硬件初始化失败: {e}")

    # 硬件控制

    def angle_to_duty_cycle(self, angle):
        """角度转占空比"""
        return 2.49 + (angle / 178.0) * 10

    def set_servo_angle(self, angle):
        """设置舵机角度（优化：减少无效更新）"""
        angle = max(0, min(180, angle))
        if abs(angle - self.current_servo_angle) < 3:
            return

        if HARDWARE_AVAILABLE and pi_io is not None:
            duty = self.angle_to_duty_cycle(angle)
            pi_io.set_PWM_dutycycle(self.servo_pin, duty)

        self.current_servo_angle = angle

    def set_motor_speed(self, speed):
        """设置电机速度（自适应速度）"""
        angle_deviation = abs(self.current_servo_angle - 90)
        if angle_deviation > self.speed_reduction_angle:
            reduction_factor = 1.0 - (angle_deviation - self.speed_reduction_angle) / 30.0
            reduction_factor = max(0.7, min(1.0, reduction_factor))
            speed = int(speed * reduction_factor)
            speed = max(self.min_speed, speed)

        if abs(speed - self.current_motor_speed) < 100:
            return

        speed = max(-3000, min(20000, speed))

        if HARDWARE_AVAILABLE and pi_io is not None:
            output = 10000 + int(speed)
            output = max(6000, min(29000, output))
            pi_io.set_PWM_dutycycle(self.motor_pin, output)

        self.current_motor_speed = speed
        self.current_speed = speed

    def stop_vehicle(self):
        """停车"""
        self.set_motor_speed(0)
        self.set_servo_angle(88)

    def emergency_stop(self):
        """紧急停止"""
        print("[紧急] 紧急停止！")
        self.stop_vehicle()
        if HARDWARE_AVAILABLE and pi_io is not None:
            pi_io.set_PWM_dutycycle(self.motor_pin, 0)

    # 资源优化工具函数
    def get_hsv_cached(self, frame):
        """HSV缓存机制（避免重复转换）"""
        if self.hsv_frame_id != self.frame_counter:
            self.hsv_cache = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            self.hsv_frame_id = self.frame_counter
        return self.hsv_cache

    def should_check(self, task_name):
        """判断是否应该执行某个检测任务"""
        interval = self.detection_intervals.get(task_name, 1)
        return self.frame_counter % interval == 0

    # 循线功能
    def calculate_slope(self, line):
        """计算直线斜率"""
        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            return float('inf')
        return (y2 - y1) / (x2 - x1)

    def get_lane_center(self, left_lines, right_lines):
        """计算车道中心"""
        y_bottom = 250

        def get_x_at_y(line_1, y):
            x1, y1, x2, y2 = line_1[0]
            if y2 == y1:
                return (x1 + x2) / 2
            slope = (x2 - x1) / (y2 - y1)
            return x1 + slope * (y - y1)

        left_x_list = []
        right_x_list = []

        if left_lines:
            for line in left_lines[:3]:
                try:
                    x = get_x_at_y(line, y_bottom)
                    if 0 <= x <= self.img_width:
                        left_x_list.append(x)
                except:
                    continue

        if right_lines:
            for line in right_lines[:3]:
                try:
                    x = get_x_at_y(line, y_bottom)
                    if 0 <= x <= self.img_width:
                        right_x_list.append(x)
                except:
                    continue

        if left_x_list and right_x_list:
            left_avg = sum(left_x_list) / len(left_x_list)
            right_avg = sum(right_x_list) / len(right_x_list)
            new_center = (left_avg + right_avg) / 2
            self.lane_lost_count = 0
        elif left_x_list:
            new_center = sum(left_x_list) / len(left_x_list) + 200
            self.lane_lost_count = 0
        elif right_x_list:
            new_center = sum(right_x_list) / len(right_x_list) - 200
            self.lane_lost_count = 0
        else:
            self.lane_lost_count += 1
            new_center = self.center_x

        new_center = max(0, min(self.img_width, new_center))
        return new_center

    def detect_lane_lines(self, frame):
        """增强版车道线检测，更稳定，适应强光环境"""

        h, w = frame.shape[:2]

        # 1) 使用CLAHE提升亮度对比
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel = lab[:, :, 0]
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        l_clahe = clahe.apply(l_channel)

        # 2) 二值化 + 高斯去噪
        _, binary = cv2.threshold(l_clahe, 0, 255,
                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary = cv2.GaussianBlur(binary, (5, 5), 0)

        # 3) 只取图像下部 ROI
        roi_vertices = np.array([[
            (int(w * 0.1), int(h * 0.45)),
            (int(w * 0.9), int(h * 0.45)),
            (int(w * 0.9), int(h * 0.95)),
            (int(w * 0.1), int(h * 0.95))
        ]], dtype=np.int32)
        mask = np.zeros_like(binary)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked = cv2.bitwise_and(binary, mask)

        # 4) Canny边缘检测
        edges = cv2.Canny(masked, 50, 150)

        # 5) Hough直线检测
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=40,  # 增强检测准确度
            minLineLength=45,
            maxLineGap=30
        )

        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(x2 - x1) < 1e-5:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                length = np.hypot(x2 - x1, y2 - y1)

                # 过滤掉太平的线段
                if abs(slope) < 0.3 or length < 45:
                    continue

                mid_x = (x1 + x2) / 2
                if mid_x < w / 2:
                    left_lines.append(line)
                else:
                    right_lines.append(line)

        new_center = self.get_lane_center(left_lines, right_lines)

        # 平滑更新中心
        alpha = 0.3
        with self.center_x_lock:
            self.center_x = alpha * new_center + (1 - alpha) * self.center_x

        # 调试画线
        if self.debug_mode and self.frame_counter % 5 == 0:
            for line in left_lines[:2]:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            for line in right_lines[:2]:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    def pid_control(self):
        """PID控制舵机角度"""
        with self.center_x_lock:
            mid_final = self.center_x

        error = (self.img_width / 2) - mid_final
        self.error_sum += error
        self.error_sum = max(-1000, min(1000, self.error_sum))
        error_derivative = error - self.last_error

        error_angle = (self.kp * error +
                       self.ki * self.error_sum +
                       self.kd * error_derivative)

        error_angle = max(self.angle_outmin, min(self.angle_outmax, error_angle))
        target_angle = 88 + error_angle
        self.set_servo_angle(target_angle)

        self.last_error = error

    # 任务检测

    def detect_red_barrier_removed(self, frame):
        """检测红色挡板是否移除"""
        hsv = self.get_hsv_cached(frame)
        h, w = frame.shape[:2]
        roi = hsv[int(h * 0.4):int(h * 0.8), int(w * 0.3):int(w * 0.7)]

        red_mask1 = cv2.inRange(roi, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(roi, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_pixels = cv2.countNonZero(red_mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        red_ratio = red_pixels / total_pixels
        return red_ratio < 0.05

    def detect_zebra_crossing(self, frame):
        """增强版斑马线检测（仅在未完成停车前有效）"""
        if self.zebra_crossing_stopped:
            return False  # 已识别过，不再检测
        height = frame.shape[0]
        roi = frame[int(height * 0.65):int(height * 0.95), :]  # 更聚焦底部

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 使用自适应阈值增强对比度
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5
        )

        # 水平方向闭运算连接白色条纹
        kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 1))
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_h)

        # 垂直方向开运算分离条纹
        kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 15))
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_v)

        # 查找轮廓
        contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 筛选水平长条状区域
        valid_stripes = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            aspect_ratio = w / max(h, 1)
            # 条纹：宽 >> 高，面积适中
            if area > 800 and aspect_ratio > 3.0 and h < 40:
                valid_stripes.append((y, y + h))  # 记录垂直位置区间

        if len(valid_stripes) < 2:
            return False

        # 按垂直位置排序
        valid_stripes.sort()
        # 检查是否有至少2条间距合理的条纹（模拟斑马线交替）
        gaps = []
        for i in range(1, len(valid_stripes)):
            gap = valid_stripes[i][0] - valid_stripes[i - 1][1]
            if 5 <= gap <= 50:  # 合理间隙
                gaps.append(gap)

        # 至少有2条有效条纹且有合理间隙
        return len(gaps) >= 1

    def detect_cone(self, frame):
        """增强版锥桶检测，减少误判，适应强光环境"""

        hsv = self.get_hsv_cached(frame)
        h, w = frame.shape[:2]
        y1, y2 = int(h * 0.65), int(h * 0.85)
        x1, x2 = int(w * 0.3), int(w * 0.7)

        roi_hsv = hsv[y1:y2, x1:x2]

        # 提取黄色和红色的颜色范围
        yellow_mask = cv2.inRange(roi_hsv, self.yellow_lower, self.yellow_upper)
        red_mask1 = cv2.inRange(roi_hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(roi_hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # 形态学处理：闭运算（连接断裂的部分）→ 开运算（去掉噪声）
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, self.kernel_5x5)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.kernel_5x5)

        # 高斯模糊去噪
        yellow_mask = cv2.GaussianBlur(yellow_mask, (5, 5), 0)
        red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)

        cone_masks = {'yellow': yellow_mask, 'red': red_mask}
        detected_cones = []

        for color, mask in cone_masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 250:
                    continue

                x, y, w_roi, h_roi = cv2.boundingRect(contour)
                aspect_ratio = h_roi / max(w_roi, 1)

                # 典型锥桶：高 > 宽，面积适中
                if aspect_ratio < 2.0:
                    continue

                moments = cv2.moments(contour)
                if moments['m00'] <= 0:
                    continue

                cx = int(moments['m10'] / moments['m00']) + x1
                cy = int(moments['m01'] / moments['m00']) + y1
                detected_cones.append((cx, cy, color, area))

        if detected_cones:
            detected_cones.sort(key=lambda x: x[3], reverse=True)
            cx, cy, color, area = detected_cones[0]
            return True, cx, cy, color, area

        return False, 0, 0, None, 0

    def detect_parking_sign(self, frame):
        """检测停车标志（使用红色）"""
        if self.parking_letter_cached is not None:
            return ParkingZone.A if self.parking_letter_cached == 'A' else ParkingZone.B

        if not PYTESSERACT_AVAILABLE:
            return ParkingZone.NONE

        current_time = time.time()
        if current_time - self.last_ocr_time < self.ocr_min_interval:
            return ParkingZone.NONE

        hsv = self.get_hsv_cached(frame)
        y1, y2 = 50, 250
        x1, x2 = 100, 500
        roi_frame = frame[y1:y2, x1:x2]
        roi_hsv = hsv[y1:y2, x1:x2]

        red_mask1 = cv2.inRange(roi_hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(roi_hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1500:
                x, y, w_roi, h_roi = cv2.boundingRect(contour)
                letter_roi = roi_frame[y:y + h_roi, x:x + w_roi]

                letter = self._recognize_letter_multiple_attempts(letter_roi, "parking")

                if letter in ['A', 'B']:
                    self.parking_letter_cached = letter
                    self.last_ocr_time = current_time
                    print(f"[停车] 识别到区域: {letter}")
                    return ParkingZone.A if letter == 'A' else ParkingZone.B

        self.last_ocr_time = current_time
        return ParkingZone.NONE

    def detect_floor_letter_sign(self, frame):
        """检测地面字母（使用红色）"""
        if self.floor_letter_cached is not None:
            return self.floor_letter_cached

        if not PYTESSERACT_AVAILABLE:
            return None

        current_time = time.time()
        if current_time - self.last_ocr_time < self.ocr_min_interval:
            return None

        hsv = self.get_hsv_cached(frame)
        y1, y2 = 250, 380
        x1, x2 = 150, 450
        roi_frame = frame[y1:y2, x1:x2]
        roi_hsv = hsv[y1:y2, x1:x2]

        red_mask1 = cv2.inRange(roi_hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(roi_hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 2000:
                x, y, w_roi, h_roi = cv2.boundingRect(contour)
                letter_roi = roi_frame[y:y + h_roi, x:x + w_roi]

                letter = self._recognize_letter_multiple_attempts(letter_roi, "floor")

                if letter in ['A', 'B']:
                    self.floor_letter_cached = letter
                    self.last_ocr_time = current_time
                    print(f"[地面] 识别到字母: {letter}")
                    return letter

        self.last_ocr_time = current_time
        return None

    def _recognize_letter_multiple_attempts(self, roi, location_name):
        """OCR识别（多次尝试）"""
        if not PYTESSERACT_AVAILABLE:
            return None

        if location_name not in self.ocr_attempts:
            self.ocr_attempts[location_name] = 0

        if self.ocr_attempts[location_name] >= self.max_ocr_attempts:
            return None

        self.ocr_attempts[location_name] += 1

        try:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, thresh1 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            thresh2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY, 11, 2)
            _, thresh3 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            results = []
            for thresh in [thresh1, thresh2, thresh3]:
                pil_img = Image.fromarray(thresh)
                text = pytesseract.image_to_string(pil_img,
                                                   config='--psm 10 -c tessedit_char_whitelist=AB')
                text = text.strip().upper()
                if 'A' in text or 'B' in text:
                    results.append('A' if 'A' in text else 'B')

            if results:
                from collections import Counter
                letter = Counter(results).most_common(1)[0][0]
                return letter

        except Exception as e:
            if self.debug_mode:
                print(f"[OCR错误] {location_name}: {e}")

        return None

    def detect_qr_code(self, frame):
        """检测二维码"""
        if not PYZBAR_AVAILABLE:
            return False, None

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_codes = pyzbar.decode(gray)

            if qr_codes:
                for qr_code in qr_codes:
                    qr_data = qr_code.data.decode('utf-8')
                    print(f"[二维码] 检测到: {qr_data}")
                    return True, qr_data
        except Exception as e:
            print(f"[二维码错误] {e}")

        return False, None

    # 任务执行

    def avoid_cone_improved(self, cone_x, cone_y, frame_width):
        """改进的避障"""
        print(f"[避障] 开始避让锥桶")

        self.in_avoiding = True
        self.avoid_start_time = time.time()

        if cone_x < frame_width / 2:
            avoid_angle = 73
            print(f"[避障] 向右避让")
        else:
            avoid_angle = 107
            print(f"[避障] 向左避让")

        self.set_servo_angle(avoid_angle)

        while (time.time() - self.avoid_start_time) < self.max_avoid_duration:
            time.sleep(0.1)

        self.set_servo_angle(88)
        self.in_avoiding = False
        self.cone_detected_time = time.time()

        print(f"[避障] 避让完成")

    def perform_zebra_stop(self):
        """斑马线停车"""
        print(f"[斑马线] 停车{self.zebra_stop_duration}秒...")
        self.stop_vehicle()

        voice_played = False
        try:
            pygame.mixer.init()
            if os.path.exists("12.mp3"):
                pygame.mixer.music.load("12.mp3")
                pygame.mixer.music.set_volume(1.0)
                pygame.mixer.music.play()
                voice_played = True

                start_time = time.time()
                while time.time() - start_time < self.zebra_stop_duration:
                    if not pygame.mixer.music.get_busy():
                        time.sleep(0.1)
                    else:
                        time.sleep(0.1)
            else:
                print("[警告] 未找到12.mp3文件")
                time.sleep(self.zebra_stop_duration)
        except Exception as e:
            print(f"[语音错误] {e}")
            time.sleep(self.zebra_stop_duration)

        self.zebra_crossing_stopped = True
        self.after_zebra_crossing = True

        if voice_played:
            print(f"[斑马线] 停车完成，语音已播放")
        else:
            print(f"[斑马线] 停车完成")

        self.set_motor_speed(self.base_speed)

    def enter_parking_zone(self, target_zone):
        """进入停车区域"""
        print(f"[停车] 进入停车区域{target_zone.name}...")

        if target_zone == ParkingZone.A:
            self.set_servo_angle(105)
            self.set_motor_speed(1500)
            time.sleep(1.5)
        elif target_zone == ParkingZone.B:
            self.set_servo_angle(75)
            self.set_motor_speed(1500)
            time.sleep(1.5)

        self.stop_vehicle()
        self.parking_completed = True
        print(f"[停车] 已停入{target_zone.name}区")

    def scan_qr_and_pay(self, cap):
        """扫描二维码并支付"""
        print("[二维码] 开始扫描...")
        start_time = time.time()
        timeout = 30

        scan_count = 0

        while time.time() - start_time < timeout:
            ret, frame = cap.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (self.img_width, self.img_height))
            qr_detected, qr_data = self.detect_qr_code(frame)

            if qr_detected:
                print(f"[二维码] 检测成功: {qr_data}")
                print("[支付] 模拟支付0.01元...")
                time.sleep(1)
                print("[支付] 支付成功！")
                self.qr_code_scanned = True
                return True

            scan_count += 1
            time.sleep(0.05)

        print(f"[二维码] 扫描超时")
        return False

    def determine_parking_zone(self, floor_letter):
        """确定停车区域"""
        if floor_letter == 'A':
            return ParkingZone.A
        elif floor_letter == 'B':
            return ParkingZone.B
        return ParkingZone.NONE

    # 状态处理

    def check_state_timeout(self):
        """检查状态是否超时"""
        if self.state in self.state_timeout:
            elapsed = time.time() - self.state_start_time
            timeout = self.state_timeout[self.state]
            if elapsed > timeout:
                print(f"[超时] 状态{self.state.value}超时，强制继续")
                return True
        return False

    def change_state(self, new_state):
        """切换状态"""
        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        print(f"[状态] {self.prev_state.value} → {new_state.value}")

    def handle_waiting_start(self, frame):
        """等待发车（优化：降低检测频率）"""
        if self.should_check('barrier') and self.detect_red_barrier_removed(frame):
            print("[发车] 检测到红色挡板移除，开始计时！")
            self.start_detected = True
            self.change_state(RaceState.NORMAL_DRIVING)
            self.set_motor_speed(self.base_speed)
            self.task_start_time = time.time()

    def handle_normal_driving(self, frame, a):
        """正常行驶（优化：智能调度）"""
        # 1. 循迹控制（每帧执行）
        self.detect_lane_lines(frame)
        self.pid_control()

        # 2. 斑马线（降低频率）
        if not self.zebra_crossing_stopped and self.should_check('zebra'):
            if self.detect_zebra_crossing(frame):
                print("[检测] 斑马线")
                self.change_state(RaceState.ZEBRA_STOPPING)
                return

        # 3. 停车标志
        if (self.after_zebra_crossing and
                not self.parking_completed and
                self.parking_sign_detected == ParkingZone.NONE and
                self.should_check('sign')):
            sign = self.detect_parking_sign(frame)
            if sign != ParkingZone.NONE:
                self.parking_sign_detected = sign

        # 4. 地面字母 → 停车
        if (self.parking_sign_detected != ParkingZone.NONE and
                not self.parking_completed and
                self.should_check('letter')):
            floor_letter = self.detect_floor_letter_sign(frame)
            if floor_letter:
                target_zone = self.determine_parking_zone(floor_letter)
                if target_zone != ParkingZone.NONE:
                    self.target_parking_zone = target_zone
                    print(f"[决策] 目标停车区: {target_zone.name}")
                    self.change_state(RaceState.PARKING)
                    return

        # 5. 锥桶避障（每帧检测，有冷却）
        current_time = time.time()
        if not self.in_avoiding and (current_time - self.cone_detected_time) > self.cone_avoid_cooldown:
            if self.should_check('cone'):
                cone_detected, cone_x, cone_y, cone_color, cone_area = self.detect_cone(frame)
                if cone_detected and cone_area > 450 and a != 1:
                    a += 1
                    print(f"[检测] {cone_color}色锥桶")
                    self.avoid_cone_improved(cone_x, cone_y, frame.shape[1])

    def handle_zebra_stopping(self, frame):
        """斑马线停车"""
        self.perform_zebra_stop()
        self.change_state(RaceState.NORMAL_DRIVING)

    def handle_parking(self, frame):
        """停车"""
        self.enter_parking_zone(self.target_parking_zone)
        self.change_state(RaceState.QR_SCANNING)

    def handle_qr_scanning(self, cap):
        """扫码"""
        self.scan_qr_and_pay(cap)
        self.change_state(RaceState.FINISHED)

    # 主循环

    def main_loop(self, video_source=0):
        """主控制循环"""
        cap = cv2.VideoCapture(video_source)
        if not cap.isOpened():
            print("[错误] 无法打开摄像头")
            return
        a = 0
        print(f"\n{'=' * 60}")
        print(f"  {self.school_name} - {self.team_name}")
        print(f"  高级资源优化版控制系统")
        print(f"{'=' * 60}")
        print("[系统] 等待移除红色挡板...")

        prev_time = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[警告] 读帧失败")
                    time.sleep(0.1)
                    continue

                frame = cv2.resize(frame, (self.img_width, self.img_height))
                h_orig = frame.shape[0]
                crop_h = int(h_orig * 0.8)  # 裁剪掉图像底部无关部分
                frame = frame[0:crop_h, :]

                self.frame_counter += 1
                self.hsv_cache = None  # 每帧刷新缓存

                # 检查状态超时
                if self.check_state_timeout():
                    if self.state == RaceState.ZEBRA_STOPPING:
                        self.zebra_crossing_stopped = True
                        self.change_state(RaceState.NORMAL_DRIVING)
                    elif self.state == RaceState.PARKING:
                        self.change_state(RaceState.QR_SCANNING)
                    elif self.state == RaceState.QR_SCANNING:
                        print("[超时] 二维码扫描超时")
                        self.change_state(RaceState.FINISHED)

                # 状态机调度
                if self.state == RaceState.WAITING_START:
                    self.handle_waiting_start(frame)

                elif self.state == RaceState.NORMAL_DRIVING:
                    self.handle_normal_driving(frame, a)

                elif self.state == RaceState.ZEBRA_STOPPING:
                    self.handle_zebra_stopping(frame)

                elif self.state == RaceState.PARKING:
                    self.handle_parking(frame)

                elif self.state == RaceState.QR_SCANNING:
                    self.handle_qr_scanning(cap)
                    break

                elif self.state == RaceState.FINISHED:
                    total_time = time.time() - self.task_start_time
                    print(f"\n{'=' * 60}")
                    print(f"[完成] 所有任务完成！总用时: {total_time:.2f}秒")
                    print(f"{'=' * 60}")
                    break

                current_real_time = time.time()
                if current_real_time - self.last_save_time >= 30:
                    timestamp_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    frame_to_save = frame.copy()
                    cv2.putText(
                        frame_to_save,
                        f"Saved at: {timestamp_str}",
                        (10, 30),  # 位置：左上角
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,  # 字体大小
                        (0, 255, 0),  # 绿色
                        2  # 线宽
                    )
                    os.makedirs("./camera", exist_ok=True)
                    filename = f"frame_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                    save_path = os.path.join("./camera", filename)
                    if cv2.imwrite(save_path, frame_to_save):
                        print(f"带时间水印的图像已保存: {save_path}")
                    else:
                        print(f"保存失败: {save_path}")

                    self.last_save_time = current_real_time

                # 调试显示（大幅降低频率）
                if self.debug_mode and self.frame_counter % self.debug_update_interval == 0:
                    current_time = time.time()
                    fps = self.debug_update_interval / (current_time - prev_time + 1e-6)
                    prev_time = current_time

                    cv2.circle(frame, (int(self.center_x), 350), 5, (0, 255, 255), -1)
                    cv2.line(frame, (300, 0), (300, 400), (255, 0, 0), 1)

                    info_text = f"State: {self.state.value}"
                    cv2.putText(frame, info_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    current_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    cv2.putText(frame, f"Time: {current_time_str} | Speed: {self.current_speed}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"Center: {int(self.center_x)} | Angle: {int(self.current_servo_angle)}",
                                (10, 85),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    if self.lane_lost_count > 0:
                        cv2.putText(frame, f"Lane Lost: {self.lane_lost_count}", (10, 110),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    cv2.imshow('Race Control - Advanced Optimized', frame)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("[用户] 手动退出")
                        break
                    elif key == ord('s'):
                        if self.state == RaceState.WAITING_START:
                            print("[手动] 强制发车")
                            self.change_state(RaceState.NORMAL_DRIVING)
                            self.set_motor_speed(self.base_speed)
                            self.task_start_time = time.time()

        except KeyboardInterrupt:
            print("\n[中断] 用户中断")

        except Exception as e:
            print(f"\n[错误] 主循环异常: {e}")
            import traceback
            traceback.print_exc()

        finally:
            print("\n[清理] 正在清理资源...")
            self.emergency_stop()
            cap.release()
            cv2.destroyAllWindows()

            if HARDWARE_AVAILABLE and pi_io is not None:
                pi_io.stop()
                os.system("sudo killall pigpiod")

            print("\n[系统] 程序已安全退出")
            print(f"\n[统计] 任务完成情况:")
            print(f"  发车: {'✓' if self.start_detected else '✗'}")
            print(f"  斑马线: {'✓' if self.zebra_crossing_stopped else '✗'}")
            print(f"  停车: {'✓' if self.parking_completed else '✗'}")
            print(f"  支付: {'✓' if self.qr_code_scanned else '✗'}")


if __name__ == "__main__":
    controller = AdvancedOptimizedRaceController(
        school_name="安徽医科大学",
        team_name="杏林方舟队"
    )

    controller.main_loop(video_source=0)