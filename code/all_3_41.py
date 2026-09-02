# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
from enum import Enum
import os
import threading
import traceback

# 尝试初始化硬件（pigpio），如果失败则进入模拟模式
try:
    import pigpio as pio

    os.system("sudo killall pigpiod")
    os.system("sudo pigpiod")
    time.sleep(1)
    pi_io = pio.pi()
    HARDWARE_AVAILABLE = True
    print("[初始化] 硬件控制已启用")
except Exception as e:
    print(f"[警告] 初始化 pigpio 失败: {e}")
    pi_io = None
    HARDWARE_AVAILABLE = False


class RaceState(Enum):
    WAITING_START = "WAITING_START"
    NORMAL_DRIVING = "NORMAL_DRIVING"


class SimpleLineFollower:

    def __init__(self, school_name="安徽医科大学", team_name="杏林方舟队"):
        self.school_name = school_name
        self.team_name = team_name

        # 调试显示（True 时会显示 OpenCV 窗口）
        self.debug_mode = True

        # 硬件控制参数
        self.servo_pin = 12
        self.motor_pin = 13
        self.current_servo_angle = 90
        self.current_motor_speed = 0

        # PID 控制参数（循迹）
        self.kp = 0.28
        self.kd = 0.11
        self.ki = 0.002
        self.last_error = 0
        self.error_sum = 0
        self.angle_outmax = 18
        self.angle_outmin = -18

        # 速度控制参数
        self.base_speed = 1950   # 基础前进速度
        self.min_speed = 1900
        self.current_speed = 0
        self.speed_reduction_angle = 15  # 大弯减速阈值（这里保留自动减速逻辑）

        # 图像尺寸 / 循迹参数
        self.center_x = 300
        self.img_width = 600
        self.img_height = 450
        self.center_x_lock = threading.Lock()
        self.lane_lost_count = 0
        self.lane_lost_threshold = 10

        # 颜色检测阈值（HSV）——只保留红色，用于识别起跑挡板
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        # 形态学 kernel（这里实际上没怎么用到，留一个通用的 5x5）
        self.kernel_5x5 = np.ones((5, 5), dtype=np.uint8)

        # 帧计数器 & 处理策略
        self.frame_counter = 0
        self.process_every_n_frames = 2  # 每 2 帧处理一次，降低 CPU 占用

        # HSV 缓存
        self.hsv_cache = None
        self.hsv_frame_id = -1

        # 调试输出频率
        self.debug_update_interval = 10

        # 状态机
        self.state = RaceState.WAITING_START
        self.prev_state = RaceState.WAITING_START
        self.state_start_time = time.time()

        # 计时（统计总用时）
        self.task_start_time = 0

        # 发车标记
        self.start_detected = False

        self._init_hardware()
        print("[初始化] 只循迹模式就绪")

    # -------------------- 硬件控制相关 --------------------

    def _init_hardware(self):
        """初始化舵机、电机。如果没有硬件则进入模拟模式"""
        if not HARDWARE_AVAILABLE or pi_io is None:
            print("[控制] 硬件控制已禁用（模拟模式）")
            return

        try:
            # 舵机：50Hz，range=100，对应后面用占空比直接写百分比
            pi_io.set_PWM_frequency(self.servo_pin, 50)
            pi_io.set_PWM_range(self.servo_pin, 100)
            self.set_servo_angle(88)  # 中位

            # 电机：200Hz，range=40000
            pi_io.set_mode(self.motor_pin, pio.OUTPUT)
            pi_io.set_PWM_frequency(self.motor_pin, 200)
            pi_io.set_PWM_range(self.motor_pin, 40000)
            self.set_motor_speed(0)

            print("[硬件] 舵机和电机初始化成功")
        except Exception as e:
            print(f"[错误] 硬件初始化失败: {e}")

    def angle_to_duty_cycle(self, angle):
        """舵机角度 → 占空比（百分比）"""
        return 2.49 + (angle / 178.0) * 8

    def set_servo_angle(self, angle):
        """设置舵机角度，带死区，避免频繁发命令"""
        angle = max(0, min(180, angle))
        if abs(angle - self.current_servo_angle) < 1:
            return

        if HARDWARE_AVAILABLE and pi_io is not None:
            duty = self.angle_to_duty_cycle(angle)
            pi_io.set_PWM_dutycycle(self.servo_pin, duty)

        self.current_servo_angle = angle

    def set_motor_speed(self, speed):
        """
        设置电机速度：
        - speed <= 0 时认为是停止
        - 正值向前
        - 内部会根据转向角自动减速
        """
        if speed <= 0:
            target_speed = 0
        else:
            target_speed = int(speed)

            # 根据当前转向角自动减速（大角度转弯时减速）
            angle_deviation = abs(self.current_servo_angle - 88)
            if angle_deviation > self.speed_reduction_angle:
                reduction_factor = 1.0 - (angle_deviation - self.speed_reduction_angle) / 30.0
                reduction_factor = max(0.4, min(1.0, reduction_factor))
                target_speed = int(target_speed * reduction_factor)
                if target_speed < self.min_speed:
                    target_speed = self.min_speed

        # 限幅
        target_speed = max(-3000, min(20000, target_speed))

        # 小变化忽略，减少 PWM 调用
        if abs(target_speed - self.current_motor_speed) < 50:
            return

        if HARDWARE_AVAILABLE and pi_io is not None:
            output = 10000 + int(target_speed)
            output = max(6000, min(29000, output))
            pi_io.set_PWM_dutycycle(self.motor_pin, output)

        self.current_motor_speed = target_speed
        self.current_speed = target_speed

    def stop_vehicle(self):
        """正常停车（电机停、舵机回中）"""
        self.set_motor_speed(0)
        self.set_servo_angle(88)

    def emergency_stop(self):
        """紧急停车（完全断 PWM）"""
        print("[紧急] 紧急停止！")
        self.stop_vehicle()
        if HARDWARE_AVAILABLE and pi_io is not None:
            pi_io.set_PWM_dutycycle(self.motor_pin, 0)

    # -------------------- 图像 / 循迹相关 --------------------

    def get_hsv_cached(self, frame):
        """同一帧只做一次 BGR→HSV 转换"""
        if self.hsv_frame_id != self.frame_counter:
            self.hsv_cache = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            self.hsv_frame_id = self.frame_counter
        return self.hsv_cache

    def get_lane_center(self, left_lines, right_lines):
        """根据左右车道线估计赛道中心位置"""
        y_bottom = 200

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
                except Exception:
                    continue

        if right_lines:
            for line in right_lines[:3]:
                try:
                    x = get_x_at_y(line, y_bottom)
                    if 0 <= x <= self.img_width:
                        right_x_list.append(x)
                except Exception:
                    continue

        if left_x_list and right_x_list:
            left_avg = sum(left_x_list) / len(left_x_list)
            right_avg = sum(right_x_list) / len(right_x_list)
            new_center = (left_avg + right_avg) / 2
            self.lane_lost_count = 0
        elif left_x_list:
            new_center = sum(left_x_list) / len(left_x_list) + 180
            self.lane_lost_count = 0
        elif right_x_list:
            new_center = sum(right_x_list) / len(right_x_list) - 180
            self.lane_lost_count = 0
        else:
            # 没检测到线，使用上一次结果，并记录丢线次数
            self.lane_lost_count += 1
            new_center = self.center_x

        new_center = max(0, min(self.img_width, new_center))
        return new_center

    def detect_lane_lines(self, frame):
        """检测赛道线，更新 center_x"""
        h, w = frame.shape[:2]

        # 灰度 + Canny 边缘
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # ROI（下半部分中间区域）
        roi_vertices = np.array([[
            (int(w * 0.15), int(h * 0.45)),
            (int(w * 0.85), int(h * 0.45)),
            (int(w * 0.85), int(h * 0.90)),
            (int(w * 0.15), int(h * 0.90))
        ]], dtype=np.int32)

        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # 霍夫直线检测
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=15,
            minLineLength=30,
            maxLineGap=25
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
                if abs(slope) > 0.2 and length > 35:
                    mid_x = (x1 + x2) / 2
                    if mid_x < w / 2:
                        left_lines.append(line)
                    else:
                        right_lines.append(line)

        new_center = self.get_lane_center(left_lines, right_lines)
        alpha = 0.4
        with self.center_x_lock:
            self.center_x = alpha * new_center + (1 - alpha) * self.center_x

        # 调试：画出检测到的线
        if self.debug_mode and self.frame_counter % 5 == 0:
            for line in left_lines[:2]:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            for line in right_lines[:2]:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    def pid_control(self):
        """根据 center_x 做 PID 控制舵机"""
        with self.center_x_lock:
            mid_final = self.center_x

        # 误差：图像中点相对于赛道中心的偏差
        error = (self.img_width / 2) - mid_final

        # 积分项防止发散
        self.error_sum += error
        self.error_sum = max(-1000, min(1000, self.error_sum))

        # 微分项
        error_derivative = error - self.last_error

        error_angle = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * error_derivative
        )

        # 输出限幅
        error_angle = max(self.angle_outmin, min(self.angle_outmax, error_angle))
        target_angle = 88 + error_angle
        self.set_servo_angle(target_angle)

        self.last_error = error

    # -------------------- 起跑挡板检测（可选） --------------------

    def detect_red_barrier_removed(self, frame):
        """
        检测红色挡板是否移除：
        在中间区域统计红色像素比例，当比例很低时认为挡板已经拿走。
        """
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

    # -------------------- 状态处理函数 --------------------

    def change_state(self, new_state):
        self.prev_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        print(f"[状态] {self.prev_state.value} → {new_state.value}")

    def handle_waiting_start(self, frame):
        """等待红挡板移除，移除后发车并切到 NORMAL_DRIVING"""
        if self.detect_red_barrier_removed(frame):
            print("[发车] 检测到红色挡板移除，开始计时！")
            self.start_detected = True
            self.change_state(RaceState.NORMAL_DRIVING)
            self.set_motor_speed(self.base_speed)
            self.task_start_time = time.time()

    def handle_normal_driving(self, frame):
        """正常行驶：只做循迹"""
        # 1. 赛道线检测
        self.detect_lane_lines(frame)

        # 2. PID 转向控制
        self.pid_control()

        # 3. 如果因为其他原因速度变成 0，自动恢复基础速度
        if self.current_speed == 0:
            self.set_motor_speed(self.base_speed)

    # -------------------- 主循环 --------------------

    def main_loop(self, video_source=0):
        time.sleep(0.03)
        cap = cv2.VideoCapture(video_source)
        if not cap.isOpened():
            print("[错误] 无法打开摄像头")
            return

        print("\n" + "=" * 60)
        print(f"  {self.school_name} - {self.team_name}")
        print("  只循迹控制系统")
        print("=" * 60)
        print("[系统] 等待红色挡板移除...（按 s 可强制发车）")

        prev_time = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[警告] 读帧失败")
                    time.sleep(0.1)
                    continue

                # 统一分辨率
                frame = cv2.resize(frame, (self.img_width, self.img_height))
                h_orig = frame.shape[0]
                crop_h = int(h_orig * 0.8)
                frame = frame[0:crop_h, :]

                self.frame_counter += 1

                # 非处理帧：只负责显示和键盘响应
                if self.frame_counter % self.process_every_n_frames != 0:
                    if self.debug_mode:
                        cv2.imshow("Line Follower", frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord("q"):
                            print("[用户] 手动退出")
                            break
                    continue

                # 将要处理这一帧：清空 HSV 缓存
                self.hsv_cache = None

                # 根据状态机执行逻辑
                if self.state == RaceState.WAITING_START:
                    self.handle_waiting_start(frame)
                elif self.state == RaceState.NORMAL_DRIVING:
                    self.handle_normal_driving(frame)

                # 调试信息：每 debug_update_interval 帧打印一次
                if self.debug_mode and self.frame_counter % self.debug_update_interval == 0:
                    current_time = time.time()
                    fps = self.debug_update_interval / (current_time - prev_time + 1e-6)
                    prev_time = current_time

                    # 画出中心点与中线
                    cv2.circle(frame, (int(self.center_x), frame.shape[0] - 10),
                               5, (0, 255, 255), -1)
                    mid_x = self.img_width // 2
                    cv2.line(frame, (mid_x, 0), (mid_x, frame.shape[0]),
                             (255, 0, 0), 1)

                    info_text = f"State: {self.state.value} | FPS: {fps:.1f}"
                    cv2.putText(frame, info_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    current_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    cv2.putText(frame, f"Time: {current_time_str} | Speed: {self.current_speed}",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 2)
                    cv2.putText(frame, f"Center: {int(self.center_x)} | Angle: {int(self.current_servo_angle)}",
                                (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 0, 0), 2)

                    if self.lane_lost_count > 0:
                        cv2.putText(frame, f"Lane Lost: {self.lane_lost_count}", (10, 110),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    cv2.imshow("Line Follower", frame)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        print("[用户] 手动退出")
                        break
                    elif key == ord("s"):
                        # 按 s 强制发车（不等红挡板）
                        if self.state == RaceState.WAITING_START:
                            print("[手动] 强制发车")
                            self.change_state(RaceState.NORMAL_DRIVING)
                            self.set_motor_speed(self.base_speed)
                            self.task_start_time = time.time()

        except KeyboardInterrupt:
            print("\n[中断] 用户中断")
        except Exception as e:
            print(f"\n[错误] 主循环异常: {e}")
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
            if self.start_detected and self.task_start_time > 0:
                total_time = time.time() - self.task_start_time
                print(f"[统计] 发车后总用时: {total_time:.2f} 秒")


if __name__ == "__main__":
    controller = SimpleLineFollower(
        school_name="安徽医科大学",
        team_name="杏林方舟队"
    )
    controller.main_loop(video_source=0)
