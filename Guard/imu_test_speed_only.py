#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 速度计算测试程序（修正版）
功能：每0.5秒独立计算该时间段内的速度（通过加速度积分）
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
from collections import deque


# ==================== 配置参数 ====================
ANALYSIS_WINDOW_SIZE = 100
INTEGRATION_INTERVAL = 0.5  # 秒


# ==================== 滤波器类 ====================

class KalmanFilter:
    """卡尔曼滤波器（单变量）"""
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        self.Q = process_noise
        self.R = measurement_noise
        self.P = 1.0
        self.X = 0.0

    def update(self, measurement):
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        return self.X


class IMUFilter:
    """IMU 滤波器类"""
    def __init__(self, filter_type='kalman'):
        self.filter_type = filter_type
        if filter_type == 'kalman':
            self.kalman_filters = {
                'ax': KalmanFilter(), 'ay': KalmanFilter(), 'az': KalmanFilter(),
                'gx': KalmanFilter(), 'gy': KalmanFilter(), 'gz': KalmanFilter()
            }
        self.last_filtered = None

    def filter(self, raw_data):
        if self.filter_type == 'kalman':
            filtered = {
                'ax': self.kalman_filters['ax'].update(raw_data['ax']),
                'ay': self.kalman_filters['ay'].update(raw_data['ay']),
                'az': self.kalman_filters['az'].update(raw_data['az']),
                'gx': self.kalman_filters['gx'].update(raw_data['gx']),
                'gy': self.kalman_filters['gy'].update(raw_data['gy']),
                'gz': self.kalman_filters['gz'].update(raw_data['gz']),
            }
        else:
            filtered = raw_data.copy()

        self.last_filtered = filtered
        return filtered


# ==================== 方向检测器（已验证正确）====================

class DirectionDetector:
    """方向检测器"""
    def __init__(self, window_size=ANALYSIS_WINDOW_SIZE):
        self.window_size = window_size
        self.buffers = {
            'ax': deque(maxlen=window_size),
            'ay': deque(maxlen=window_size),
            'az': deque(maxlen=window_size),
        }
        self.forward_axis = None
        self.vertical_axis = None
        self.lateral_axis = None
        self.forward_sign = 1.0
        self.is_calibrated = False
        self.calibration_count = 0

    def add_data(self, imu_filtered):
        for axis in ['ax', 'ay', 'az']:
            self.buffers[axis].append(imu_filtered[axis])

    def analyze_waveform_characteristics(self, axis_data):
        data = np.array(axis_data)
        accumulated = np.cumsum(data)
        accumulated_variance = np.var(accumulated)
        return {
            'variance': np.var(data),
            'mean': np.mean(data),
            'accumulated_variance': accumulated_variance,
        }

    def detect_directions(self):
        if len(self.buffers['ax']) < self.window_size:
            return False

        accel_axes = ['ax', 'ay', 'az']
        characteristics = {}

        for axis in accel_axes:
            characteristics[axis] = self.analyze_waveform_characteristics(
                list(self.buffers[axis])
            )

        # 1. 检测垂直方向（重力方向）
        gravity_magnitude = {}
        for axis in accel_axes:
            gravity_magnitude[axis] = abs(characteristics[axis]['mean'])

        self.vertical_axis = max(gravity_magnitude, key=gravity_magnitude.get)

        # 2. 检测前进方向和横向方向
        remaining_axes = [ax for ax in accel_axes if ax != self.vertical_axis]

        if len(remaining_axes) == 2:
            accumulated_variances = {
                remaining_axes[0]: characteristics[remaining_axes[0]]['accumulated_variance'],
                remaining_axes[1]: characteristics[remaining_axes[1]]['accumulated_variance']
            }

            means = {
                remaining_axes[0]: characteristics[remaining_axes[0]]['mean'],
                remaining_axes[1]: characteristics[remaining_axes[1]]['mean']
            }

            # 累积方差大 = 前进方向
            if accumulated_variances[remaining_axes[0]] > accumulated_variances[remaining_axes[1]]:
                self.forward_axis = remaining_axes[0]
                self.lateral_axis = remaining_axes[1]
            else:
                self.forward_axis = remaining_axes[1]
                self.lateral_axis = remaining_axes[0]

            # 判断前进方向的符号
            forward_mean = means[self.forward_axis]
            self.forward_sign = 1.0 if forward_mean > 0 else -1.0

            # 调试输出（只在第一次校准时打印）
            if self.calibration_count == 0:
                print(f"\n[方向检测完成]")
                print(f"  垂直方向: {self.vertical_axis}")
                print(f"  前进方向: {self.forward_sign:+.0f}{self.forward_axis}")
                print(f"  横向方向: {self.lateral_axis}")

        self.calibration_count += 1
        if self.calibration_count >= 3:
            self.is_calibrated = True

        return True

    def get_forward_acceleration(self, imu_filtered):
        if self.forward_axis is None:
            return 0.0
        return imu_filtered[self.forward_axis] * self.forward_sign

    def get_lateral_acceleration(self, imu_filtered):
        if self.lateral_axis is None:
            return 0.0
        return imu_filtered[self.lateral_axis]

    def reset(self):
        for axis in ['ax', 'ay', 'az']:
            self.buffers[axis].clear()
        self.is_calibrated = False
        self.calibration_count = 0
        self.forward_axis = None
        self.vertical_axis = None
        self.lateral_axis = None
        self.forward_sign = 1.0


# ==================== 速度计算器（修正版）====================

class SpeedCalculator:
    """
    加速度积分计算器
    每隔0.5秒，计算该0.5秒内加速度的积分（速度变化量）
    注意：这里不计算绝对速度，而是计算加速度的积分值
    """
    def __init__(self):
        # 存储0.5秒内的加速度和时间
        self.accel_buffer = deque()  # 原始加速度 (g)
        self.time_buffer = deque()   # 时间戳 (ms)

    def update(self, forward_accel_filtered, current_time):
        """
        更新加速度数据并计算积分

        Args:
            forward_accel_filtered: 前进方向滤波后加速度 (g)
            current_time: 当前时间戳 (ms)

        Returns:
            integral 如果到了计算时间（最近0.5秒内的加速度积分，单位m/s）
            None 如果还没到计算时间
        """
        # 添加数据到缓冲区
        self.accel_buffer.append(forward_accel_filtered)
        self.time_buffer.append(current_time)

        # 每隔 INTEGRATION_INTERVAL (0.5秒) 计算一次积分
        if len(self.time_buffer) > 1:
            time_span = (self.time_buffer[-1] - self.time_buffer[0]) / 1000.0

            if time_span >= INTEGRATION_INTERVAL:
                # 计算这段时间内的加速度积分
                integral = self._calculate_integral()

                # 清空缓冲区
                self.accel_buffer.clear()
                self.time_buffer.clear()

                return integral

        return None  # 还没有到计算时间

    def _calculate_integral(self):
        """
        通过加速度积分计算速度变化量
        Δv = ∫ a dt
        """
        if len(self.accel_buffer) < 2:
            return 0.0

        # 转换为列表
        accels = list(self.accel_buffer)
        times = list(self.time_buffer)

        # 计算积分：速度变化 = ∫a dt
        # 使用梯形法则数值积分
        integral = 0.0
        for i in range(len(accels) - 1):
            dt = (times[i+1] - times[i]) / 1000.0  # 转换为秒
            # 梯形法则：integral += 0.5 * (y[i] + y[i+1]) * dt
            avg_accel = (accels[i] + accels[i+1]) / 2.0
            integral += avg_accel * dt

        # integral 的单位是 g·s，乘以 9.81 得到 m/s
        integral_m_s = integral * 9.81

        return integral_m_s


# ==================== 串口处理 ====================

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None

    keywords = ['USB', 'UART', 'CH340', 'CP210', 'FTDI', 'Arduino', 'ESP32']
    for port in ports:
        for keyword in keywords:
            if keyword.upper() in port.description.upper():
                return port.device

    return ports[0].device if ports else None


def parse_imu_line(line):
    keys = ['ax=', 'ay=', 'az=', 'gx=', 'gy=', 'gz=']
    if not all(k in line for k in keys):
        return None

    try:
        line = line.replace('|', '').strip()
        parts = line.split()

        data = {}
        for p in parts:
            k, v = p.split('=')
            data[k] = float(v)

        return {
            'ax': data['ax'], 'ay': data['ay'], 'az': data['az'],
            'gx': data['gx'], 'gy': data['gy'], 'gz': data['gz']
        }
    except:
        return None


# ==================== 打印函数 ====================

def print_status_line(now, accel_raw, integral):
    """
    打印状态行

    Args:
        now: 当前时间 (s)
        accel_raw: 当前时刻前进方向原始加速度 (g)
        integral: 最近0.5秒内加速度的积分 (m/s)
    """
    print(f"[{now:7.1f}s] "
          f"前进方向原始加速度: {accel_raw:+6.3f} g, "
          f"最近0.5s内加速度积分: {integral:+5.2f} m/s")


# ==================== 主程序 ====================

def main():
    # 配置
    PORT = 'COM15'  # 或 'auto'
    BAUD = 115200

    # 查找串口
    port_name = PORT
    if PORT == 'auto':
        port_name = find_serial_port()
        if port_name is None:
            print("未找到可用串口！")
            return

    print(f"正在连接串口 {port_name} @ {BAUD} baud...")

    try:
        ser = serial.Serial(port_name, BAUD, timeout=0.1)
        print(f"✓ 已连接串口: {port_name}\n")
    except serial.SerialException as e:
        print(f"✗ 串口连接失败: {e}")
        return

    # 初始化
    imu_filter = IMUFilter(filter_type='kalman')
    direction_detector = DirectionDetector()
    speed_calculator = SpeedCalculator()

    print("开始实时测试，按 Ctrl+C 退出...")
    print("每0.5秒计算并显示加速度积分（基于0.5秒内的原始加速度）\n")

    print("-" * 100)
    print("等待方向检测...")
    print("-" * 100)

    t0 = time.time()

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            now = time.time() - t0

            # 解析 IMU 数据
            imu_raw = parse_imu_line(line)
            if imu_raw is None:
                continue

            # 滤波（用于方向检测）
            imu_filtered = imu_filter.filter(imu_raw)

            # 方向检测
            direction_detector.add_data(imu_filtered)
            if len(direction_detector.buffers['ax']) >= ANALYSIS_WINDOW_SIZE:
                direction_detector.detect_directions()

            # 只有校准完成后才计算加速度积分
            if direction_detector.is_calibrated:
                current_time = time.time() * 1000

                # 获取前进方向加速度（滤波后）
                forward_accel_filtered = direction_detector.get_forward_acceleration(imu_filtered)

                # 获取前进方向原始加速度（仅用于显示）
                forward_axis = direction_detector.forward_axis
                forward_sign = direction_detector.forward_sign
                forward_accel_raw = imu_raw[forward_axis] * forward_sign

                # 计算加速度积分（每0.5秒，使用滤波后的数据）
                result = speed_calculator.update(forward_accel_filtered, current_time)

                # 打印（每0.5秒）
                if result is not None:
                    integral = result
                    print_status_line(now, forward_accel_raw, integral)

    except KeyboardInterrupt:
        print("\n\n" + "=" * 100)
        print("测试结束")
        print("=" * 100)
        print("\n✓ 程序退出")

    finally:
        ser.close()


if __name__ == '__main__':
    main()
