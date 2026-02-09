#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 方向检测测试程序（简化版）
只输出方向检测相关数据
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
from collections import deque


# ==================== 配置参数 ====================
ANALYSIS_WINDOW_SIZE = 100    # 方向分析窗口大小
CALIBRATION_TIME = 5.0        # 校准时间（秒）
SAMPLE_RATE = 100             # 假设采样率（Hz）


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

        # 计算基础衍生数据
        filtered['acc_magnitude'] = np.sqrt(
            filtered['ax']**2 + filtered['ay']**2 + filtered['az']**2
        )

        self.last_filtered = filtered
        return filtered


# ==================== 方向检测器（核心）====================

class DirectionDetector:
    """
    方向检测器
    基于累积加速度方差判断前进方向和横向方向
    """
    def __init__(self, window_size=ANALYSIS_WINDOW_SIZE):
        self.window_size = window_size

        # 数据缓冲
        self.buffers = {
            'ax': deque(maxlen=window_size),
            'ay': deque(maxlen=window_size),
            'az': deque(maxlen=window_size),
        }

        # 检测到的方向
        self.forward_axis = None      # 'ax', 'ay', 或 'az'
        self.vertical_axis = None     # 'ax', 'ay', 或 'az'
        self.lateral_axis = None      # 'ax', 'ay', 或 'az'

        # 前进方向符号（+1 或 -1）
        self.forward_sign = 1.0

        # 校准状态
        self.is_calibrated = False
        self.calibration_count = 0

    def add_data(self, imu_filtered):
        """添加数据到缓冲区"""
        for axis in ['ax', 'ay', 'az']:
            self.buffers[axis].append(imu_filtered[axis])

    def analyze_waveform_characteristics(self, axis_data):
        """分析波形特征"""
        data = np.array(axis_data)

        # 基础统计
        variance = np.var(data)
        mean = np.mean(data)
        peak_to_peak = np.max(data) - np.min(data)

        # 计算累积加速度及其方差
        # 这是区分前进方向和横向方向的关键
        accumulated = np.cumsum(data)  # 累积加速度
        accumulated_variance = np.var(accumulated)  # 累积后的方差

        return {
            'variance': variance,
            'mean': mean,
            'peak_to_peak': peak_to_peak,
            'accumulated_variance': accumulated_variance,
        }

    def detect_directions(self):
        """
        检测前进方向、垂直方向和横向方向

        跑步时的特征：
        1. 垂直方向：均值绝对值最大（接近 ±1g 重力）
        2. 前进方向：累积方差大（持续向前，加速度累积）
        3. 横向方向：累积方差小（左右摆动，加速度正负抵消）
        """
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

        # 验证：垂直方向的均值应该 > 0.5g
        if gravity_magnitude[self.vertical_axis] < 0.5:
            self.vertical_axis = 'az'  # 默认假设

        # 2. 检测前进方向和横向方向
        remaining_axes = [ax for ax in accel_axes if ax != self.vertical_axis]

        if len(remaining_axes) == 2:
            # 计算累积加速度的方差
            accumulated_variances = {
                remaining_axes[0]: characteristics[remaining_axes[0]]['accumulated_variance'],
                remaining_axes[1]: characteristics[remaining_axes[1]]['accumulated_variance']
            }

            # 计算均值
            means = {
                remaining_axes[0]: characteristics[remaining_axes[0]]['mean'],
                remaining_axes[1]: characteristics[remaining_axes[1]]['mean']
            }

            # 使用累积加速度的方差来区分
            # - 累积方差大 = 前进方向（持续向前，加速度累积）
            # - 累积方差小 = 横向方向（左右摆动，加速度正负抵消）
            if accumulated_variances[remaining_axes[0]] > accumulated_variances[remaining_axes[1]]:
                self.forward_axis = remaining_axes[0]
                self.lateral_axis = remaining_axes[1]
            else:
                self.forward_axis = remaining_axes[1]
                self.lateral_axis = remaining_axes[0]

            # 3. 判断前进方向的符号
            forward_mean = means[self.forward_axis]
            self.forward_sign = 1.0 if forward_mean > 0 else -1.0

        # 更新校准状态
        self.calibration_count += 1
        if self.calibration_count >= 3:
            self.is_calibrated = True

        return True

    def get_forward_acceleration(self, imu_filtered):
        """获取前进方向的加速度（带符号修正）"""
        if self.forward_axis is None:
            return 0.0

        raw_forward = imu_filtered[self.forward_axis]
        return raw_forward * self.forward_sign

    def get_vertical_acceleration(self, imu_filtered):
        """获取垂直方向的加速度"""
        if self.vertical_axis is None:
            return imu_filtered['az']
        return imu_filtered[self.vertical_axis]

    def get_lateral_acceleration(self, imu_filtered):
        """获取横向加速度"""
        if self.lateral_axis is None:
            return 0.0
        return imu_filtered[self.lateral_axis]

    def reset(self):
        """重置检测器"""
        for axis in ['ax', 'ay', 'az']:
            self.buffers[axis].clear()
        self.is_calibrated = False
        self.calibration_count = 0
        self.forward_axis = None
        self.vertical_axis = None
        self.lateral_axis = None
        self.forward_sign = 1.0


# ==================== 串口处理 ====================

def find_serial_port():
    """自动查找可用的串口"""
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
    """解析 IMU 数据行"""
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

def print_status_line(now, imu_raw, imu_filtered, detector):
    """打印状态行"""
    # 方向信息
    if detector.is_calibrated:
        sign_str = "+" if detector.forward_sign > 0 else "-"
        dir_info = f"{sign_str}{detector.forward_axis}→ V:{detector.vertical_axis} L:{detector.lateral_axis}"
    else:
        dir_info = "校准中..."

    # 原始IMU数据
    raw_str = f"ax:{imu_raw['ax']:+6.3f} ay:{imu_raw['ay']:+6.3f} az:{imu_raw['az']:+6.3f}"

    # 方向加速度
    acc_f = detector.get_forward_acceleration(imu_filtered)
    acc_v = detector.get_vertical_acceleration(imu_filtered)
    acc_l = detector.get_lateral_acceleration(imu_filtered)

    dir_acc_str = f"F:{acc_f:+6.3f} V:{acc_v:+6.3f} L:{acc_l:+6.3f} g"

    print(f"[{now:7.1f}s] {dir_info:^20} | 原始: {raw_str:^40} | 方向加速度: {dir_acc_str}")


def print_calibration_info(detector):
    """打印校准信息"""
    print("\n" + "=" * 120)
    print("方向检测完成 (基于累积加速度方差):")
    print(f"  前进方向: {detector.forward_sign:+.0f}{detector.forward_axis} (累积方差大，持续向前)")
    print(f"  垂直方向: {detector.vertical_axis} (重力方向)")
    print(f"  横向方向: {detector.lateral_axis} (累积方差小，左右摆动)")
    print("=" * 120)


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

    # 初始化滤波器和方向检测器
    imu_filter = IMUFilter(filter_type='kalman')
    direction_detector = DirectionDetector()

    print("开始实时测试，按 Ctrl+C 退出...")
    print("提示: 自动检测前进方向（需要5-10秒校准时间）")
    print("      每0.5秒输出IMU数据和方向加速度\n")

    print("-" * 120)
    print("等待校准...")
    print("-" * 120)

    t0 = time.time()
    last_print = 0.0
    calibration_shown = False

    try:
        while True:
            # 读取串口
            line = ser.readline().decode(errors='ignore').strip()
            now = time.time() - t0

            # 解析原始 IMU 数据
            imu_raw = parse_imu_line(line)
            if imu_raw is None:
                continue

            # IMU 滤波
            imu_filtered = imu_filter.filter(imu_raw)

            # 方向检测
            direction_detector.add_data(imu_filtered)
            if len(direction_detector.buffers['ax']) >= ANALYSIS_WINDOW_SIZE:
                direction_detector.detect_directions()

            # 打印数据（每0.5秒）
            if now - last_print >= 0.5:
                # 首次完成校准时打印一次校准信息
                if direction_detector.is_calibrated and not calibration_shown:
                    print_calibration_info(direction_detector)
                    calibration_shown = True

                print_status_line(now, imu_raw, imu_filtered, direction_detector)
                last_print = now

    except KeyboardInterrupt:
        print("\n\n" + "=" * 120)
        print("测试结束")
        print("=" * 120)
        if direction_detector.is_calibrated:
            print_calibration_info(direction_detector)
        print("\n✓ 程序退出")

    finally:
        ser.close()


if __name__ == '__main__':
    main()
