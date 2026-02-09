#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU + GPS 测试程序
功能：方向检测和GPS距离累积
"""

import serial
import serial.tools.list_ports
import time
import numpy as np
from collections import deque
import math

EARTH_RADIUS = 6371000.0  # 地球半径，单位：米

def gps_distance(lat1, lon1, lat2, lon2):
    """
    计算两点 GPS 经纬度之间的球面距离
    :param lat1: 第一点纬度（度）
    :param lon1: 第一点经度（度）
    :param lat2: 第二点纬度（度）
    :param lon2: 第二点经度（度）
    :return: 距离（米）
    """
    lat1, lon1, lat2, lon2 = map(math.radians,
                                 [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2)**2 + \
        math.cos(lat1) * math.cos(lat2) * \
        math.sin(dlon / 2)**2

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS * c


# ==================== 配置参数 ====================
ANALYSIS_WINDOW_SIZE = 100
DISPLAY_INTERVAL = 0.5  # 秒
GPS_MIN_SATELLITES = 3  # 最小卫星数


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


# ==================== 方向检测器 ====================

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


# ==================== GPS 处理类 ====================

# class GPSProcessor:
#     """GPS 数据处理器 - 暂时不用"""
#     def __init__(self):
#         pass
#     # 省略实现...


# ==================== 融合速度估计器 ====================

# class FusionSpeedEstimator:
#     """暂时不用"""
#     def __init__(self):
#         pass
#     # 省略实现...


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


def parse_gps_line(line):
    """
    解析 GPS 数据行
    格式: ax=xx ay=xx az=xx | gx=xx gy=xx gz=xx | lat=xx.xxxxx lng=xxx.xxxxx sat=xx
    """
    try:
        parts = line.split('|')
        data = {}

        for part in parts:
            # 将每个部分按空格分割成多个键值对
            items = part.strip().split()
            for item in items:
                if '=' in item:
                    key, value = item.split('=', 1)
                    key = key.strip()
                    value = value.strip()

                    if key == 'lat':
                        data['latitude'] = float(value)
                    elif key in ['lon', 'lng']:
                        data['longitude'] = float(value)
                    elif key in ['sats', 'sat']:
                        data['satellites'] = int(value)

        if 'latitude' in data and 'longitude' in data and 'satellites' in data:
            return data
    except:
        pass

    return None


# ==================== 打印函数 ====================

def print_header():
    """打印表头"""
    print("-" * 100)
    print(f"{'时间(s)':<10} {'前进加速度(g)':<15} {'总路程(m)':<15} {'卫星数':<10} "
          f"{'经度':<12} {'纬度':<12}")
    print("-" * 100)


def print_status_line(now, forward_accel, total_distance, num_sats, longitude, latitude):
    """打印状态行"""
    print(f"{now:8.1f}s   {forward_accel:+10.3f}      {total_distance:10.2f}      "
          f"{num_sats:3d}       {longitude:10.6f}  {latitude:10.6f}")


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
    # gps_processor = GPSProcessor()  # 暂时不用
    # fusion_estimator = FusionSpeedEstimator()  # 暂时不用

    print("开始实时测试，按 Ctrl+C 退出...")
    print("等待方向检测和 GPS 信号...\n")

    print("-" * 100)
    print("状态初始化...")
    print("-" * 100)

    t0 = time.time()
    last_print_time = 0.0

    # 状态标志
    direction_detected = False
    gps_locked = False

    # GPS 距离累积相关
    gps_start_position = None  # GPS 锁定时的初始位置
    total_distance = 0.0  # 累积总路程
    last_gps_position = None  # 上一次的 GPS 位置，用于计算增量距离

    # 当前 GPS 数据缓存（用于显示）
    current_gps_data = {
        'latitude': 0.0,
        'longitude': 0.0,
        'satellites': 0
    }

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            print(f'line: {line}')
            now = time.time() - t0

            # 解析 IMU 数据
            imu_raw = parse_imu_line(line)
            if imu_raw is not None:
                # 滤波
                imu_filtered = imu_filter.filter(imu_raw)

                # 方向检测
                direction_detector.add_data(imu_filtered)
                if len(direction_detector.buffers['ax']) >= ANALYSIS_WINDOW_SIZE:
                    direction_detector.detect_directions()

                # 检查方向是否已校准
                if not direction_detected and direction_detector.is_calibrated:
                    direction_detected = True
                    print(f"\n✓ 方向检测完成！")
                    if gps_locked:
                        print("\n开始输出数据...")
                        print_header()

                # 如果方向已校准，准备获取前进方向加速度
                if direction_detector.is_calibrated:
                    # 获取前进方向加速度
                    forward_accel = direction_detector.get_forward_acceleration(imu_filtered)

                    # 打印（每隔 DISPLAY_INTERVAL）
                    if direction_detected and gps_locked:
                        if now - last_print_time >= DISPLAY_INTERVAL:
                            print_status_line(
                                now,
                                forward_accel,
                                total_distance,
                                current_gps_data['satellites'],
                                current_gps_data['longitude'],
                                current_gps_data['latitude']
                            )
                            last_print_time = now

                continue  # 处理完 IMU 数据，继续下一行

            # 解析 GPS 数据
            gps_data = parse_gps_line(line)
            if gps_data is not None:
                # 更新 GPS 数据缓存
                current_gps_data = gps_data

                # 检查 GPS 是否锁定
                if not gps_locked and gps_data['satellites'] >= GPS_MIN_SATELLITES:
                    gps_locked = True
                    gps_start_position = (gps_data['latitude'], gps_data['longitude'])
                    last_gps_position = (gps_data['latitude'], gps_data['longitude'])
                    total_distance = 0.0
                    print(f"\n✓ GPS 信号已锁定 (卫星数: {gps_data['satellites']})")
                    if direction_detected:
                        print("\n开始输出数据...")
                        print_header()

                # 如果 GPS 已锁定，计算累积距离
                if gps_locked and gps_data['satellites'] >= GPS_MIN_SATELLITES:
                    current_position = (gps_data['latitude'], gps_data['longitude'])

                    # 计算从上一次位置到当前位置的距离
                    distance_increment = gps_distance(
                        last_gps_position[0], last_gps_position[1],
                        current_position[0], current_position[1]
                    )

                    # 累加到总路程
                    total_distance += distance_increment

                    # 更新上一次位置
                    last_gps_position = current_position

                    # 打印（每隔 DISPLAY_INTERVAL）
                    if direction_detected and gps_locked:
                        forward_accel = 0.0  # GPS 数据时不包含加速度
                        if now - last_print_time >= DISPLAY_INTERVAL:
                            print_status_line(
                                now,
                                forward_accel,
                                total_distance,
                                current_gps_data['satellites'],
                                current_gps_data['longitude'],
                                current_gps_data['latitude']
                            )
                            last_print_time = now

    except KeyboardInterrupt:
        print("\n\n" + "=" * 100)
        print("测试结束")
        print("=" * 100)
        print("\n✓ 程序退出")

    finally:
        ser.close()


if __name__ == '__main__':
    main()
