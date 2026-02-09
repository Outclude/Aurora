#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 运动状态测试程序 - 完全修正版
适用于胸口佩戴 + 跑步场景

主要修正：
1. 自动判断前进方向的符号（负值表示反方向）
2. 实时计算步频（基于相邻步态时间间隔）
3. 正确计算衍生数据（前进/横向/垂直加速度）
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
INTEGRATION_INTERVAL = 0.5    # 速度积分间隔（秒）


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

        # 计算基础衍生数据（总是需要的）
        filtered['acc_magnitude'] = np.sqrt(
            filtered['ax']**2 + filtered['ay']**2 + filtered['az']**2
        )

        self.last_filtered = filtered
        return filtered

    def calculate_directional_acceleration(self, forward_axis, vertical_axis, lateral_axis):
        """
        根据检测到的方向，计算方向相关的加速度
        返回包含前进、横向、垂直加速度的字典
        """
        if self.last_filtered is None or forward_axis is None:
            return None

        f = self.last_filtered

        # 前进方向加速度
        acc_forward = f[forward_axis]

        # 横向加速度
        acc_lateral = f[lateral_axis] if lateral_axis else 0.0

        # 垂直加速度
        acc_vertical = f[vertical_axis]

        return {
            'acc_forward': acc_forward,
            'acc_lateral': acc_lateral,
            'acc_vertical': acc_vertical,
            'forward_axis': forward_axis,
            'vertical_axis': vertical_axis,
            'lateral_axis': lateral_axis
        }


# ==================== 方向检测器（改进版）====================

class DirectionDetector:
    """
    方向检测器 - 改进版
    1. 检测前进方向轴
    2. 判断前进方向的符号（正值还是负值）
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
            'accumulated_variance': accumulated_variance,  # 新增：累积加速度的方差
        }

    def detect_directions(self):
        """检测前进方向、垂直方向和横向方向 - 完全修正版

        跑步时的特征：
        1. 垂直方向：均值绝对值最大（接近 ±1g 重力）
        2. 前进方向：方差小（持续向前加速），均值 > 0
        3. 横向方向：方差大（左右摆动），正负交替，均值接近 0
        """
        if len(self.buffers['ax']) < self.window_size:
            return False

        accel_axes = ['ax', 'ay', 'az']
        characteristics = {}

        for axis in accel_axes:
            characteristics[axis] = self.analyze_waveform_characteristics(
                list(self.buffers[axis])
            )

        # ========== 调试输出 ==========
        print(f"\n[方向检测分析] 校准次数: {self.calibration_count}")
        for axis in accel_axes:
            print(f"  {axis}: 均值={characteristics[axis]['mean']:+7.4f}, "
                  f"方差={characteristics[axis]['variance']:.4f}, "
                  f"累积方差={characteristics[axis]['accumulated_variance']:.4f}")
        # =====================

        # 1. 检测垂直方向（重力方向）
        # 垂直方向的特征：均值绝对值最大（接近 ±1g）
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

            # 计算均值（用于判断前进方向的符号）
            means = {
                remaining_axes[0]: characteristics[remaining_axes[0]]['mean'],
                remaining_axes[1]: characteristics[remaining_axes[1]]['mean']
            }

            # ✓ 修正逻辑：使用累积加速度的方差来区分
            # - 累积方差大 = 前进方向（持续向前，加速度累积）
            # - 累积方差小 = 横向方向（左右摆动，加速度正负抵消）

            # ========== 调试输出 ==========
            print(f"  剩余轴: {remaining_axes}")
            print(f"  累积方差对比: {accumulated_variances}")
            if accumulated_variances[remaining_axes[0]] > accumulated_variances[remaining_axes[1]]:
                print(f"  → {remaining_axes[0]} 累积方差更大 = 前进方向")
                self.forward_axis = remaining_axes[0]
                self.lateral_axis = remaining_axes[1]
            else:
                print(f"  → {remaining_axes[1]} 累积方差更大 = 前进方向")
                self.forward_axis = remaining_axes[1]
                self.lateral_axis = remaining_axes[0]
            print(f"  结果: 前进={self.forward_axis}, 横向={self.lateral_axis}")
            # =====================

            # 3. 判断前进方向的符号
            # 前进方向的特征：均值 > 0（向前加速为正）
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

        # 获取前进轴的原始值
        raw_forward = imu_filtered[self.forward_axis]

        # 应用符号修正
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


# ==================== 速度计算器（0.5秒独立窗口）====================

class SpeedCalculator:
    """
    速度计算器 - 修正版
    每隔0.5秒，独立计算该0.5秒内加速度的积分，得到速度
    匀速运动时速度应保持稳定，不会持续增长
    """
    def __init__(self):
        self.gravity = 9.81  # m/s²

        # 存储0.5秒内的加速度和时间
        self.accel_buffer = deque()  # 加速度 (g)
        self.time_buffer = deque()   # 时间戳 (ms)

        # 当前速度（用于距离累积）
        self.current_speed = 0.0

    def update(self, forward_accel, current_time):
        """
        更新速度计算

        Args:
            forward_accel: 前进方向加速度 (g)
            current_time: 当前时间戳 (ms)

        Returns:
            当前计算的速度 (m/s)，如果未到计算时间则返回 None
        """
        # 添加数据到缓冲区
        self.accel_buffer.append(forward_accel)
        self.time_buffer.append(current_time)

        # 每隔 INTEGRATION_INTERVAL (0.5秒) 计算一次速度
        if len(self.time_buffer) > 1:
            time_span = (self.time_buffer[-1] - self.time_buffer[0]) / 1000.0

            if time_span >= INTEGRATION_INTERVAL:
                # 计算这段时间内的速度
                speed = self._calculate_speed_from_integration()

                # 更新当前速度（用于距离累积）
                self.current_speed = speed

                # 清空缓冲区
                self.accel_buffer.clear()
                self.time_buffer.clear()

                return speed

        return None  # 还没有到计算时间

    def _calculate_speed_from_integration(self):
        """
        通过加速度积分计算速度
        v = ∫ a dt / Δt
        """
        if len(self.accel_buffer) < 2:
            return 0.0

        # 转换为列表
        accels = list(self.accel_buffer)
        times = list(self.time_buffer)

        # 计算积分：速度 = ∫a dt
        # 使用梯形法则数值积分
        integral = 0.0
        for i in range(len(accels) - 1):
            dt = (times[i+1] - times[i]) / 1000.0  # 转换为秒
            # 梯形法则：integral += 0.5 * (y[i] + y[i+1]) * dt
            avg_accel = (accels[i] + accels[i+1]) / 2.0
            integral += avg_accel * dt

        # 计算平均速度
        total_time = (times[-1] - times[0]) / 1000.0

        if total_time > 0:
            # 平均加速度 (m/s²)
            avg_accel_m_s2 = integral / total_time * self.gravity

            # 转换为速度 (m/s)
            speed = avg_accel_m_s2

            return speed

        return 0.0

    def get_current_speed(self):
        """获取当前速度（用于距离累积）"""
        return self.current_speed

    def get_buffer_info(self):
        """获取当前缓冲区内容（调试用）"""
        if len(self.accel_buffer) == 0:
            return None

        return {
            'accelerations': list(self.accel_buffer),
            'times': list(self.time_buffer),
            'count': len(self.accel_buffer)
        }


# ==================== 运动状态估计器（完全修正版）====================

class IMUMotionStateEstimator:
    """
    IMU 运动状态估计器 - 完全修正版
    使用0.5秒独立窗口计算速度，匀速运动时速度保持稳定
    """
    def __init__(self):
        # 累积数据
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.current_cadence = 0.0

        # 历史数据
        self.last_update_time = 0
        self.last_calculated_speed = 0.0  # 上次计算的速度

        # 方向检测器
        self.direction_detector = DirectionDetector()

        # 速度计算器（0.5秒独立窗口）
        self.speed_calculator = SpeedCalculator()

        # 步态检测状态
        self.step_state = {
            'lastStepTime': 0,
            'stepCount': 0,
            'lastAccValue': 0.0,
            'rising': True,
            'peakThreshold': 1.2,
            'minStepInterval': 200,
            'stepIntervals': deque(maxlen=10),  # 存储最近10个步态间隔
        }

        # 偏置校准
        self.forward_accel_bias = 0.0
        self.bias_samples = 0

    def update(self, imu_filtered):
        """更新运动状态"""
        current_time = time.time() * 1000  # ms

        # 计算时间间隔
        dt = 0.0
        if self.last_update_time > 0:
            dt = (current_time - self.last_update_time) / 1000.0

        self.last_update_time = current_time

        # ====== 1. 方向检测 ======
        self.direction_detector.add_data(imu_filtered)

        if len(self.direction_detector.buffers['ax']) >= ANALYSIS_WINDOW_SIZE:
            self.direction_detector.detect_directions()

        # ====== 2. 步态检测 ======
        cadence, step_detected = self._detect_step_cadence(imu_filtered)

        # ====== 3. 速度计算（使用0.5秒独立窗口）======
        speed = None
        if self.direction_detector.is_calibrated:
            # 获取修正后的前进方向加速度
            forward_accel = self.direction_detector.get_forward_acceleration(imu_filtered)

            # 去除偏置
            if self.bias_samples < 100:
                self.forward_accel_bias = (
                    self.forward_accel_bias * self.bias_samples + forward_accel
                ) / (self.bias_samples + 1)
                self.bias_samples += 1
            else:
                forward_accel -= self.forward_accel_bias

            # 使用速度计算器（0.5秒独立窗口）
            speed = self.speed_calculator.update(forward_accel, current_time)

            # 如果计算出了新速度，更新当前速度
            if speed is not None:
                self.current_speed = speed
                self.last_calculated_speed = speed

        # 使用当前速度（如果没有新计算，使用上次的值）
        current_speed_for_distance = self.current_speed

        # ====== 4. 距离累积 ======
        if dt > 0 and current_speed_for_distance > 0:
            distance_increment = current_speed_for_distance * dt
            self.total_distance += distance_increment
        else:
            distance_increment = 0

        # ====== 5. 构建状态输出 ======
        motion_state = {
            'timestamp': current_time,
            'speed': current_speed_for_distance,
            'distance_increment': distance_increment,
            'total_distance': self.total_distance,
            'cadence': cadence,
            'step_detected': step_detected,
            'total_steps': self.step_state['stepCount'],
            'forward_axis': self.direction_detector.forward_axis,
            'forward_sign': self.direction_detector.forward_sign,
            'vertical_axis': self.direction_detector.vertical_axis,
            'is_calibrated': self.direction_detector.is_calibrated,

            # 方向加速度（用于调试）
            'acc_forward': self.direction_detector.get_forward_acceleration(imu_filtered),
            'acc_vertical': self.direction_detector.get_vertical_acceleration(imu_filtered),
            'acc_lateral': self.direction_detector.get_lateral_acceleration(imu_filtered),

            # 缓冲区信息（调试用）
            'buffer_info': self.speed_calculator.get_buffer_info(),
        }

        return motion_state

    def _detect_step_cadence(self, imu_filtered):
        """
        检测步频 - 修正版
        实时计算相邻步态的时间间隔
        """
        current_time = time.time() * 1000

        # 使用垂直方向的加速度检测步态
        vertical_accel = self.direction_detector.get_vertical_acceleration(imu_filtered)

        # 使用绝对值检测峰值
        acc_abs = abs(vertical_accel)
        cadence = 0.0
        step_detected = False

        # 峰值检测
        if self.step_state['rising']:
            if acc_abs > self.step_state['lastAccValue']:
                self.step_state['lastAccValue'] = acc_abs
            else:
                # 检测到峰值
                if self.step_state['lastAccValue'] > self.step_state['peakThreshold']:
                    if current_time - self.step_state['lastStepTime'] >= self.step_state['minStepInterval']:
                        # 计算步态间隔
                        if self.step_state['lastStepTime'] > 0:
                            step_interval = current_time - self.step_state['lastStepTime']
                            self.step_state['stepIntervals'].append(step_interval)

                            # 实时计算步频（步/分钟）
                            # 步频 = 60000 / 间隔(ms)
                            cadence = 60000.0 / step_interval
                            print(f'here!!! cadence: {cadence}, step_interval: {step_interval}, curr_time: {current_time}, lastStepTime: {self.step_state["lastStepTime"]}')
                        else:
                            cadence = 0.0  # 第一步，无法计算步频

                        self.step_state['stepCount'] += 1
                        self.step_state['lastStepTime'] = current_time
                        step_detected = True

                self.step_state['rising'] = False
        else:
            if acc_abs < self.step_state['lastAccValue']:
                self.step_state['lastAccValue'] = acc_abs
            else:
                # 检测到谷值
                self.step_state['rising'] = True
                self.step_state['lastAccValue'] = acc_abs

        # 如果没有检测到新步态，使用历史平均步频
        if cadence == 0.0 and len(self.step_state['stepIntervals']) > 0:
            avg_interval = np.mean(list(self.step_state['stepIntervals']))
            cadence = 60000.0 / avg_interval

        self.current_cadence = cadence
        return cadence, step_detected

    def reset(self):
        """重置所有状态"""
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.current_cadence = 0.0
        self.last_update_time = 0
        self.last_calculated_speed = 0.0
        self.direction_detector.reset()
        self.step_state['lastStepTime'] = 0
        self.step_state['stepCount'] = 0
        self.step_state['lastAccValue'] = 0.0
        self.step_state['stepIntervals'].clear()
        self.forward_accel_bias = 0.0
        self.bias_samples = 0


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

def print_status_line(now, motion_state, detector):
    """打印简洁的状态行（每秒）"""
    # 方向信息
    if detector.is_calibrated:
        sign_str = "+" if detector.forward_sign > 0 else "-"
        dir_info = f"{sign_str}{detector.forward_axis}↑"
    else:
        dir_info = "校准中..."

    # 速度和距离
    speed = motion_state['speed']
    distance = motion_state['total_distance']

    # 步频
    cadence = motion_state['cadence']

    # 方向加速度（调试用）
    acc_f = motion_state['acc_forward']
    acc_v = motion_state['acc_vertical']
    acc_l = motion_state['acc_lateral']

    print(f"[{now:7.1f}s] {dir_info:^8} | "
          f"速度: {speed:5.2f} m/s | "
          f"距离: {distance:7.1f} m | "
          f"步频: {cadence:5.0} spm | "
          f"加速度: F:{acc_f:+5.2f} V:{acc_v:+5.2f} L:{acc_l:+5.2f} g")


def print_calibration_info(detector):
    """打印校准信息"""
    print("\n" + "=" * 100)
    print("方向检测完成 (基于累积加速度方差):")
    print(f"  前进方向: {detector.forward_sign:+.0f}{detector.forward_axis} (累积方差大，持续向前)")
    print(f"  垂直方向: {detector.vertical_axis} (重力方向)")
    print(f"  横向方向: {detector.lateral_axis} (累积方差小，左右摆动)")
    print("=" * 100)


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

    # 初始化滤波器和运动状态估计器
    imu_filter = IMUFilter(filter_type='kalman')
    motion_estimator = IMUMotionStateEstimator()

    print("\n开始实时测试，按 Ctrl+C 退出...")
    print("提示: 自动检测前进方向（需要5-10秒校准时间）")
    print("      每0.5秒输出速度和距离\n")

    print("-" * 100)
    print("等待校准...")
    print("-" * 100)

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
            # print(f'imu_filtered: {imu_filtered}')

            # 运动状态计算
            motion_state = motion_estimator.update(imu_filtered)

            # 打印数据（每0.5秒）
            if now - last_print >= 0.5:
                # 首次完成校准时打印一次校准信息
                if motion_state['is_calibrated'] and not calibration_shown:
                    print_calibration_info(motion_estimator.direction_detector)
                    calibration_shown = True

                print_status_line(now, motion_state, motion_estimator.direction_detector)
                last_print = now

    except KeyboardInterrupt:
        print("\n\n" + "=" * 100)
        print("测试结束")
        print("=" * 100)
        if motion_estimator.direction_detector.is_calibrated:
            print_calibration_info(motion_estimator.direction_detector)
        print(f"\n总距离: {motion_estimator.total_distance:.2f} m")
        print(f"最终速度: {motion_estimator.current_speed:.2f} m/s")
        print(f"最终步频: {motion_estimator.current_cadence:.1f} steps/min")
        print(f"总步数: {motion_estimator.step_state['stepCount']}")
        print("\n✓ 程序退出")

    finally:
        ser.close()


if __name__ == '__main__':
    main()
