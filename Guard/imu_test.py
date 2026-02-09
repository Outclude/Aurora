#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 运动状态测试程序（仅使用 IMU 数据）
实时打印 IMUFilteredData 和 MotionState

功能：
1. 从串口读取原始 IMU 数据
2. 应用 IMU 滤波器
3. 基于 IMU 数据计算运动状态（速度、距离、步频）
4. 实时打印所有数据
"""

import serial
import serial.tools.list_ports
import time
import numpy as np


# ==================== 滤波器类（与 C++ imu_filter.h 功能相同）====================

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

    def reset(self):
        self.X = 0.0
        self.P = 1.0


class IMUFilter:
    """
    IMU 滤波器类
    与 C++ imu_filter.h 中的 IMUFilter 类功能相同
    """
    def __init__(self, filter_type='kalman'):
        self.filter_type = filter_type

        if filter_type == 'kalman':
            self.kalman_ax = KalmanFilter()
            self.kalman_ay = KalmanFilter()
            self.kalman_az = KalmanFilter()
            self.kalman_gx = KalmanFilter()
            self.kalman_gy = KalmanFilter()
            self.kalman_gz = KalmanFilter()

        self.last_filtered = None

    def filter(self, raw_data):
        """
        滤波处理

        Args:
            raw_data: {'ax': x, 'ay': y, 'az': z, 'gx': x, 'gy': y, 'gz': z}

        Returns:
            滤波后的数据字典
        """
        if self.filter_type == 'kalman':
            filtered = {
                'ax': self.kalman_ax.update(raw_data['ax']),
                'ay': self.kalman_ay.update(raw_data['ay']),
                'az': self.kalman_az.update(raw_data['az']),
                'gx': self.kalman_gx.update(raw_data['gx']),
                'gy': self.kalman_gy.update(raw_data['gy']),
                'gz': self.kalman_gz.update(raw_data['gz']),
            }
        else:
            filtered = raw_data.copy()

        # 计算衍生数据
        acc_mag = np.sqrt(filtered['ax']**2 + filtered['ay']**2 + filtered['az']**2)
        acc_horiz = np.sqrt(filtered['ax']**2 + filtered['ay']**2)
        filtered['acc_magnitude'] = acc_mag
        filtered['acc_horizontal'] = acc_horiz

        self.last_filtered = filtered
        return filtered

    def reset(self):
        if self.filter_type == 'kalman':
            for kf in [self.kalman_ax, self.kalman_ay, self.kalman_az,
                       self.kalman_gx, self.kalman_gy, self.kalman_gz]:
                kf.reset()


# ==================== IMU 运动状态计算器（仅基于 IMU 数据）====================

class IMUMotionStateEstimator:
    """
    IMU 运动状态估计器
    仅使用 IMU 数据计算：速度、距离、步频、运动强度等
    """
    def __init__(self):
        # 累积数据
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.current_cadence = 0.0

        # 历史数据
        self.last_update_time = 0
        self.last_speed = 0.0

        # 步态检测状态
        self.step_state = {
            'lastStepTime': 0,
            'stepCount': 0,
            'lastAccMagnitude': 0.0,
            'rising': True,
            'peakThreshold': 1.2,      # 峰值阈值（降低以更灵敏）
            'minStepInterval': 200,    # 最小步态间隔
        }

        # 速度计算参数
        self.stationary_threshold = 0.2   # 静止阈值（m/s）
        self.gravity = 9.81               # 重力加速度
        self.max_speed = 15.0             # 最大速度限制

        # 加速度偏置校准（静止时的平均加速度）
        self.accel_bias = {'ax': 0.0, 'ay': 0.0, 'az': 0.0}
        self.bias_samples = 0

        # 运动强度指标
        self.motion_intensity = 0.0      # 运动强度 [0-1]
        self.activity_level = "静止"     # 活动等级

    def update(self, imu_filtered):
        """
        更新运动状态

        Args:
            imu_filtered: 滤波后的 IMU 数据

        Returns:
            MotionState 字典
        """
        current_time = time.time() * 1000  # ms

        # 计算时间间隔
        dt = 0.0
        if self.last_update_time > 0:
            dt = (current_time - self.last_update_time) / 1000.0

        self.last_update_time = current_time

        # ====== 1. 步态检测 ======
        cadence, step_detected = self._detect_step_cadence(imu_filtered) #目前120常数

        # ====== 2. 速度计算（仅使用 IMU）======
        speed = self._calculate_imu_speed(imu_filtered, dt)
        self.current_speed = speed

        # ====== 3. 距离累积 ======
        if dt > 0:
            distance_increment = speed * dt
            self.total_distance += distance_increment

        # ====== 4. 运动强度评估 ======
        self._calculate_motion_intensity(imu_filtered, speed)

        # ====== 5. 姿态评估 ======
        posture = self._estimate_posture(imu_filtered)

        # 构建 MotionState
        motion_state = {
            'timestamp': current_time,

            # 速度
            'speed': speed,
            'speed_x': self._get_axis_speed(imu_filtered['ax'], dt),
            'speed_y': self._get_axis_speed(imu_filtered['ay'], dt),
            'speed_z': self._get_axis_speed(imu_filtered['az'], dt),

            # 距离
            'distance_increment': distance_increment if dt > 0 else 0,
            'total_distance': self.total_distance,

            # 步态
            'cadence': cadence,
            'step_detected': step_detected,
            'total_steps': self.step_state['stepCount'],

            # 运动强度
            'motion_intensity': self.motion_intensity,
            'activity_level': self.activity_level,

            # 姿态
            'posture': posture,
            'inclination': self._calculate_inclination(imu_filtered),

            # 加速度信息
            'acc_magnitude': imu_filtered['acc_magnitude'],
            'acc_horizontal': imu_filtered['acc_horizontal'],
            'acc_vertical': imu_filtered['az'],

            # 角速度信息
            'gyro_magnitude': np.sqrt(
                imu_filtered['gx']**2 +
                imu_filtered['gy']**2 +
                imu_filtered['gz']**2
            ),
        }

        self.last_speed = speed
        return motion_state

    def _detect_step_cadence(self, imu_filtered):
        """检测步频"""
        current_time = time.time() * 1000
        acc_mag = imu_filtered['acc_magnitude']
        cadence = 0.0
        step_detected = False

        # 峰值检测算法
        if self.step_state['rising']:
            if acc_mag > self.step_state['lastAccMagnitude']:
                self.step_state['lastAccMagnitude'] = acc_mag
            else:
                # 检测到峰值
                if self.step_state['lastAccMagnitude'] > self.step_state['peakThreshold']:
                    # 检查最小间隔
                    if current_time - self.step_state['lastStepTime'] >= self.step_state['minStepInterval']: #200ms
                        self.step_state['stepCount'] += 1
                        self.step_state['lastStepTime'] = current_time

                        # 计算步频（基于最近几步的平均间隔）
                        # 简化：使用固定假设值
                        cadence = 120.0  # 默认 120 步/分钟
                        step_detected = True

                self.step_state['rising'] = False
        else:
            if acc_mag < self.step_state['lastAccMagnitude']:
                self.step_state['lastAccMagnitude'] = acc_mag
            else:
                # 检测到谷值
                self.step_state['rising'] = True
                self.step_state['lastAccMagnitude'] = acc_mag

        self.current_cadence = cadence
        return cadence, step_detected #这里要改cadeence(常数=120)

    def _calculate_imu_speed(self, imu_filtered, dt):
        """从 IMU 计算速度"""
        if dt <= 0 or dt > 0.5:  # 避免异常的时间间隔
            return self.current_speed

        # 使用水平加速度计算前进速度
        forward_accel = imu_filtered['acc_horizontal'] #这个acc_horizontal的计算不对

        # 去除重力分量（假设静止时接近 0）
        if abs(forward_accel) < 0.05:
            forward_accel = 0

        # 转换为 m/s²
        forward_accel_m_s2 = forward_accel * self.gravity

        # 积分计算速度
        speed_increment = forward_accel_m_s2 * dt

        # 漂移补偿
        if abs(self.current_speed) < self.stationary_threshold:
            # 低速时增加衰减
            self.current_speed *= 0.98

        new_speed = self.current_speed + speed_increment

        # 速度约束
        new_speed = max(0.0, min(new_speed, self.max_speed))

        # 平滑处理
        if abs(new_speed - self.last_speed) > 2.0:  # 突变限制
            new_speed = self.last_speed + np.sign(new_speed - self.last_speed) * 2.0

        return new_speed

    def _get_axis_speed(self, accel, dt):
        """计算单轴速度分量"""
        if dt <= 0:
            return 0.0
        # 简单积分
        return accel * self.gravity * dt

    def _calculate_motion_intensity(self, imu_filtered, speed):
        """计算运动强度 [0-1]"""
        # 基于加速度变化和速度计算运动强度
        acc_intensity = min(imu_filtered['acc_magnitude'] / 15.0, 1.0)
        speed_intensity = min(speed / 10.0, 1.0)

        # 加权平均
        self.motion_intensity = 0.6 * acc_intensity + 0.4 * speed_intensity

        # 确定活动等级
        if self.motion_intensity < 0.1:
            self.activity_level = "静止"
        elif self.motion_intensity < 0.3:
            self.activity_level = "慢走"
        elif self.motion_intensity < 0.5:
            self.activity_level = "快走"
        elif self.motion_intensity < 0.7:
            self.activity_level = "慢跑"
        else:
            self.activity_level = "快跑"

    def _estimate_posture(self, imu_filtered):
        """估计姿态"""
        # 基于 Z 轴加速度判断
        az = imu_filtered['az']

        if abs(az - 1.0) < 0.2:
            return "平放"
        elif abs(az - 1.0) < 0.5:
            return "倾斜"
        elif az > 0.5:
            return "直立"
        else:
            return "倒置"

    def _calculate_inclination(self, imu_filtered):
        """计算倾斜角度（度）"""
        # 从加速度计算倾斜
        ax, ay, az = imu_filtered['ax'], imu_filtered['ay'], imu_filtered['az']

        # 俯仰角（绕 Y 轴）
        pitch = np.arctan2(ax, np.sqrt(ay**2 + az**2)) * 180 / np.pi

        # 横滚角（绕 X 轴）
        roll = np.arctan2(-ay, np.sqrt(ax**2 + az**2)) * 180 / np.pi

        return {'pitch': pitch, 'roll': roll}

    def reset(self):
        """重置所有状态"""
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.current_cadence = 0.0
        self.last_update_time = 0
        self.last_speed = 0.0
        self.step_state['lastStepTime'] = 0
        self.step_state['stepCount'] = 0
        self.motion_intensity = 0.0
        self.activity_level = "静止"

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
        line = line.replace('|', '')
        parts = line.strip().split()

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


def print_header():
    """打印表头"""
    print("\n" + "=" * 160)
    print(f"{'时间':>8} | {'原始IMU':^32} | {'滤波后IMU':^40} | {'速度/距离':^28} | {'步态/强度':^20} | {'姿态':^15}")
    print("-" * 160)


def print_data_row(now, imu_raw, imu_filtered, motion_state):
    """打印数据行"""
    # 原始 IMU
    raw_str = f"ax:{imu_raw['ax']:6.3f} ay:{imu_raw['ay']:6.3f} az:{imu_raw['az']:6.3f}"

    # 滤波后 IMU
    filt_str = f"ax:{imu_filtered['ax']:6.3f} ay:{imu_filtered['ay']:6.3f} az:{imu_filtered['az']:6.3f} | mag:{imu_filtered['acc_magnitude']:6.3f}"

    # 速度和距离
    vel_str = f"V:{motion_state['speed']:5.2f}m/s D:{motion_state['total_distance']:6.1f}m"

    # 步态和强度
    step_mark = "*" if motion_state['step_detected'] else " "
    gait_str = f"{step_mark}{motion_state['cadence']:5.1f}spm {motion_state['activity_level']}"

    # 姿态
    posture_str = f"{motion_state['posture']} {motion_state['inclination']['pitch']:5.1f}°"

    print(f"{now:8.3f} | {raw_str:^32} | {filt_str:^40} | {vel_str:^28} | {gait_str:^20} | {posture_str:^15}")


def print_detailed_state(motion_state):
    """打印详细状态信息"""
    print("\n" + "-" * 160)
    print(f"  速度分解: X={motion_state['speed_x']:5.2f} Y={motion_state['speed_y']:5.2f} Z={motion_state['speed_z']:5.2f} m/s | 合成速度={motion_state['speed']:5.2f} m/s")
    print(f"  距离累加: 本次增量={motion_state['distance_increment']:.4f} m, 总距离={motion_state['total_distance']:.2f} m")
    print(f"  步态检测: {motion_state['cadence']:.1f} steps/min | 总步数={motion_state['total_steps']} {'[检测到新步态]' if motion_state['step_detected'] else ''}")
    print(f"  运动强度: {motion_state['motion_intensity']:.2f} | 活动等级: {motion_state['activity_level']}")
    print(f"  姿态估计: {motion_state['posture']} | 俯仰角: {motion_state['inclination']['pitch']:.1f}° | 横滚角: {motion_state['inclination']['roll']:.1f}°")
    print(f"  加速度: 合成={motion_state['acc_magnitude']:.3f}g | 水平={motion_state['acc_horizontal']:.3f}g | 垂直={motion_state['acc_vertical']:.3f}g")
    print(f"  角速度: 合成={motion_state['gyro_magnitude']:.2f} deg/s")
    print("-" * 160)


# ==================== 主程序 ====================

def main():
    # 配置
    PORT = 'COM15'  # 或 'auto' 自动检测
    BAUD = 115200
    PRINT_INTERVAL = 0.1  # 打印间隔（秒）

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
    print("提示: 仅使用 IMU 数据计算运动状态（无需 GPS）\n")

    print_header()

    t0 = time.time()
    last_print = 0.0
    last_detailed_print = 0.0
    line_count = 0

    try:
        while True:
            # 读取串口
            line = ser.readline().decode(errors='ignore').strip()
            now = time.time() - t0

            # 解析原始 IMU 数据
            imu_raw = parse_imu_line(line)
            if imu_raw is None:
                continue

            # ====== 1. IMU 滤波 ======
            imu_filtered = imu_filter.filter(imu_raw)

            # ====== 2. 运动状态计算（仅基于 IMU）======
            motion_state = motion_estimator.update(imu_filtered)

            # ====== 3. 打印数据 ======
            if now - last_print >= PRINT_INTERVAL:
                print_data_row(now, imu_raw, imu_filtered, motion_state)
                last_print = now
                line_count += 1

                # 每 20 行打印一次详细信息
                if line_count % 20 == 0:
                    print_detailed_state(motion_state)

                # 定期打印表头
                if line_count % 50 == 0:
                    print_header()

    except KeyboardInterrupt:
        print("\n\n" + "=" * 160)
        print("测试结束")
        print("=" * 160)
        print(f"\n总距离: {motion_estimator.total_distance:.2f} m")
        print(f"当前速度: {motion_estimator.current_speed:.2f} m/s")
        print(f"总步数: {motion_estimator.step_state['stepCount']}")
        print(f"运动强度: {motion_estimator.motion_intensity:.2f} ({motion_estimator.activity_level})")
        print("\n✓ 程序退出")

    finally:
        ser.close()


if __name__ == '__main__':
    main()
