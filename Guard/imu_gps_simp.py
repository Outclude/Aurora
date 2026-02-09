import serial
import serial.tools.list_ports
import time
import numpy as np
from collections import deque
import math

EARTH_RADIUS = 6371000.0  # 地球半径，单位：米

class KalmanFilter:
    """卡尔曼滤波器（单变量）"""
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        self.Q = process_noise
        self.R = measurement_noise
        self.P = 1.0
        self.X = 0.0

    def update(self, measurement): #measurement就是任何一个imu原始值
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        return self.X
    
class IMUFilter:
    """IMU 滤波器类, 内含KalmanFilter"""
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

class FusionSpeed:
    """
    IMU与GPS融合速度计算器
    使用互补滤波融合IMU加速度积分和GPS位置差分速度
    """
    def __init__(self, alpha=0.95, gravity_compensation=True):
        """
        初始化融合速度计算器
        :param alpha: 互补滤波系数 (0-1)，接近1表示更信任IMU，接近0表示更信任GPS
        :param gravity_compensation: 是否进行重力补偿
        """
        self.alpha = alpha
        self.gravity_compensation = gravity_compensation

        # 状态变量
        self.imu_velocity = 0.0  # IMU积分得到的速度
        self.fused_velocity = 0.0  # 融合后的速度
        self.last_gps_pos = None  # 上一次GPS位置
        self.last_gps_time = None  # 上一次GPS时间
        self.last_imu_time = None  # 上一次IMU时间

    def reset(self):
        """重置所有状态"""
        self.imu_velocity = 0.0
        self.fused_velocity = 0.0
        self.last_gps_pos = None
        self.last_gps_time = None
        self.last_imu_time = None

    def compute_gps_velocity(self, gps_data, current_time):
        """
        通过GPS位置差分计算速度
        :param gps_data: GPS数据字典 {'lat': float, 'lon': float, 'sat': int}
        :param current_time: 当前时间（秒）
        :return: GPS速度 (m/s)，如果无法计算则返回None
        """
        if self.last_gps_pos is None or self.last_gps_time is None:
            self.last_gps_pos = {'lat': gps_data['lat'], 'lon': gps_data['lon']}
            self.last_gps_time = current_time
            return None

        dt = current_time - self.last_gps_time
        if dt <= 0:
            return None

        # 计算距离增量
        distance = gps_distance(
            self.last_gps_pos['lat'], self.last_gps_pos['lon'],
            gps_data['lat'], gps_data['lon']
        )

        # 更新GPS位置
        self.last_gps_pos = {'lat': gps_data['lat'], 'lon': gps_data['lon']}
        self.last_gps_time = current_time

        # 速度 = 距离 / 时间
        gps_velocity = distance / dt
        return gps_velocity

    def compute_imu_velocity(self, imu_filtered, forward_acc_key='ay', current_time=None):
        """
        通过IMU加速度积分计算速度
        :param imu_filtered: 滤波后的IMU数据 {'ax': float, 'ay': float, 'az': float, ...}
        :param forward_acc_key: 前进方向的加速度键名
        :param current_time: 当前时间（秒），用于计算dt
        :return: IMU速度 (m/s)
        """
        # 获取前进方向加速度（单位：g，需要转换为m/s²）
        acc_g = imu_filtered[forward_acc_key]
        acc_ms2 = acc_g * 9.81  # 转换为 m/s²

        # 如果提供了时间，计算dt
        if current_time is not None and self.last_imu_time is not None:
            dt = current_time - self.last_imu_time
            if dt > 0 and dt < 1.0:  # 限制dt在合理范围内
                self.imu_velocity += acc_ms2 * dt
        else:
            # 没有时间信息时，使用默认dt（需根据实际采样率调整）
            default_dt = 0.01  # 假设100Hz采样率
            self.imu_velocity += acc_ms2 * default_dt

        self.last_imu_time = current_time
        return self.imu_velocity

    def fuse(self, imu_filtered, gps_data, current_time, forward_acc_key='ay'):
        """
        融合IMU和GPS数据计算瞬时速度
        :param imu_filtered: 滤波后的IMU数据
        :param gps_data: GPS数据字典 {'lat': float, 'lon': float, 'sat': int}，如果无GPS则为None
        :param current_time: 当前时间（秒）
        :param forward_acc_key: 前进方向的加速度键名
        :return: 融合后的瞬时速度 (m/s)
        """
        # 1. 更新IMU积分速度
        imu_vel = self.compute_imu_velocity(imu_filtered, forward_acc_key, current_time)

        # 2. 尝试获取GPS速度
        gps_vel = None
        if gps_data is not None:
            gps_vel = self.compute_gps_velocity(gps_data, current_time)

        # 3. 融合策略
        if gps_vel is not None:
            # 有GPS数据时：互补滤波融合
            # 融合公式: fused = alpha * imu_vel + (1 - alpha) * gps_vel
            self.fused_velocity = self.alpha * imu_vel + (1 - self.alpha) * gps_vel
        else:
            # 无GPS数据时：使用IMU积分速度
            self.fused_velocity = imu_vel

        return self.fused_velocity

    def get_velocity(self):
        """获取当前融合速度"""
        return self.fused_velocity

    def get_imu_velocity(self):
        """获取当前IMU积分速度"""
        return self.imu_velocity

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

def parse_imu_line(line):
    """
    解析IMU数据行为dict ->
    {'ax': float, 'ay': float, 'az': float, 'gx': float, 'gy': float, 'gz': float}
    """
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
    解析 GPS 数据行为dict
    {'lat': float, 'lon': float, 'sat': int}
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
                        data['lat'] = float(value)
                    elif key in ['lon', 'lng']:
                        data['lon'] = float(value)
                    elif key in ['sats', 'sat']:
                        data['sat'] = int(value)

        if 'lat' in data and 'lon' in data and 'sat' in data:
            return data
    except:
        pass

    return None

def print_header(): #TODO：表头标上单位
    """打印表头"""
    print("-" * 120)
    # print(f"{'时间(s)':<10} {'前进加速度(g)':<10} {'总路程(m)':<10} {'前进加速度(g)':<10} {'卫星数':<10} "
    #       f"{'经度':<10} {'纬度':<10}")
    print(f'时间 - 前进加速度 - 位移 - 总路程 - 融合速度 (ax - ay - az - lat - lon - sat)')
    print("-" * 120)

def print_status_line(
    now,
    forward_acc,
    move,
    total_distance,
    fused_speed,
    ax,ay,az,
    lat,lon,sat):
    """打印状态行"""
    # print(f"{now:8.1f}s   {forward_accel:+10.3f}      {total_distance:10.2f}      "
    #       f"{num_sats:3d}       {longitude:10.6f}  {latitude:10.6f}")
    print(f'{now:8.1f}s {forward_acc:+10.3f} {move:10.3f} {total_distance:10.3f} {fused_speed:10.3f} {ax:8.3f} {ay:8.3f} {az:8.3f} {lat:10.6f} {lon:10.6f} {sat:3d}')


# ==================== 配置参数 ====================
PORT = 'COM15'  # 或 'auto'
BAUD = 115200
# ANALYSIS_WINDOW_SIZE = 100
DISPLAY_INTERVAL = 0.5  # 秒
forward_acc_key = 'ay'
crosswise_acc_key = 'az'
GPS_MIN_SATELLITES = 4  # 最小卫星数
# ==================== 滤波器类 ====================

def main():
    '''查找串口'''
    port_name = PORT
    print(f"正在连接串口 {port_name} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port_name, BAUD, timeout=0.1)
        print(f"✓ 已连接串口: {port_name}\n")
    except serial.SerialException as e:
        print(f"✗ 串口连接失败: {e}")
        return
    
    '''初始化'''
    t0 = time.time()
    imu_filter = IMUFilter(filter_type='kalman')
    # GPS 距离累积相关
    gps_start_position = None  # GPS 锁定时的初始位置
    total_distance = 0.0  # 累积总路程
    last_gps_position = {}  # 上一次的 GPS 位置，用于计算增量距离

    # 融合算法
    speed_fusion = FusionSpeed(alpha=0.95)  # alpha=0.95: 95%信任IMU，5%信任GPS
    fused_speed = 0.0  # 融合速度
    
    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            now = time.time() - t0
            # 解析数据
            imu_raw = parse_imu_line(line)
            gps_data = parse_gps_line(line)
            # print(f'line:{line}\nimu_raw:{imu_raw}\ngps_data:{gps_data}\n')
            gps_detected = False
            # _________________搜索GPS信号_________________
            gps_detected = (gps_data is not None) and (gps_data['sat'] >= GPS_MIN_SATELLITES)
            # print(f'last_gps_position: {last_gps_position} is None: {last_gps_position is None}')
            if not gps_detected:
                print(f'搜索gps信号...')
                # continue
            # elif not last_gps_position:
            if not last_gps_position:
                print(f'第一次收到GPS信号')
                gps_detected = True
                last_gps_position['lat'] = gps_data['lat']
                last_gps_position['lon'] = gps_data['lon']
                time.sleep(DISPLAY_INTERVAL)
                # 打印信息
                print_header()
                continue
            # ____________________________________________
            
            if imu_raw is not None:
                # 滤波
                imu_filtered = imu_filter.filter(imu_raw)
                # 假定前进方向\横向方向已确定
                forward_acc = imu_filtered[forward_acc_key]
                crosswise_acc = imu_filtered[crosswise_acc_key]

                # 计算融合速度
                fused_speed = speed_fusion.fuse(imu_filtered, gps_data, now, forward_acc_key)
            if (gps_data is not None) and (last_gps_position is not {}):
                # 计算距离
                # (f'last_gps_pos: {last_gps_position}')
                distance_increment = gps_distance(
                    last_gps_position['lat'], last_gps_position['lon'],
                    gps_data['lat'], gps_data['lon']
                )
                total_distance += distance_increment
                last_gps_position['lat'] = gps_data['lat']
                last_gps_position['lon'] = gps_data['lon']

            # 打印信息
            print_status_line(
                now,
                forward_acc,
                distance_increment,
                total_distance,
                fused_speed,
                imu_filtered['ax'],imu_filtered['ay'],imu_filtered['az'],
                gps_data['lat'], gps_data['lon'],gps_data['sat']
            )
            time.sleep(DISPLAY_INTERVAL)

    except KeyboardInterrupt:
        print("\n✓ 退出")

    finally:
        ser.close()
        
        
        
        
        
if __name__ == '__main__':
    main()