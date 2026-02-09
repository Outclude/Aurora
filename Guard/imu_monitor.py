"""
IMU 过滤前后波形可视化
"""
import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
from collections import deque
import numpy as np


# ==================== 串口配置 ====================
# 自动检测串口或手动指定
PORT = 'COM15'         # 'auto' = 自动检测，或指定 'COM14' 等
BAUD = 115200


def find_serial_port():
    """自动查找可用的串口"""
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("未找到任何串口！")
        print("\n请检查：")
        print("  1. 设备是否已连接")
        print("  2. 驱动是否已安装")
        print("  3. 运行 python list_ports.py 查看所有串口")
        return None

    # 优先选择包含关键词的串口
    keywords = ['USB', 'UART', 'CH340', 'CP210', 'FTDI', 'Arduino', 'ESP32', 'STLink']

    for port in ports:
        for keyword in keywords:
            if keyword.upper() in port.description.upper():
                print(f"自动检测到串口: {port.device} ({port.description})")
                return port.device

    # 如果没找到，返回第一个可用串口
    print(f"使用第一个可用串口: {ports[0].device} ({ports[0].description})")
    print(f"可用串口列表: {[p.device for p in ports]}")
    return ports[0].device

# ==================== 参数 ====================
WINDOW_SEC = 10.0
ACC_LIMIT  = 20.0      # g
GYRO_LIMIT = 2000.0    # deg/s
PRINT_INTERVAL = 0.1   # s，串口打印频率（10 Hz）

# ==================== 滤波器配置 ====================
FILTER_TYPE = 'kalman'  # 'none', 'kalman', 'lowpass', 'moving_avg'
LOW_PASS_ALPHA = 0.3    # 低通滤波系数
MOVING_AVG_SIZE = 5     # 移动平均窗口大小


# ==================== Python 滤波器实现 ====================
# （与 C++ imu_filter.h 中 IMUFilter 类功能相同的 Python 实现）

class KalmanFilter:
    """卡尔曼滤波器（单变量）"""
    def __init__(self, process_noise=0.01, measurement_noise=0.1, initial_value=0.0):
        self.Q = process_noise      # 过程噪声
        self.R = measurement_noise  # 测量噪声
        self.P = 1.0                # 估计误差协方差
        self.X = initial_value      # 状态估计值

    def update(self, measurement):
        """更新滤波器"""
        # 预测步骤
        self.P = self.P + self.Q

        # 更新步骤
        K = self.P / (self.P + self.R)  # 卡尔曼增益
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P

        return self.X

    def reset(self, initial_value=0.0):
        """重置滤波器"""
        self.X = initial_value
        self.P = 1.0


class LowPassFilter:
    """一阶低通滤波器"""
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.y = 0.0  # 上次输出值

    def update(self, measurement):
        """更新滤波器: y[n] = alpha * x[n] + (1-alpha) * y[n-1]"""
        self.y = self.alpha * measurement + (1.0 - self.alpha) * self.y
        return self.y

    def reset(self):
        """重置滤波器"""
        self.y = 0.0


class MovingAvgFilter:
    """移动平均滤波器"""
    def __init__(self, size=5):
        self.size = size
        self.buffer = [0.0] * size
        self.index = 0
        self.filled = False

    def update(self, measurement):
        """更新滤波器"""
        self.buffer[self.index] = measurement
        self.index = (self.index + 1) % self.size

        if not self.filled and self.index == 0:
            self.filled = True

        count = self.size if self.filled else self.index
        return sum(self.buffer[:count]) / count

    def reset(self):
        """重置滤波器"""
        self.buffer = [0.0] * self.size
        self.index = 0
        self.filled = False


class IMUFilter:
    """
    IMU 滤波器类
    与 C++ imu_filter.h 中的 IMUFilter 类功能相同
    """
    def __init__(self, filter_type='kalman'):
        """
        初始化滤波器

        Args:
            filter_type: 滤波器类型 ('none', 'kalman', 'lowpass', 'moving_avg')
        """
        self.filter_type = filter_type

        # 为6个轴分别创建滤波器
        if filter_type == 'kalman':
            self.kalman_ax = KalmanFilter()
            self.kalman_ay = KalmanFilter()
            self.kalman_az = KalmanFilter()
            self.kalman_gx = KalmanFilter()
            self.kalman_gy = KalmanFilter()
            self.kalman_gz = KalmanFilter()

        elif filter_type == 'lowpass':
            self.lowpass_ax = LowPassFilter(LOW_PASS_ALPHA)
            self.lowpass_ay = LowPassFilter(LOW_PASS_ALPHA)
            self.lowpass_az = LowPassFilter(LOW_PASS_ALPHA)
            self.lowpass_gx = LowPassFilter(LOW_PASS_ALPHA)
            self.lowpass_gy = LowPassFilter(LOW_PASS_ALPHA)
            self.lowpass_gz = LowPassFilter(LOW_PASS_ALPHA)

        elif filter_type == 'moving_avg':
            self.moving_ax = MovingAvgFilter(MOVING_AVG_SIZE)
            self.moving_ay = MovingAvgFilter(MOVING_AVG_SIZE)
            self.moving_az = MovingAvgFilter(MOVING_AVG_SIZE)
            self.moving_gx = MovingAvgFilter(MOVING_AVG_SIZE)
            self.moving_gy = MovingAvgFilter(MOVING_AVG_SIZE)
            self.moving_gz = MovingAvgFilter(MOVING_AVG_SIZE)

        self.last_filtered = None

    def filter(self, raw_data):
        """
        滤波处理

        Args:
            raw_data: 原始IMU数据，字典格式 {'ax': x, 'ay': y, ...}

        Returns:
            滤波后的数据，字典格式
        """
        if self.filter_type == 'none':
            return raw_data.copy()

        if self.filter_type == 'kalman':
            filtered = {
                'ax': self.kalman_ax.update(raw_data['ax']),
                'ay': self.kalman_ay.update(raw_data['ay']),
                'az': self.kalman_az.update(raw_data['az']),
                'gx': self.kalman_gx.update(raw_data['gx']),
                'gy': self.kalman_gy.update(raw_data['gy']),
                'gz': self.kalman_gz.update(raw_data['gz']),
            }

        elif self.filter_type == 'lowpass':
            filtered = {
                'ax': self.lowpass_ax.update(raw_data['ax']),
                'ay': self.lowpass_ay.update(raw_data['ay']),
                'az': self.lowpass_az.update(raw_data['az']),
                'gx': self.lowpass_gx.update(raw_data['gx']),
                'gy': self.lowpass_gy.update(raw_data['gy']),
                'gz': self.lowpass_gz.update(raw_data['gz']),
            }

        elif self.filter_type == 'moving_avg':
            filtered = {
                'ax': self.moving_ax.update(raw_data['ax']),
                'ay': self.moving_ay.update(raw_data['ay']),
                'az': self.moving_az.update(raw_data['az']),
                'gx': self.moving_gx.update(raw_data['gx']),
                'gy': self.moving_gy.update(raw_data['gy']),
                'gz': self.moving_gz.update(raw_data['gz']),
            }

        else:
            return raw_data.copy()

        # 计算衍生数据
        acc_mag = np.sqrt(filtered['ax']**2 + filtered['ay']**2 + filtered['az']**2)
        acc_horiz = np.sqrt(filtered['ax']**2 + filtered['ay']**2)
        filtered['acc_magnitude'] = acc_mag
        filtered['acc_horizontal'] = acc_horiz

        self.last_filtered = filtered
        return filtered

    def reset(self):
        """重置滤波器状态"""
        if self.filter_type == 'kalman':
            for kf in [self.kalman_ax, self.kalman_ay, self.kalman_az,
                       self.kalman_gx, self.kalman_gy, self.kalman_gz]:
                kf.reset()

        elif self.filter_type == 'lowpass':
            for lp in [self.lowpass_ax, self.lowpass_ay, self.lowpass_az,
                       self.lowpass_gx, self.lowpass_gy, self.lowpass_gz]:
                lp.reset()

        elif self.filter_type == 'moving_avg':
            for ma in [self.moving_ax, self.moving_ay, self.moving_az,
                       self.moving_gx, self.moving_gy, self.moving_gz]:
                ma.reset()


# ==================== 数据缓存（原始 + 滤波后）====================
t_buf = deque()

# 原始数据
ax_raw_buf = deque()
ay_raw_buf = deque()
az_raw_buf = deque()
gx_raw_buf = deque()
gy_raw_buf = deque()
gz_raw_buf = deque()

# 滤波后数据
ax_filt_buf = deque()
ay_filt_buf = deque()
az_filt_buf = deque()
gx_filt_buf = deque()
gy_filt_buf = deque()
gz_filt_buf = deque()


# ==================== 工具函数 ====================
def sane(v, limit):
    """检查数据是否合理"""
    return abs(v) < limit


def parse_imu_line(line):
    """
    解析串口数据
    格式: ax=0.0123 ay=-0.0345 az=0.9981 | gx=1.2345 gy=-0.4567 gz=0.0789
    """
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

        ax, ay, az = data['ax'], data['ay'], data['az']
        gx, gy, gz = data['gx'], data['gy'], data['gz']

        if not (
            sane(ax, ACC_LIMIT) and sane(ay, ACC_LIMIT) and sane(az, ACC_LIMIT) and
            sane(gx, GYRO_LIMIT) and sane(gy, GYRO_LIMIT) and sane(gz, GYRO_LIMIT)
        ):
            return None

        return {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}

    except:
        return None


# ==================== Matplotlib 初始化（12个子图）====================
plt.ion()
fig, axs = plt.subplots(4, 3, figsize=(15, 10), sharex=True)
axs = axs.flatten()

# 12个子图的标签
labels = [
    'ax 原始 (g)', 'ay 原始 (g)', 'az 原始 (g)',
    'ax 滤波 (g)', 'ay 滤波 (g)', 'az 滤波 (g)',
    'gx 原始 (deg/s)', 'gy 原始 (deg/s)', 'gz 原始 (deg/s)',
    'gx 滤波 (deg/s)', 'gy 滤波 (deg/s)', 'gz 滤波 (deg/s)'
]

# 颜色：原始=红色，滤波=蓝色
colors = [
    'red', 'red', 'red',
    'blue', 'blue', 'blue',
    'red', 'red', 'red',
    'blue', 'blue', 'blue'
]

lines_raw = []
lines_filt = []

for i, (ax, label, color) in enumerate(zip(axs, labels, colors)):
    line, = ax.plot([], [], linewidth=1.2, color=color, label=label.split()[1])
    ax.set_ylabel(label, fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=7)

    # 固定 y 轴范围
    if i < 6:  # 加速度
        ax.set_ylim(-2, 2)
    else:  # 角速度
        ax.set_ylim(-500, 500)

# 分离原始和滤波后的线
for i in range(12):
    if i in [0, 1, 2, 6, 7, 8]:  # 原始数据
        lines_raw.append(axs[i])
    else:  # 滤波后数据
        lines_filt.append(axs[i])

# 设置 x 轴标签
for i in [2, 5, 8, 11]:
    axs[i].set_xlabel("Time (s)")

plt.suptitle(f'IMU 实时监视器 - 滤波器: {FILTER_TYPE.upper()}', fontsize=14, fontweight='bold')
plt.tight_layout()

# ==================== 初始化滤波器 ====================
imu_filter = IMUFilter(filter_type=FILTER_TYPE)

# ==================== 串口连接 ====================
# 自动检测或使用指定串口
port_name = PORT
if port_name == 'auto':
    port_name = find_serial_port()
    if port_name is None:
        print("\n运行 'python list_ports.py' 查看所有可用串口")
        exit(1)

print(f"正在连接串口 {port_name} @ {BAUD} baud...")

try:
    ser = serial.Serial(port_name, BAUD, timeout=0.1)
    print(f"✓ 已连接串口: {port_name}")
except serial.SerialException as e:
    print(f"✗ 串口连接失败: {e}")
    print(f"\n请运行 'python list_ports.py' 查看可用串口")
    exit(1)

# ==================== 主循环 ====================
t0 = time.time()
last_print = 0.0

print("\n开始实时监视，按 Ctrl+C 退出...\n")
print(f"滤波器类型: {FILTER_TYPE}")
print(f"时间窗口: {WINDOW_SEC} 秒")
print("=" * 60)

try:
    while True:
        # 读取串口数据
        raw = ser.readline().decode(errors='ignore').strip()
        now = time.time() - t0

        # 解析数据
        imu_raw = parse_imu_line(raw)
        if imu_raw is None:
            continue

        # ========== 滤波处理 ==========
        imu_filtered = imu_filter.filter(imu_raw)

        # ========== 串口打印（限频）==========
        if now - last_print >= PRINT_INTERVAL:
            print(f"[{now:7.3f}s] "
                  f"RAW: ax={imu_raw['ax']:+.4f} ay={imu_raw['ay']:+.4f} az={imu_raw['az']:+.4f} | "
                  f"gx={imu_raw['gx']:+.3f} gy={imu_raw['gy']:+.3f} gz={imu_raw['gz']:+.3f}")
            print(f"          "
                  f"FIL: ax={imu_filtered['ax']:+.4f} ay={imu_filtered['ay']:+.4f} az={imu_filtered['az']:+.4f} | "
                  f"gx={imu_filtered['gx']:+.3f} gy={imu_filtered['gy']:+.3f} gz={imu_filtered['gz']:+.3f}")
            last_print = now

        # ========== 存储数据 ==========
        t_buf.append(now)

        # 原始数据
        ax_raw_buf.append(imu_raw['ax'])
        ay_raw_buf.append(imu_raw['ay'])
        az_raw_buf.append(imu_raw['az'])
        gx_raw_buf.append(imu_raw['gx'])
        gy_raw_buf.append(imu_raw['gy'])
        gz_raw_buf.append(imu_raw['gz'])

        # 滤波后数据
        ax_filt_buf.append(imu_filtered['ax'])
        ay_filt_buf.append(imu_filtered['ay'])
        az_filt_buf.append(imu_filtered['az'])
        gx_filt_buf.append(imu_filtered['gx'])
        gy_filt_buf.append(imu_filtered['gy'])
        gz_filt_buf.append(imu_filtered['gz'])

        # ========== 滑动窗口 ==========
        while t_buf and (now - t_buf[0]) > WINDOW_SEC:
            t_buf.popleft()
            ax_raw_buf.popleft()
            ay_raw_buf.popleft()
            az_raw_buf.popleft()
            gx_raw_buf.popleft()
            gy_raw_buf.popleft()
            gz_raw_buf.popleft()
            ax_filt_buf.popleft()
            ay_filt_buf.popleft()
            az_filt_buf.popleft()
            gx_filt_buf.popleft()
            gy_filt_buf.popleft()
            gz_filt_buf.popleft()

        # ========== 更新绘图 ==========
        t_list = list(t_buf)

        # 更新原始数据线（红色）
        axs[0].lines[0].set_data(t_list, list(ax_raw_buf))
        axs[1].lines[0].set_data(t_list, list(ay_raw_buf))
        axs[2].lines[0].set_data(t_list, list(az_raw_buf))
        axs[6].lines[0].set_data(t_list, list(gx_raw_buf))
        axs[7].lines[0].set_data(t_list, list(gy_raw_buf))
        axs[8].lines[0].set_data(t_list, list(gz_raw_buf))

        # 更新滤波后数据线（蓝色）
        axs[3].lines[0].set_data(t_list, list(ax_filt_buf))
        axs[4].lines[0].set_data(t_list, list(ay_filt_buf))
        axs[5].lines[0].set_data(t_list, list(az_filt_buf))
        axs[9].lines[0].set_data(t_list, list(gx_filt_buf))
        axs[10].lines[0].set_data(t_list, list(gy_filt_buf))
        axs[11].lines[0].set_data(t_list, list(gz_filt_buf))

        # 更新 x 轴范围
        for ax in axs:
            ax.set_xlim(max(0, now - WINDOW_SEC), now)

        plt.pause(0.001)

except KeyboardInterrupt:
    print("\n\n正在退出...")
    ser.close()
    print("✓ 串口已关闭")
    print("✓ 程序退出")
