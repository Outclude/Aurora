#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 6DoF 数据实时读取器 + CSV 记录器
用于读取胸前 IMU 数据并保存，为跑步姿态检测算法提供数据基础
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys
import os
import csv
import threading
from datetime import datetime
from collections import deque
from pathlib import Path


class IMUReader:
    def __init__(self, port=None, baudrate=115200, timeout=0.1, 
                 save_dir="./imu_data", filename=None):
        """
        初始化 IMU 读取器
        :param port: 串口号，None 则自动检测
        :param baudrate: 波特率
        :param timeout: 读取超时时间
        :param save_dir: CSV 保存目录
        :param filename: 自定义文件名，None 则自动生成
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        
        # CSV 相关
        self.save_dir = Path(save_dir)
        self.filename = filename
        self.csv_file = None
        self.csv_writer = None
        self.csv_buffer = []
        self.buffer_lock = threading.Lock()
        self.save_thread = None
        self.last_save_time = 0
        
        # 数据缓存
        self.data_buffer = deque(maxlen=1000)
        
        # 统计
        self.frame_count = 0
        self.start_time = None
        self.current_label = "NONE"  # 当前标签：LEFT/RIGHT/NONE
        
        # 键盘监听
        self.keyboard_thread = None
        
    def setup_csv(self):
        """初始化 CSV 文件"""
        # 创建保存目录
        self.save_dir.mkdir(parents=True, exist_ok=True)
        
        # 生成文件名
        if self.filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.filename = f"imu_run_{timestamp}.csv"
        
        filepath = self.save_dir / self.filename
        
        # 检查文件是否存在，避免覆盖
        counter = 1
        original_filepath = filepath
        while filepath.exists():
            stem = original_filepath.stem
            filepath = self.save_dir / f"{stem}_{counter}.csv"
            counter += 1
        
        self.csv_path = filepath
        
        # 打开文件并写入表头
        self.csv_file = open(filepath, 'w', newline='', buffering=1)  # 行缓冲
        self.csv_writer = csv.writer(self.csv_file)
        
        # 写入表头
        headers = [
            'timestamp',          # 时间戳（秒）
            'datetime',           # 可读时间
            'ax', 'ay', 'az',     # 加速度 (g)
            'gx', 'gy', 'gz',     # 角速度 (dps)
            'label',              # 人工标签：LEFT/RIGHT/NONE
            'frame_id'            # 帧序号
        ]
        self.csv_writer.writerow(headers)
        self.csv_file.flush()
        
        print(f"✓ CSV 文件已创建: {filepath.absolute()}")
        print(f"  保存路径: {self.save_dir.absolute()}")
        
        # 启动后台保存线程（批量写入，减少磁盘 IO）
        self.save_thread = threading.Thread(target=self._background_save, daemon=True)
        self.save_thread.start()
        
        return True
    
    def _background_save(self):
        """后台线程：定期批量保存数据到 CSV"""
        while self.running or len(self.csv_buffer) > 0:
            time.sleep(0.1)  # 100ms 保存一次
            
            with self.buffer_lock:
                if len(self.csv_buffer) > 0:
                    # 批量写入
                    self.csv_writer.writerows(self.csv_buffer)
                    self.csv_file.flush()
                    batch_size = len(self.csv_buffer)
                    self.csv_buffer.clear()
                    # print(f"  [后台] 保存了 {batch_size} 行数据")
    
    def write_to_csv(self, data):
        """写入数据到 CSV 缓冲区"""
        row = [
            data['timestamp'],
            datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S.%f')[:-3],
            data.get('ax', 0), data.get('ay', 0), data.get('az', 0),
            data.get('gx', 0), data.get('gy', 0), data.get('gz', 0),
            self.current_label,
            self.frame_count
        ]
        
        with self.buffer_lock:
            self.csv_buffer.append(row)
    
    def close_csv(self):
        """关闭 CSV 文件"""
        # 等待后台线程完成
        if self.save_thread and self.save_thread.is_alive():
            self.save_thread.join(timeout=2.0)
        
        # 写入剩余数据
        with self.buffer_lock:
            if len(self.csv_buffer) > 0 and self.csv_writer:
                self.csv_writer.writerows(self.csv_buffer)
                self.csv_buffer.clear()
        
        if self.csv_file:
            self.csv_file.close()
            print(f"✓ CSV 文件已保存: {self.csv_path}")
            print(f"  总行数: {self.frame_count}")
    
    def list_ports(self):
        """列出所有可用串口"""
        ports = serial.tools.list_ports.comports()
        print("可用串口列表：")
        for i, port in enumerate(ports, 1):
            print(f"  {i}. {port.device} - {port.description}")
        return [p.device for p in ports]
    
    def auto_connect(self):
        """自动检测并连接 IMU 串口"""
        ports = self.list_ports()
        
        if not ports:
            print("错误：未找到可用串口！")
            return False
            
        if self.port and self.port in ports:
            return self.connect(self.port)
        
        for port in ports:
            if any(x in port.lower() for x in ['bluetooth', 'bt', 'wireless']):
                continue
            print(f"\n尝试连接 {port}...")
            if self.connect(port):
                return True
                
        print("错误：无法自动识别 IMU 设备，请手动指定串口号")
        return False
    
    def connect(self, port):
        """连接到指定串口"""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            self.port = port
            print(f"✓ 成功连接到 {port} @ {self.baudrate}bps")
            self.ser.reset_input_buffer()
            time.sleep(0.1)
            return True
            
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False
    
    def parse_csv_format(self, line):
        """解析 CSV 格式数据"""
        try:
            values = list(map(float, line.strip().split(',')))
            if len(values) >= 6:
                return {
                    'ax': values[0], 'ay': values[1], 'az': values[2],
                    'gx': values[3], 'gy': values[4], 'gz': values[5],
                    'timestamp': time.time()
                }
        except:
            pass
        return None
    
    def parse_witmotion_format(self, data):
        """解析 WitMotion 协议"""
        if len(data) < 11 or data[0] != 0x55:
            return None
            
        data_type = data[1]
        parsed = {'timestamp': time.time()}
        
        if data_type == 0x51:
            ax = struct.unpack('<h', bytes(data[2:4]))[0] / 32768.0 * 16
            ay = struct.unpack('<h', bytes(data[4:6]))[0] / 32768.0 * 16
            az = struct.unpack('<h', bytes(data[6:8]))[0] / 32768.0 * 16
            parsed.update({'ax': ax, 'ay': ay, 'az': az, 'type': 'accel'})
        elif data_type == 0x52:
            gx = struct.unpack('<h', bytes(data[2:4]))[0] / 32768.0 * 2000
            gy = struct.unpack('<h', bytes(data[4:6]))[0] / 32768.0 * 2000
            gz = struct.unpack('<h', bytes(data[6:8]))[0] / 32768.0 * 2000
            parsed.update({'gx': gx, 'gy': gy, 'gz': gz, 'type': 'gyro'})
        elif data_type == 0x53:
            roll = struct.unpack('<h', bytes(data[2:4]))[0] / 32768.0 * 180
            pitch = struct.unpack('<h', bytes(data[4:6]))[0] / 32768.0 * 180
            yaw = struct.unpack('<h', bytes(data[6:8]))[0] / 32768.0 * 180
            parsed.update({'roll': roll, 'pitch': pitch, 'yaw': yaw, 'type': 'angle'})
        else:
            return None
            
        return parsed
    
    def read_frame(self):
        """读取一帧数据"""
        if not self.ser or not self.ser.is_open:
            return None
            
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and ',' in line:
                    return self.parse_csv_format(line)
            
            if self.ser.in_waiting >= 11:
                header = self.ser.read(1)
                if header == b'\x55':
                    data = bytearray(header)
                    data.extend(self.ser.read(10))
                    if len(data) == 11:
                        return self.parse_witmotion_format(data)
                        
        except Exception as e:
            print(f"读取错误: {e}")
            
        return None
    
    def print_data(self, data):
        """格式化打印数据"""
        if not data:
            return
            
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        label_str = f"[{self.current_label:5}]" if self.current_label != "NONE" else "[     ]"
        
        if 'type' in data:
            if data['type'] == 'accel':
                print(f"{label_str} [{timestamp}] Accel(g):  X:{data['ax']:8.3f} Y:{data['ay']:8.3f} Z:{data['az']:8.3f}")
            elif data['type'] == 'gyro':
                print(f"{label_str} [{timestamp}] Gyro(dps): X:{data['gx']:8.2f} Y:{data['gy']:8.2f} Z:{data['gz']:8.2f}")
            elif data['type'] == 'angle':
                print(f"{label_str} [{timestamp}] Angle(°):  R:{data['roll']:8.2f} P:{data['pitch']:8.2f} Y:{data['yaw']:8.2f}")
        else:
            print(f"{label_str} [{timestamp}] "
                  f"A:[{data['ax']:7.3f},{data['ay']:7.3f},{data['az']:7.3f}] "
                  f"G:[{data['gx']:7.2f},{data['gy']:7.2f},{data['gz']:7.2f}]")
    
    def check_keyboard(self):
        """检查键盘输入（非阻塞）"""
        # 跨平台键盘检测
        try:
            if sys.platform == 'win32':
                import msvcrt
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8', errors='ignore').upper()
                    self.handle_key(key)
            else:
                # Unix/Linux/Mac 使用 select
                import select
                import termios
                import tty
                
                # 保存终端设置
                old_settings = termios.tcgetattr(sys.stdin)
                try:
                    tty.setcbreak(sys.stdin.fileno())
                    if select.select([sys.stdin], [], [], 0)[0]:
                        key = sys.stdin.read(1).upper()
                        self.handle_key(key)
                finally:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    
        except Exception as e:
            pass  # 键盘检测失败不影响主程序
    
    def handle_key(self, key):
        """处理按键"""
        if key == 'L':
            self.current_label = "LEFT"
            print(f"\n*** 标记: 左脚在前 (LEFT) ***")
        elif key == 'R':
            self.current_label = "RIGHT"
            print(f"\n*** 标记: 右脚在前 (RIGHT) ***")
        elif key == 'N':
            self.current_label = "NONE"
            print(f"\n*** 标记: 清除标签 (NONE) ***")
        elif key == 'Q':
            print(f"\n*** 退出程序 ***")
            self.running = False
    
    def start(self):
        """开始读取循环"""
        if not self.ser:
            if not self.auto_connect():
                return False
        
        # 初始化 CSV
        if not self.setup_csv():
            return False
        
        print("\n" + "="*70)
        print("IMU 数据读取开始...")
        print("快捷键: [L]左脚在前  [R]右脚在前  [N]清除标签  [Q]退出")
        print("="*70 + "\n")
        
        self.running = True
        self.start_time = time.time()
        
        try:
            while self.running:
                # 检查键盘输入
                self.check_keyboard()
                
                # 读取 IMU 数据
                data = self.read_frame()
                
                if data:
                    self.frame_count += 1
                    self.data_buffer.append(data)
                    
                    # 实时打印
                    self.print_data(data)
                    
                    # 保存到 CSV（合并 accel 和 gyro）
                    if 'type' not in data or data.get('type') in ['accel', 'gyro']:
                        self.write_to_csv(data)
                    
                else:
                    time.sleep(0.001)
                    
        except KeyboardInterrupt:
            print("\n\n用户停止 (Ctrl+C)")
        finally:
            self.stop()
            
        return True
    
    def stop(self):
        """停止并清理"""
        self.running = False
        
        # 关闭 CSV
        self.close_csv()
        
        # 关闭串口
        if self.ser:
            self.ser.close()
        
        # 打印统计
        duration = time.time() - self.start_time if self.start_time else 0
        fps = self.frame_count / duration if duration > 0 else 0
        
        print("\n" + "="*70)
        print("运行统计：")
        print(f"  总帧数: {self.frame_count}")
        print(f"  运行时间: {duration:.2f} 秒")
        print(f"  平均采样率: {fps:.1f} Hz")
        print(f"  数据文件: {self.csv_path}")
        print("="*70)


def main():
    """主函数"""
    print("IMU 6DoF 实时数据读取器 + CSV 记录器")
    print("用于跑步姿态检测数据收集\n")
    
    # 配置参数
    reader = IMUReader(
        port='COM14',           # 自动检测，或手动指定如 'COM3', '/dev/ttyUSB0'
        baudrate=115200,     # 根据 IMU 调整
        timeout=0.1,
        save_dir="./imu_data",  # CSV 保存目录
        filename=None        # 自动生成文件名，或手动指定如 "run_session_1.csv"
    )
    
    reader.start()
    
    print("\n程序结束，数据已保存。")


if __name__ == "__main__":
    main()