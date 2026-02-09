#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 6DoF 数据实时读取器
用于读取胸前 IMU 的加速度 + 陀螺仪数据
为跑步姿态检测（左脚/右脚在前）算法提供数据基础
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys
from datetime import datetime
from collections import deque


class IMUReader:
    def __init__(self, port=None, baudrate=115200, timeout=0.1):
        """
        初始化 IMU 读取器
        :param port: 串口号，None 则自动检测
        :param baudrate: 波特率，常见 115200, 921600
        :param timeout: 读取超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        
        # 数据缓存（用于后续算法）
        self.data_buffer = deque(maxlen=1000)  # 保存最近 1000 帧数据
        
        # 统计数据
        self.frame_count = 0
        self.start_time = None
        
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
            print("请检查：1. IMU 是否插好  2. 驱动是否安装")
            return False
            
        # 如果指定了端口，直接尝试连接
        if self.port and self.port in ports:
            return self.connect(self.port)
        
        # 否则尝试每个端口（排除 obvious 的非 IMU 设备）
        for port in ports:
            # 跳过蓝牙和其他常见非 IMU 设备
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
            
            # 清空缓冲区
            self.ser.reset_input_buffer()
            time.sleep(0.1)
            return True
            
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False
    
    def parse_csv_format(self, line):
        """
        解析 CSV 格式数据 (如: "0.12,-0.05,1.02,0.01,-0.02,0.05")
        顺序: Ax, Ay, Az, Gx, Gy, Gz
        """
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
        """
        解析 WitMotion 协议 (0x55 开头)
        常见于 WT901、JY901 等模块
        """
        if len(data) < 11 or data[0] != 0x55:
            return None
            
        data_type = data[1]
        parsed = {'timestamp': time.time()}
        
        # 加速度包 (0x51)
        if data_type == 0x51:
            ax = struct.unpack('<h', bytes(data[2:4]))[0] / 32768.0 * 16  # ±16g
            ay = struct.unpack('<h', bytes(data[4:6]))[0] / 32768.0 * 16
            az = struct.unpack('<h', bytes(data[6:8]))[0] / 32768.0 * 16
            parsed.update({'ax': ax, 'ay': ay, 'az': az, 'type': 'accel'})
            
        # 角速度包 (0x52)
        elif data_type == 0x52:
            gx = struct.unpack('<h', bytes(data[2:4]))[0] / 32768.0 * 2000  # ±2000°/s
            gy = struct.unpack('<h', bytes(data[4:6]))[0] / 32768.0 * 2000
            gz = struct.unpack('<h', bytes(data[6:8]))[0] / 32768.0 * 2000
            parsed.update({'gx': gx, 'gy': gy, 'gz': gz, 'type': 'gyro'})
            
        # 角度包 (0x53)
        elif data_type == 0x53:
            roll = struct.unpack('<h', bytes(data[2:4]))[0] / 32768.0 * 180
            pitch = struct.unpack('<h', bytes(data[4:6]))[0] / 32768.0 * 180
            yaw = struct.unpack('<h', bytes(data[6:8]))[0] / 32768.0 * 180
            parsed.update({'roll': roll, 'pitch': pitch, 'yaw': yaw, 'type': 'angle'})
            
        else:
            return None
            
        return parsed
    
    def read_frame(self):
        """
        读取一帧数据
        支持多种格式自动识别
        """
        if not self.ser or not self.ser.is_open:
            return None
            
        try:
            # 策略 1: 尝试读取 CSV 文本行
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and ',' in line:
                    return self.parse_csv_format(line)
            
            # 策略 2: 尝试读取 WitMotion 二进制协议
            if self.ser.in_waiting >= 11:
                # 查找包头 0x55
                header = self.ser.read(1)
                if header == b'\x55':
                    data = bytearray(header)
                    data.extend(self.ser.read(10))
                    # 校验和检查 (简单版)
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
        
        # 根据数据类型打印
        if 'type' in data:
            if data['type'] == 'accel':
                print(f"[{timestamp}] 加速度(g):  X:{data['ax']:8.3f}  Y:{data['ay']:8.3f}  Z:{data['az']:8.3f}")
            elif data['type'] == 'gyro':
                print(f"[{timestamp}] 角速度(dps): X:{data['gx']:8.2f}  Y:{data['gy']:8.2f}  Z:{data['gz']:8.2f}")
            elif data['type'] == 'angle':
                print(f"[{timestamp}] 角度(°):    R:{data['roll']:8.2f}  P:{data['pitch']:8.2f}  Y:{data['yaw']:8.2f}")
        else:
            # CSV 格式，同时显示 6DoF
            print(f"[{timestamp}] "
                  f"Acc(g):[{data['ax']:7.3f},{data['ay']:7.3f},{data['az']:7.3f}]  "
                  f"Gyro(dps):[{data['gx']:7.2f},{data['gy']:7.2f},{data['gz']:7.2f}]")
    
    def start(self):
        """开始读取循环"""
        if not self.ser:
            if not self.auto_connect():
                return False
        
        print("\n" + "="*60)
        print("IMU 数据读取开始...")
        print("按 Ctrl+C 停止")
        print("="*60 + "\n")
        
        self.running = True
        self.start_time = time.time()
        
        try:
            while self.running:
                data = self.read_frame()
                
                if data:
                    self.frame_count += 1
                    self.data_buffer.append(data)
                    self.print_data(data)
                    
                    # 这里可以插入你的跑步姿态检测算法
                    # self.detect_running_phase(data)
                    
                else:
                    # 没有数据时短暂休眠，避免 CPU 占用过高
                    time.sleep(0.001)
                    
        except KeyboardInterrupt:
            print("\n\n用户停止")
        finally:
            self.stop()
            
        return True
    
    def stop(self):
        """停止并清理"""
        self.running = False
        if self.ser:
            self.ser.close()
        
        # 打印统计信息
        duration = time.time() - self.start_time if self.start_time else 0
        fps = self.frame_count / duration if duration > 0 else 0
        
        print("\n" + "="*60)
        print("读取统计：")
        print(f"  总帧数: {self.frame_count}")
        print(f"  运行时间: {duration:.2f} 秒")
        print(f"  平均帧率: {fps:.1f} fps")
        print("="*60)
    
    def detect_running_phase(self, data):
        """
        【预留接口】跑步姿态检测算法
        目标：根据 6DoF 数据判断左脚在前还是右脚在前
        
        思路提示：
        1. 观察 Z 轴加速度的周期性变化（步态周期）
        2. 结合 X/Y 轴角速度判断躯干扭转方向
        3. 利用加速度峰值检测触地时刻
        4. 通过机器学习或阈值法分类左右脚
        
        示例伪代码：
        if peak_detected_in_az:
            if gyro_y > threshold:
                return "LEFT_FOOT_FORWARD"
            else:
                return "RIGHT_FOOT_FORWARD"
        """
        pass


def main():
    """主函数"""
    print("IMU 6DoF 实时数据读取器")
    print("适用于跑步姿态检测项目\n")
    
    # 创建读取器实例
    # 常见波特率：115200 (Arduino), 921600 (高速 IMU)
    # reader = IMUReader(baudrate=115200)
    
    # 手动指定串口号（如果自动检测失败）
    reader = IMUReader(port='COM14', baudrate=115200)  # Windows
    # reader = IMUReader(port='/dev/ttyUSB0', baudrate=115200)  # Linux
    # reader = IMUReader(port='/dev/cu.usbserial-XXXX', baudrate=115200)  # Mac
    
    # 开始读取
    reader.start()


if __name__ == "__main__":
    main()