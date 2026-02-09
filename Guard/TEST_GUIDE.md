# IMU + GPS 数据融合测试指南

## 📋 概述

本测试系统用于验证 IMU 滤波和 GPS 融合算法的正确性。包含完整的 Python 实现和 Arduino 测试程序。

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `imu_gps_test.py` | Python 测试主程序，实现所有 C++ 函数 |
| `test_imu_gps_serial.ino` | Arduino 测试程序，发送 IMU+GPS 数据 |

## 🚀 快速开始

### 步骤1: 上传 Arduino 程序

1. 打开 Arduino IDE
2. 打开 `test_imu_gps_serial.ino`
3. 选择开发板和端口
4. 上传程序

**说明**：
- 默认使用**模拟数据**（无需真实传感器）
- 如需使用真实传感器，修改 `#define USE_REAL_IMU` 等宏定义

### 步骤2: 运行 Python 测试程序

```bash
# 查找可用串口（可选）
python list_ports.py

# 运行测试程序
python imu_gps_test.py
```

### 步骤3: 观察输出

程序会实时打印：
1. **IMU 原始数据** - 串口接收的原始加速度和角速度
2. **GPS 原始数据** - 纬度、经度、卫星数量
3. **IMU 滤波后数据** - 经过 Kalman/低通/移动平均滤波
4. **运动融合状态** - 速度、距离、步频等

## 📊 数据格式

### 串口数据格式

**IMU 数据**：
```
IMU:ax=-9.1970 ay=0.6150 az=0.4210 gx=0.1462 gy=0.3218 gz=-0.1949
```

**GPS 数据**：
```
GPS:lat=39.904200 lng=116.407400 sat=8
```

**JSON 格式**（可选）：
```json
{"type":"imu", "ax":-9.197, "ay":0.615, ...}
{"type":"gps", "lat":39.9042, "lng":116.4074, ...}
```

## ⚙️ 配置选项

### Python 程序配置

编辑 `imu_gps_test.py`：

```python
# 串口配置
PORT = 'auto'         # 'auto' = 自动检测，或指定 'COM14'
BAUD = 115200

# 打印间隔
PRINT_INTERVAL = 0.1  # 秒

# 滤波器类型
FILTER_TYPE = 'kalman'  # 'none', 'kalman', 'lowpass', 'moving_avg'
```

### Arduino 程序配置

编辑 `test_imu_gps_serial.ino`：

```cpp
#define SERIAL_BAUD 115200
#define SEND_INTERVAL_MS 50  // 发送间隔（毫秒）

// 启用真实传感器
#define USE_REAL_IMU
#define USE_REAL_GPS
```

## 🧪 测试场景

### 测试1: 滤波效果对比

运行程序并修改滤波器类型，观察滤波效果：

```python
# 无滤波
FILTER_TYPE = 'none'

# Kalman 滤波
FILTER_TYPE = 'kalman'

# 低通滤波
FILTER_TYPE = 'lowpass'

# 移动平均
FILTER_TYPE = 'moving_avg'
```

**观察要点**：
- 原始数据的噪声水平
- 滤波后数据的平滑度
- 不同滤波器的响应速度

### 测试2: 步态检测

观察 `【4. 运动融合状态】` 中的步频：
- 正常行走：约 100-120 步/分钟
- 慢跑：约 140-160 步/分钟
- 快跑：约 160-180 步/分钟

### 测试3: 速度融合

对比 IMU 速度和 GPS 速度：
- 低速时（< 1 m/s）：GPS 权重更高
- 高速时（> 5 m/s）：IMU 权重更高
- 静止时：速度应接近 0

### 测试4: 距离累积

观察总距离的变化：
- 应该逐渐增加
- 不会出现负值
- 数值合理（步行约 1.2 m/s，跑步约 2.5-3.5 m/s）

## 🔍 故障排除

### 问题1: 找不到串口

**解决**：
```bash
# 查看所有可用串口
python list_ports.py
```

### 问题2: 接收不到数据

**检查**：
1. Arduino 是否正确上传程序
2. 串口号是否正确
3. 波特率是否匹配（默认 115200）
4. Arduino 串口监视器是否能看到数据

### 问题3: 数据解析失败

**检查数据格式**：
- 必须以 `IMU:` 或 `GPS:` 开头
- 数据之间用空格分隔
- 键值对用 `=` 连接

正确示例：
```
IMU:ax=1.0 ay=2.0 az=3.0 gx=0.1 gy=0.2 gz=0.3
GPS:lat=39.9 lng=116.4 sat=8
```

## 📈 与 C++ 版本对比

Python 实现与 C++ 版本完全对应：

| C++ 类/函数 | Python 类/函数 |
|------------|---------------|
| `IMUFilter::filter()` | `IMUFilter.filter()` |
| `GPSProcessor::calculateDistance()` | `GPSProcessor.calculate_distance()` |
| `MotionSensorFusion::update()` | `MotionSensorFusion.update()` |
| `IMURawData` | 字典 `{'ax': x, 'ay': y, ...}` |
| `IMUFilteredData` | 字典 `{'ax': x, ..., 'acc_magnitude': m}` |
| `MotionState` | 字典 `{'speed_fused': v, ...}` |

## 🎯 验证要点

确保 Python 实现与 C++ 实现结果一致：

1. ✅ **滤波器输出**：相同输入应产生相同输出
2. ✅ **距离计算**：Haversine 公式计算 GPS 距离
3. ✅ **速度融合**：动态权重调整逻辑一致
4. ✅ **步态检测**：峰值检测算法一致
5. ✅ **边界处理**：静止、低速、高速场景

## 📝 输出示例

```
================================================================================
[123.456] 数据处理结果
================================================================================

【1. IMU 原始数据】
  加速度: ax=-9.1970 g, ay=0.6150 g, az=0.4210 g
  角速度: gx=0.146 deg/s, gy=0.322 deg/s, gz=-0.195 deg/s

【2. GPS 原始数据】
  位置: lat=39.904200, lng=116.407400
  卫星: 8 颗 | 有效: 是

【3. IMU 滤波后数据】
  加速度: ax=-9.1850 g, ay=0.6100 g, az=0.4180 g
  角速度: gx=0.140 deg/s, gy=0.318 deg/s, gz=-0.190 deg/s
  合加速度: 9.2500 g | 水平加速度: 0.6500 g

【4. 运动融合状态】
  速度: IMU=1.250 m/s, GPS=1.180 m/s, 融合=1.200 m/s
  距离增量: 0.0600 m | 总距离: 125.50 m
  步频: 120.0 步/分 | 检测步态: 是
  权重: IMU=0.30, GPS=0.70
  位置有效: 是
================================================================================
```

## 🔗 相关文件

- `imu_filter.h/cpp` - C++ 滤波器实现
- `gps_processor.h/cpp` - C++ GPS 处理
- `motion_sensor_fusion.h/cpp` - C++ 运动融合
- `imu_monitor.py` - 可视化监视器

## 📞 问题反馈

如发现问题，请检查：
1. Python 和 C++ 算法是否完全一致
2. 数据类型精度（float vs double）
3. 边界条件处理
4. 时间戳计算方式
