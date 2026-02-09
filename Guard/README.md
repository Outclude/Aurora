# IMU 实时监视器 - 使用说明

## 功能说明

这个 IMU 监视器可以实时显示 IMU 传感器的原始数据和滤波后数据，共 12 个坐标系：
- **6 个原始数据通道**（红色）：ax, ay, az, gx, gy, gz
- **6 个滤波后数据通道**（蓝色）：ax, ay, az, gx, gy, gz

## 支持的滤波器类型

1. **Kalman 滤波** (`kalman`) - 默认，适合大多数场景
2. **低通滤波** (`lowpass`) - 简单快速
3. **移动平均** (`moving_avg`) - 平滑效果明显
4. **无滤波** (`none`) - 直接显示原始数据

## 使用方法

### 方法 1：使用纯 Python 实现（推荐用于快速测试）

```bash
python imu_monitor.py
```

**修改滤波器类型：**
编辑 `imu_monitor.py` 第 18 行：
```python
FILTER_TYPE = 'kalman'  # 改为 'lowpass', 'moving_avg', 或 'none'
```

**修改串口：**
编辑 `imu_monitor.py` 第 8 行：
```python
PORT = 'COM14'  # Windows: COMx, Linux/Mac: /dev/ttyUSBx
```

### 方法 2：使用 C++ 实现的滤波器（性能更好）

#### 编译步骤：

```bash
# 1. 安装依赖
pip install pybind11

# 2. 编译 C++ 扩展
python setup.py build_ext --inplace

# 3. 使用 C++ 版本（自动检测）
python imu_monitor.py
```

#### 编译输出示例：

```
running build_ext
building 'imu_filter_cpp' extension
creating build
...
Successfully built imu_filter_cpp
```

### 串口数据格式

监视器期望的串口输出格式：
```
ax=0.0123 ay=-0.0345 az=0.9981 | gx=1.2345 gy=-0.4567 gz=0.0789
```

**Arduino 输出示例：**
```cpp
Serial.print("ax="); Serial.print(ax, 4);
Serial.print(" ay="); Serial.print(ay, 4);
Serial.print(" az="); Serial.print(az, 4);
Serial.print(" | gx="); Serial.print(gx, 4);
Serial.print(" gy="); Serial.print(gy, 4);
Serial.print(" gz="); Serial.println(gz, 4);
```

## 配置参数

在 `imu_monitor.py` 中可以修改以下参数：

```python
# 窗口持续时间（秒）
WINDOW_SEC = 10.0

# 加速度限值（g）
ACC_LIMIT = 20.0

# 角速度限值（deg/s）
GYRO_LIMIT = 2000.0

# 打印间隔（秒）
PRINT_INTERVAL = 0.1

# 滤波器类型
FILTER_TYPE = 'kalman'

# 低通滤波系数（仅当 FILTER_TYPE='lowpass' 时生效）
LOW_PASS_ALPHA = 0.3

# 移动平均窗口大小（仅当 FILTER_TYPE='moving_avg' 时生效）
MOVING_AVG_SIZE = 5
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `imu_monitor.py` | 主程序，纯 Python 实现 |
| `imu_filter.h` | C++ 滤波器头文件 |
| `imu_filter.cpp` | C++ 滤波器实现 |
| `imu_filter_pybind.cpp` | pybind11 绑定代码 |
| `setup.py` | 编译脚本 |
| `README.md` | 本文档 |

## 滤波效果对比

| 滤波器 | 优点 | 缺点 | 适用场景 |
|--------|------|------|----------|
| Kalman | 最优估计，参数可调 | 计算量大 | 一般情况 |
| LowPass | 简单快速 | 延迟较大 | 高频噪声 |
| MovingAvg | 平滑明显 | 响应慢 | 稳定数据 |
| None | 无延迟 | 噪声大 | 调试对比 |

## 依赖库

```bash
pip install pyserial matplotlib numpy
```

如果使用 C++ 滤波器，还需要：
```bash
pip install pybind11
```

## 常见问题

### Q1: 串口连接失败
```
✗ 串口连接失败: could not open port 'COM14'
```
**解决：** 检查串口名称是否正确，Windows 设备管理器查看

### Q2: 编译 C++ 扩展失败
```
error: Python.h: No such file or directory
```
**解决：** 安装 Python 开发头文件
- Windows: 使用预编译的 Python
- Linux: `sudo apt-get install python3-dev`
- Mac: `xcode-select --install`

### Q3: 图形窗口显示不正常
**解决：** 更新 matplotlib
```bash
pip install --upgrade matplotlib
```

## 快捷键

- **Ctrl+C**: 退出程序

## 性能建议

1. 使用 C++ 滤波器时性能更好（编译后）
2. 降低 `WINDOW_SEC` 可减少内存使用
3. 增加 `PRINT_INTERVAL` 可减少 CPU 占用

## 许可证

MIT License
