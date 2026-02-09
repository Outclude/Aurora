"""
串口查找工具
列出所有可用的串口
"""

import serial
import serial.tools.list_ports

def list_serial_ports():
    """列出所有可用的串口"""
    print("=" * 60)
    print("可用串口列表：")
    print("=" * 60)

    ports = serial.tools.list_ports.comports()

    if not ports:
        print("\n未找到任何串口！")
        print("\n可能的原因：")
        print("1. 设备未连接")
        print("2. 驱动程序未安装")
        print("3. 设备被其他程序占用")
        return []

    print(f"\n找到 {len(ports)} 个串口：\n")

    for i, port in enumerate(ports, 1):
        print(f"{i}. 端口: {port.device}")
        print(f"   描述: {port.description}")
        print(f"   硬件ID: {port.hwid}")

        # 检查是否正在使用
        try:
            ser = serial.Serial(port.device, timeout=1)
            ser.close()
            status = "可用"
        except serial.SerialException:
            status = "占用或不可用"

        print(f"   状态: {status}")
        print()

    print("=" * 60)

    # 返回可用串口列表
    available_ports = [port.device for port in ports]
    return available_ports

def test_serial_connection(port_name, baudrate=115200):
    """测试串口连接"""
    print(f"\n测试连接 {port_name} @ {baudrate} baud...")

    try:
        ser = serial.Serial(port_name, baudrate, timeout=2)
        print(f"✓ 成功连接到 {port_name}")

        # 尝试读取数据
        print("正在读取数据（2秒）...")
        start_time = time.time()
        data_count = 0

        while time.time() - start_time < 2:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"  接收到: {data[:80]}")
                    data_count += 1
            time.sleep(0.1)

        if data_count == 0:
            print("  (未接收到数据)")
        else:
            print(f"  共接收到 {data_count} 条数据")

        ser.close()
        return True

    except serial.SerialException as e:
        print(f"✗ 连接失败: {e}")
        return False

def auto_select_port():
    """自动选择可能的串口"""
    ports = serial.tools.list_ports.comports()

    # 优先选择包含以下关键词的串口
    keywords = ['USB', 'UART', 'CH340', 'CP210', 'FTDI', 'Arduino', 'ESP32']

    for port in ports:
        for keyword in keywords:
            if keyword.upper() in port.description.upper() or \
               keyword.upper() in port.hwid.upper():
                return port.device

    # 如果没有找到，返回第一个串口
    if ports:
        return ports[0].device

    return None


if __name__ == '__main__':
    import time

    print("\n" + "=" * 60)
    print(" IMU 串口查找工具")
    print("=" * 60)

    # 列出所有串口
    available_ports = list_serial_ports()

    if available_ports:
        print(f"\n建议：修改 imu_monitor.py 第 8 行为：")
        print(f"PORT = '{available_ports[0]}'")

        # 自动选择可能的串口
        auto_port = auto_select_port()
        if auto_port:
            print(f"\n自动检测到的可能串口: {auto_port}")

            # 询问是否测试
            try:
                choice = input("\n是否测试连接？(y/n): ").strip().lower()
                if choice == 'y':
                    test_serial_connection(auto_port)
            except KeyboardInterrupt:
                print("\n")
    else:
        print("\n未找到可用串口，请检查：")
        print("1. 设备是否已连接（USB线）")
        print("2. 驱动是否已安装（如 CH340, CP210x）")
        print("3. 设备是否被其他程序占用（如 Arduino IDE）")

        print("\n在 Windows 设备管理器中查看：")
        print("  Win + X -> 设备管理器 -> 端口(COM和LPT)")
