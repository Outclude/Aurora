import math
import serial
import time


EARTH_RADIUS = 6371000.0  # 地球半径，单位：米

# current_gps_data = {
#         'latitude': 0.0,
#         'longitude': 0.0,
#         'satellites': 0
#     }

latitude = 0.0
longtitude = 0.0
satellites = 0

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
    # imu_filter = IMUFilter(filter_type='kalman')
    # direction_detector = DirectionDetector()
    # gps_processor = GPSProcessor()
    # fusion_estimator = FusionSpeedEstimator()

    # print("开始实时测试，按 Ctrl+C 退出...")
    # print("等待方向检测和 GPS 信号...\n")

    # print("-" * 120)
    # print("状态初始化...")
    # print("-" * 120)

    t0 = time.time()
    last_print_time = 0.0

    # 状态标志
    # direction_detected = False
    # gps_locked = False

    # 当前 GPS 数据缓存（用于显示）
    # current_gps_data = {
    #     'latitude': 0.0,
    #     'longitude': 0.0,
    #     'satellites': 0
    # }

    try:
        print('1')
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            # print(f'line: {line}')
            now = time.time() - t0

            # 解析 IMU 数据
            # imu_raw = parse_imu_line(line)
            # if imu_raw is not None:
            #     # 滤波
            #     imu_filtered = imu_filter.filter(imu_raw)

            #     # 方向检测
            #     direction_detector.add_data(imu_filtered)
            #     if len(direction_detector.buffers['ax']) >= ANALYSIS_WINDOW_SIZE:
            #         direction_detector.detect_directions()

            #     # 检查方向是否已校准
            #     if not direction_detected and direction_detector.is_calibrated:
            #         direction_detected = True
            #         print(f"\n✓ 方向检测完成！")
            #         if gps_locked:
            #             print("\n开始融合速度计算...")
            #             print_header()

            #     # 如果方向已校准，进行速度估计
            #     if direction_detector.is_calibrated:
            #         current_time = time.time() * 1000

            #         # 获取前进方向加速度
            #         forward_accel = direction_detector.get_forward_acceleration(imu_filtered)

            #         # 使用 IMU 进行预测
            #         fusion_estimator.predict_from_imu(forward_accel, current_time)

            #         # 打印（每隔 DISPLAY_INTERVAL）
            #         if direction_detected and gps_locked:
            #             if now - last_print_time >= DISPLAY_INTERVAL:
            #                 lateral_accel = direction_detector.get_lateral_acceleration(imu_filtered)
            #                 fusion_speed = fusion_estimator.get_velocity()

            #                 print_status_line(
            #                     now,
            #                     forward_accel,
            #                     lateral_accel,
            #                     current_gps_data['satellites'],
            #                     current_gps_data['longitude'],
            #                     current_gps_data['latitude'],
            #                     fusion_speed
            #                 )
            #                 last_print_time = now

            #     continue  # 处理完 IMU 数据，继续下一行

            # 解析 GPS 数据
            # print(f'\n\n现在开始解析GPS数据\n\n')
            gps_data = parse_gps_line(line)
            print(f'gps_data:{gps_data}')
            if gps_data is not None:
                # 打印 GPS 原始数据
                # print(f"[GPS RAW] {line}")

                # 更新 GPS 数据缓存
                current_gps_data = gps_data

                # # 处理 GPS 数据
                latitude = current_gps_data['latitude']
                longitude = current_gps_data['longitude']
                satellites =  current_gps_data['satellites']
                print(f'\n\nlat: {latitude}\n\n')
                # gps_velocity, gps_valid = gps_processor.update(
                #     gps_data['latitude'],
                #     gps_data['longitude'],
                #     gps_data['satellites']
                # )

                # # 检查 GPS 是否锁定
                # if not gps_locked and gps_valid:
                #     gps_locked = True
                #     print(f"\n✓ GPS 信号已锁定 (卫星数: {gps_data['satellites']})")
                #     if direction_detected:
                #         print("\n开始融合速度计算...")
                #         print_header()

                # # 如果 GPS 有效，进行更新
                # if gps_valid:
                #     fusion_estimator.update_with_gps(gps_velocity, gps_valid)
                
                
    except:
        return 0
    
    
    
main()