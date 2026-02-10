/**
 * IMU与GPS集成使用示例
 *
 * 此文件展示如何在Guard项目中使用IMU-GPS融合功能
 *
 * 功能说明：
 * 1. 从传感器读取IMU原始数据（加速度、角速度）
 * 2. 从GPS模块读取位置数据（经纬度、卫星数）
 * 3. 使用IMUSimpleFilter对IMU数据进行卡尔曼滤波
 * 4. 使用GPSFusion融合IMU和GPS数据计算速度和距离
 * 5. 自动通过data.h的setDistance()和setCurrentSpeed()更新SystemData
 *
 * 使用方法：
 * 1. 在Guard.ino中包含此文件：#include "imu_gps_integration.h"
 * 2. 创建全局实例：IMUGPSIntegration imu_gps;
 * 3. 在setup()中初始化传感器
 * 4. 在loop()中调用imu_gps.update(imu_raw, gps_data, time)
 * 5. 融合结果会自动写入SystemData，可通过SystemData::getInstance()访问
 */

#ifndef __IMU_GPS_EXAMPLE_H__
#define __IMU_GPS_EXAMPLE_H__

#include <Arduino.h>
#include "imu_gps_integration.h"
#include "data.h"

// ==================== 全局变量 ====================
extern IMUGPSIntegration imu_gps;  // 在Guard.ino中定义

// ==================== 使用示例 ====================

/*
// 在Guard.ino中的实现示例：

#include "imu_gps_integration.h"

// 创建集成实例（前进方向为Y轴，融合系数0.95，最小GPS卫星数4）
IMUGPSIntegration imu_gps('y', 0.95f, 4);

unsigned long startTime = 0;

void setup() {
    Serial.begin(115200);
    startTime = millis();

    // 初始化IMU传感器（根据实际硬件配置）
    // initIMU();

    // 初始化GPS模块（根据实际硬件配置）
    // initGPS();

    Serial.println("IMU-GPS Integration System Started");
}

void loop() {
    // 1. 读取IMU原始数据
    IMURawData imu_raw;
    // TODO: 从实际传感器读取
    // imu_raw.ax = readAccX();
    // imu_raw.ay = readAccY();
    // imu_raw.az = readAccZ();
    // imu_raw.gx = readGyroX();
    // imu_raw.gy = readGyroY();
    // imu_raw.gz = readGyroZ();

    // 2. 读取GPS数据
    GPSData gps_data;
    // TODO: 从GPS模块读取
    // gps_data.latitude = gps.location.lat();
    // gps_data.longitude = gps.location.lng();
    // gps_data.satellites = gps.satellites.value();
    // gps_data.valid = (gps_data.satellites >= 4);

    // 3. 计算当前时间（秒）
    float current_time = (millis() - startTime) / 1000.0f;

    // 4. 更新融合算法（一步到位）
    FusionResult result = imu_gps.update(imu_raw, gps_data, current_time);

    // 5. 结果已自动写入SystemData，可直接使用
    // SystemData::getInstance().getDistance()    // 总路程
    // SystemData::getInstance().getCurrentSpeed() // 当前速度

    // 6. 调试输出（可选）
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {  // 每500ms打印一次
        imu_gps.printStatus();
        lastPrint = millis();
    }

    delay(10);  // 适当延时
}

*/

// ==================== 分步使用示例 ====================

/*
// 如果需要更精细的控制，可以分步处理：

void loop() {
    // 方式1：分步处理
    IMURawData imu_raw = readIMU();
    GPSData gps_data = readGPS();

    // Step 1: 只处理IMU滤波
    IMUFilteredData filtered = imu_gps.processIMU(imu_raw);

    // Step 2: 获取前进方向加速度
    float forward_acc = filtered.ay;  // 或 filtered.ax / filtered.az

    // Step 3: GPS融合
    float current_time = millis() / 1000.0f;
    FusionResult result = imu_gps.processGPS(forward_acc, gps_data, current_time);

    // 方式2：一步处理（推荐）
    FusionResult result = imu_gps.update(imu_raw, gps_data, current_time);
}

*/

// ==================== 接口说明 ====================

/**
 * GPSData 结构体：
 * - double latitude:      纬度（度）
 * - double longitude:     经度（度）
 * - uint8_t satellites:   卫星数量
 * - bool valid:           数据是否有效
 *
 * IMURawData 结构体：
 * - float ax, ay, az:    三轴加速度 (g)
 * - float gx, gy, gz:    三轴角速度 (deg/s)
 *
 * FusionResult 结构体：
 * - double total_distance:   总路程（米）
 * - double current_speed:    融合后的瞬时速度（m/s）
 * - double imu_speed:        IMU积分速度（m/s）
 * - double gps_speed:        GPS计算速度（m/s）
 */

// ==================== 参数调整建议 ====================

/*
融合系数 alpha 的选择：
- alpha = 0.90-0.98: 更信任IMU，响应快但可能有漂移（适合跑步等动态场景）
- alpha = 0.70-0.90: 平衡IMU和GPS（一般场景）
- alpha = 0.50-0.70: 更信任GPS，稳定但响应较慢（适合匀速运动）

前进方向轴设置：
- 'x': X轴为前进方向
- 'y': Y轴为前进方向（默认）
- 'z': Z轴为前进方向

最小GPS卫星数：
- 3-4: 城市环境，可能有遮挡
- 5-6: 开阔环境
- 7+: 理想环境
*/

#endif // __IMU_GPS_EXAMPLE_H__
