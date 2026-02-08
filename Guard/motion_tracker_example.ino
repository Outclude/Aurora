/**
 * 运动追踪示例代码
 * 演示如何使用 IMU + GPS 融合模块计算步频、速度和距离
 *
 * 功能说明：
 * 1. 读取 IMU 原始数据 (ax, ay, az, gx, gy, gz)
 * 2. 读取 GPS 原始数据 (gps_lat, gps_lng, gps_sat)
 * 3. IMU 数据滤波
 * 4. 融合 IMU 和 GPS 数据计算运动状态
 * 5. 记录运动轨迹
 *
 * 使用方法：
 * 1. 初始化各模块对象
 * 2. 在 loop() 中定期调用 update()
 * 3. 从 MotionState 获取实时运动数据
 * 4. 从 TrajectoryTracker 获取总距离和轨迹
 */

#include <Arduino.h>
#include "imu_filter.h"
#include "gps_processor.h"
#include "motion_sensor_fusion.h"
#include "trajectory_tracker.h"

// ==================== 全局变量声明 ====================

// IMU 原始数据 (全局 volatile float，与现有代码兼容)
volatile float ax = 0.0f;
volatile float ay = 0.0f;
volatile float az = 0.0f;
volatile float gx = 0.0f;
volatile float gy = 0.0f;
volatile float gz = 0.0f;

// GPS 原始数据
volatile double gps_lat = 0.0;
volatile double gps_lng = 0.0;
volatile int gps_sat = 0;

// ==================== 模块对象 ====================

// IMU 滤波器 (使用卡尔曼滤波)
IMUFilter imuFilter(FILTER_KALMAN);

// 运动传感器融合模块
MotionSensorFusion motionFusion;

// 轨迹追踪模块
TrajectoryTracker trajectoryTracker;

// ==================== 辅助函数 ====================

/**
 * 模拟读取 IMU 数据
 * TODO: 替换为实际的 IMU 读取代码
 */
bool readIMURawData(IMURawData& raw) {
    // 模拟数据 - 替换为实际读取逻辑
    raw.ax = ax;
    raw.ay = ay;
    raw.az = az;
    raw.gx = gx;
    raw.gy = gy;
    raw.gz = gz;
    return true;
}

/**
 * 模拟读取 GPS 数据
 * TODO: 替换为实际的 GPS 读取代码
 */
bool readGPSRawData(GPSRawData& raw) {
    raw.latitude = gps_lat;
    raw.longitude = gps_lng;
    raw.satellites = gps_sat;
    raw.timestamp = millis();

    // 简单验证数据有效性
    raw.valid = (raw.satellites >= 4) &&
                (raw.latitude != 0.0) &&
                (raw.longitude != 0.0);

    return true;
}

/**
 * 打印运动状态 (调试用)
 */
void printMotionState(const MotionState& state) {
    Serial.print("Speed: ");
    Serial.print(state.speed_fused, 2);
    Serial.print(" m/s (IMU: ");
    Serial.print(state.speed_imu, 2);
    Serial.print(", GPS: ");
    Serial.print(state.speed_gps, 2);
    Serial.print(") | Cadence: ");
    Serial.print(state.cadence, 1);
    Serial.print(" spm | Distance: ");
    Serial.print(motionFusion.getTotalDistance(), 1);
    Serial.print(" m");

    if (state.step_detected) {
        Serial.print(" [STEP]");
    }

    Serial.println();
}

/**
 * 打印轨迹统计信息
 */
void printTrajectoryStats() {
    TrajectoryStats stats = trajectoryTracker.getStats();

    Serial.println("\n=== Trajectory Statistics ===");
    Serial.print("Total Distance: ");
    Serial.print(stats.totalDistance, 2);
    Serial.println(" m");

    Serial.print("Total Time: ");
    Serial.print(stats.totalTime / 1000.0f, 1);
    Serial.println(" s");

    Serial.print("Average Speed: ");
    Serial.print(stats.averageSpeed, 2);
    Serial.println(" m/s");

    Serial.print("Max Speed: ");
    Serial.print(stats.maxSpeed, 2);
    Serial.println(" m/s");

    Serial.print("Total Steps: ");
    Serial.println(stats.totalSteps);

    Serial.print("Average Cadence: ");
    Serial.print(stats.averageCadence, 1);
    Serial.println(" spm");

    if (stats.startPoint.valid) {
        Serial.print("Start: (");
        Serial.print(stats.startPoint.latitude, 6);
        Serial.print(", ");
        Serial.print(stats.startPoint.longitude, 6);
        Serial.println(")");
    }

    if (stats.endPoint.valid) {
        Serial.print("End: (");
        Serial.print(stats.endPoint.latitude, 6);
        Serial.print(", ");
        Serial.print(stats.endPoint.longitude, 6);
        Serial.println(")");
    }

    Serial.print("Trajectory Points: ");
    Serial.println(trajectoryTracker.getTrajectoryPointCount());
    Serial.println("============================\n");
}

// ==================== 主程序 ====================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Motion Tracker Example ===");
    Serial.println("Initializing modules...");

    // 模块已通过构造函数初始化
    // 可以在这里设置额外参数：

    // 设置步态检测阈值
    motionFusion.setStepPeakThreshold(1.5f);  // 峰值阈值 (g)

    // 设置静止阈值
    motionFusion.setStationaryThreshold(0.3f);  // m/s

    // 设置融合权重
    motionFusion.setFusionWeights(0.3f, 0.7f);  // (IMU, GPS)

    // 设置轨迹记录间隔
    trajectoryTracker.setMinRecordDistance(5.0f);      // 5米
    trajectoryTracker.setMinRecordInterval(1000);      // 1秒

    Serial.println("Initialization complete!");
    Serial.println("Waiting for sensor data...\n");
}

void loop() {
    // 1. 读取 IMU 原始数据
    IMURawData imuRaw;
    if (!readIMURawData(imuRaw)) {
        delay(10);
        return;
    }

    // 2. 读取 GPS 原始数据
    GPSRawData gpsRaw;
    readGPSRawData(gpsRaw);  // GPS 无效也可以继续

    // 3. IMU 数据滤波
    IMUFilteredData imuFiltered = imuFilter.filter(imuRaw);

    // 4. 融合 IMU 和 GPS 数据，计算运动状态
    MotionState motionState = motionFusion.update(imuFiltered, gpsRaw);

    // 5. 记录轨迹点
    trajectoryTracker.addMotionState(motionState);

    // 6. 打印实时数据 (调试)
    printMotionState(motionState);

    // 7. 定期打印统计信息 (每10秒)
    static unsigned long lastStatsPrint = 0;
    if (millis() - lastStatsPrint > 10000) {
        printTrajectoryStats();
        lastStatsPrint = millis();
    }

    // 采样周期：10ms (100Hz)
    delay(10);
}

// ==================== API 使用示例 ====================

/**
 * 示例1: 获取实时速度
 */
float getRealTimeSpeed() {
    return motionFusion.getSpeed();  // m/s
}

/**
 * 示例2: 获取总距离
 */
float getTotalDistance() {
    // 方式1: 从融合模块获取
    return motionFusion.getTotalDistance();  // 米

    // 方式2: 从轨迹追踪器获取
    // return trajectoryTracker.getTotalDistance();
}

/**
 * 示例3: 获取实时步频
 */
float getRealTimeCadence() {
    return motionFusion.getCadence();  // 步/分钟
}

/**
 * 示例4: 重置所有模块 (开始新的运动)
 */
void resetMotionTracking() {
    motionFusion.reset();
    trajectoryTracker.reset();
    imuFilter.reset();
}
