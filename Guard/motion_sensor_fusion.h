#ifndef __MOTION_SENSOR_FUSION_H__
#define __MOTION_SENSOR_FUSION_H__

#include <Arduino.h>
#include "imu_filter.h"
#include "gps_processor.h"

/**
 * 运动状态结构体
 */
struct MotionState {
    unsigned long timestamp;     // 时间戳(ms)

    // 速度相关
    float speed_imu;             // 从IMU计算的速度 (m/s)
    float speed_gps;             // 从GPS计算的速度 (m/s)
    float speed_fused;           // 融合后的速度 (m/s)

    // 距离相关
    float distance_increment;    // 本次更新的距离增量 (m)

    // 步频相关
    float cadence;               // 实时步频 (步/分钟)
    bool step_detected;          // 是否检测到新的一步

    // 位置相关
    double latitude;             // 当前纬度
    double longitude;            // 当前经度
    bool position_valid;         // 位置是否有效

    // 传感器权重（用于调试）
    float imu_weight;            // IMU权重 [0-1]
    float gps_weight;            // GPS权重 [0-1]
};

/**
 * 步态检测器状态
 */
struct StepDetectionState {
    unsigned long lastStepTime;  // 上次检测到步态的时间
    int stepCount;               // 总步数
    float lastAccMagnitude;      // 上次合加速度
    bool rising;                 // 是否在上升沿
    float peakThreshold;         // 峰值检测阈值
    unsigned long minStepInterval; // 最小步态间隔(ms)
};

/**
 * 运动传感器融合类
 * 功能：
 * 1. 融合IMU和GPS数据计算实时速度
 * 2. 检测步态并计算步频
 * 3. 计算距离增量
 */
class MotionSensorFusion {
public:
    MotionSensorFusion();
    ~MotionSensorFusion() = default;

    /**
     * 更新融合状态（主要函数）
     * @param imuFiltered 滤波后的IMU数据
     * @param gps GPS原始数据
     * @return 运动状态
     */
    MotionState update(const IMUFilteredData& imuFiltered, const GPSRawData& gps);

    /**
     * 重置所有状态
     */
    void reset();

    /**
     * 获取当前总距离 (米)
     */
    float getTotalDistance() const { return totalDistance_; }

    /**
     * 获取当前步频 (步/分钟)
     */
    float getCadence() const { return currentCadence_; }

    /**
     * 获取当前速度 (m/s)
     */
    float getSpeed() const { return currentSpeed_; }

    /**
     * 设置静止阈值 (用于判断是否在运动)
     */
    void setStationaryThreshold(float threshold) {
        stationaryThreshold_ = threshold;
    }

    /**
     * 设置步态峰值阈值
     */
    void setStepPeakThreshold(float threshold) {
        stepState_.peakThreshold = threshold;
    }

    /**
     * 设置融合权重
     * GPS速度权重：静止时使用GPS速度，运动时融合IMU和GPS
     */
    void setFusionWeights(float imuWeight, float gpsWeight);

private:
    // 累积数据
    float totalDistance_;        // 总距离 (米)
    float currentSpeed_;         // 当前速度 (m/s)
    float currentCadence_;       // 当前步频 (步/分钟)

    // 上次状态
    GPSPosition lastGPSPos_;
    IMUFilteredData lastIMUData_;
    unsigned long lastUpdateTime_;

    // 步态检测
    StepDetectionState stepState_;

    // 参数
    float stationaryThreshold_;   // 静止阈值 (m/s)，低于此值认为静止
    float imuWeight_;            // IMU权重
    float gpsWeight_;            // GPS权重

    // 内部函数

    /**
     * 从IMU加速度积分计算速度
     * @param imuFiltered 滤波后的IMU数据
     * @param dt 时间间隔 (秒)
     * @return 速度增量 (m/s)
     */
    float calculateIMUSpeed(const IMUFilteredData& imuFiltered, float dt);

    /**
     * 检测步态
     * @param imuFiltered 滤波后的IMU数据
     * @return 步频 (步/分钟), 0表示未检测到新步态
     */
    float detectStepCadence(const IMUFilteredData& imuFiltered);

    /**
     * 融合IMU和GPS速度
     * @param speedIMU IMU计算的速度
     * @param speedGPS GPS计算的速度
     * @param gpsValid GPS数据是否有效
     * @return 融合后的速度
     */
    float fuseSpeed(float speedIMU, float speedGPS, bool gpsValid);

    /**
     * 计算距离增量
     * @param speed 融合后的速度
     * @param dt 时间间隔 (秒)
     * @return 距离增量 (米)
     */
    float calculateDistanceIncrement(float speed, float dt);

    /**
     * 判断是否在静止状态
     */
    bool isStationary(float speed) const;
};

#endif // __MOTION_SENSOR_FUSION_H__
