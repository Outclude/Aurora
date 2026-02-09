#ifndef __IMU_GPS_INTEGRATION_H__
#define __IMU_GPS_INTEGRATION_H__

#include <Arduino.h>
#include "imu_simple.h"
#include "gps_fusion.h"

/**
 * IMU与GPS集成管理器
 * 类似于imu_gps_simp.py的功能，整合IMU滤波和GPS融合
 */
class IMUGPSIntegration {
public:
    /**
     * 构造函数
     * @param forward_acc_key 前进方向加速度轴 ('x', 'y', 或 'z')
     * @param fusion_alpha 融合滤波系数
     * @param min_gps_sats 最小GPS卫星数
     */
    explicit IMUGPSIntegration(char forward_acc_key = 'y',
                               float fusion_alpha = 0.95f,
                               uint8_t min_gps_sats = 4);

    /**
     * 处理IMU原始数据
     * @param raw IMU原始数据
     * @return 滤波后的IMU数据
     */
    IMUFilteredData processIMU(const IMURawData& raw);

    /**
     * 处理GPS数据并更新融合算法
     * @param imu_forward_acc 滤波后的前进方向加速度 (g)
     * @param gps GPS原始数据
     * @param current_time 当前时间（秒）
     * @return 融合结果
     */
    FusionResult processGPS(float imu_forward_acc, const GPSData& gps, float current_time = 0.0f);

    /**
     * 完整处理流程（一步到位）
     * @param imu_raw IMU原始数据
     * @param gps GPS原始数据
     * @param current_time 当前时间（秒）
     * @return 融合结果
     */
    FusionResult update(const IMURawData& imu_raw, const GPSData& gps, float current_time = 0.0f);

    /**
     * 重置所有状态
     */
    void reset();

    /**
     * 获取融合结果
     */
    FusionResult getResult() const;

    /**
     * 获取滤波后的IMU数据
     */
    IMUFilteredData getFilteredIMU() const;

    /**
     * 设置前进方向轴
     */
    void setForwardAxis(char axis);

    /**
     * 打印调试信息
     */
    void printStatus() const;

private:
    IMUSimpleFilter imu_filter_;     // IMU滤波器
    GPSFusion gps_fusion_;           // GPS融合器

    char forward_acc_key_;           // 前进方向轴 ('x', 'y', 'z')
    FusionResult last_result_;       // 上次融合结果
    IMUFilteredData last_imu_;       // 上次滤波IMU数据

    // 从滤波后的IMU数据提取前进方向加速度
    float getForwardAcc(const IMUFilteredData& filtered) const;
};

#endif // __IMU_GPS_INTEGRATION_H__
