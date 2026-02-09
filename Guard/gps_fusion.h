#ifndef __GPS_FUSION_H__
#define __GPS_FUSION_H__

#include <Arduino.h>

// GPS原始数据结构
struct GPSData {
    double latitude;      // 纬度（度）
    double longitude;     // 经度（度）
    uint8_t satellites;   // 卫星数量
    bool valid;           // 数据是否有效

    GPSData() : latitude(0.0), longitude(0.0), satellites(0), valid(false) {}
};

// 融合结果结构体
struct FusionResult {
    double total_distance;   // 总路程（米）
    double current_speed;    // 融合后的瞬时速度（m/s）
    double imu_speed;        // IMU积分速度（m/s）
    double gps_speed;        // GPS计算速度（m/s），如果无GPS则为0

    FusionResult() : total_distance(0.0), current_speed(0.0), imu_speed(0.0), gps_speed(0.0) {}
};

/**
 * IMU与GPS融合速度和距离计算器
 * 使用互补滤波融合IMU加速度积分和GPS位置差分
 *
 * 融合原理：
 * - IMU加速度积分：高频响应快，但有漂移
 * - GPS位置差分：低频稳定，但有延迟
 * - 互补滤波：融合两者优点，得到准确且响应快的速度估计
 */
class GPSFusion {
public:
    /**
     * 构造函数
     * @param alpha 互补滤波系数 (0-1)，接近1表示更信任IMU，接近0表示更信任GPS
     * @param min_gps_sats 最小有效卫星数
     */
    explicit GPSFusion(float alpha = 0.95f, uint8_t min_gps_sats = 4);

    /**
     * 重置所有状态
     */
    void reset();

    /**
     * 更新融合算法（主入口）
     * @param imu_forward_acc 前进方向加速度（单位：g，已滤波）
     * @param gps GPS原始数据
     * @param current_time 当前时间（秒），可选
     * @return 融合结果
     */
    FusionResult update(float imu_forward_acc, const GPSData& gps, float current_time = 0.0f);

    /**
     * 获取当前总路程
     */
    double getTotalDistance() const { return total_distance_; }

    /**
     * 获取当前融合速度
     */
    double getCurrentSpeed() const { return fused_velocity_; }

    /**
     * 计算两点间GPS球面距离（Haversine公式）
     * 静态方法，可独立使用
     */
    static double computeGPSDistance(double lat1, double lon1, double lat2, double lon2);

private:
    // 常量
    static constexpr double EARTH_RADIUS = 6371000.0;  // 地球半径（米）
    static constexpr double GRAVITY = 9.81;            // 重力加速度（m/s²）
    static constexpr float DEFAULT_DT = 0.01f;         // 默认时间步长（秒）

    // 参数
    float alpha_;                 // 互补滤波系数
    uint8_t min_gps_sats_;        // 最小有效卫星数

    // 状态变量
    double imu_velocity_;         // IMU积分速度（m/s）
    double fused_velocity_;       // 融合速度（m/s）
    double total_distance_;       // 累积总路程（米）

    // GPS状态
    double last_gps_lat_;         // 上一次GPS纬度
    double last_gps_lon_;         // 上一次GPS经度
    float last_gps_time_;         // 上一次GPS时间
    bool gps_initialized_;        // GPS是否已初始化

    // IMU状态
    float last_imu_time_;         // 上一次IMU时间
    bool imu_initialized_;        // IMU是否已初始化

    // 内部函数
    double computeGPSVelocity(const GPSData& gps, float current_time);
    double computeIMUVelocity(float forward_acc, float current_time);
    double degToRad(double degrees) const;
};

#endif // __GPS_FUSION_H__
