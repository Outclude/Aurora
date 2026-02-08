#ifndef __GPS_PROCESSOR_H__
#define __GPS_PROCESSOR_H__

#include <Arduino.h>

/**
 * GPS原始数据结构
 */
struct GPSRawData {
    double latitude;   // 纬度 (度)
    double longitude;  // 经度 (度)
    int satellites;    // 卫星数量
    unsigned long timestamp;  // 时间戳(ms)
    bool valid;        // 数据是否有效
};

/**
 * GPS位置点（用于轨迹记录）
 */
struct GPSPosition {
    double latitude;   // 纬度 (度)
    double longitude;  // 经度 (度)
    unsigned long timestamp;  // 时间戳(ms)
    float speed;       // 速度 (m/s, 从GPS获取)
    float altitude;    // 海拔 (m)
    bool valid;        // 数据是否有效
};

/**
 * GPS处理器类
 * 功能：验证GPS数据有效性、计算两点间距离、坐标转换等
 */
class GPSProcessor {
public:
    GPSProcessor();
    ~GPSProcessor() = default;

    /**
     * 验证GPS数据是否有效
     * @param gps GPS原始数据
     * @return true=有效, false=无效
     */
    bool isValid(const GPSRawData& gps) const;

    /**
     * 计算两个GPS坐标点之间的距离 (Haversine公式)
     * @param lat1, lon1 第一个点的纬度和经度 (度)
     * @param lat2, lon2 第二个点的纬度和经度 (度)
     * @return 距离 (米)
     */
    float calculateDistance(double lat1, double lon1, double lat2, double lon2) const;

    /**
     * 计算两个GPS坐标点之间的距离 (重载版本)
     */
    float calculateDistance(const GPSPosition& pos1, const GPSPosition& pos2) const;

    /**
     * 计算从点1到点2的速度
     * @param pos1, pos2 两个位置点
     * @return 速度 (m/s)
     */
    float calculateSpeed(const GPSPosition& pos1, const GPSPosition& pos2) const;

    /**
     * 计算从点1到点2的方位角
     * @param lat1, lon1 起点坐标
     * @param lat2, lon2 终点坐标
     * @return 方位角 (度, 0-360, 0=正北, 90=正东)
     */
    float calculateBearing(double lat1, double lon1, double lat2, double lon2) const;

    /**
     * 将GPS原始数据转换为位置点
     */
    GPSPosition convertToPosition(const GPSRawData& gps) const;

    /**
     * 设置最小卫星数量阈值
     */
    void setMinSatellites(int minSat) { minSatellites_ = minSat; }

    /**
     * 设置最小HDOP阈值
     */
    void setMinHDOP(float maxHDOP) { maxHDOP_ = maxHDOP; }

private:
    int minSatellites_;      // 最小卫星数量要求
    float maxHDOP_;          // 最大水平精度因子要求

    /**
     * 将角度转换为弧度
     */
    float degToRad(float deg) const;

    /**
     * 将弧度转换为角度
     */
    float radToDeg(float rad) const;
};

#endif // __GPS_PROCESSOR_H__
