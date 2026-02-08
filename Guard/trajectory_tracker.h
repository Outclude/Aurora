#ifndef __TRAJECTORY_TRACKER_H__
#define __TRAJECTORY_TRACKER_H__

#include <Arduino.h>
#include "gps_processor.h"
#include "motion_sensor_fusion.h"

/**
 * 轨迹点结构
 */
struct TrajectoryPoint {
    double latitude;       // 纬度
    double longitude;      // 经度
    unsigned long timestamp;  // 时间戳(ms)
    float distance_from_start;  // 距离起点的距离 (m)
    float speed;           // 速度 (m/s)
    float cadence;         // 步频 (步/分钟)
    bool valid;            // 是否有效
};

/**
 * 轨迹统计信息
 */
struct TrajectoryStats {
    float totalDistance;       // 总距离 (米)
    unsigned long totalTime;   // 总时间 (毫秒)
    float averageSpeed;        // 平均速度 (m/s)
    float maxSpeed;            // 最大速度 (m/s)
    int totalSteps;            // 总步数
    float averageCadence;      // 平均步频 (步/分钟)
    GPSPosition startPoint;    // 起点
    GPSPosition endPoint;      // 终点
    GPSPosition currentPoint;  // 当前位置
};

/**
 * 轨迹追踪器类
 * 功能：
 * 1. 记录运动路线
 * 2. 计算总运动距离
 * 3. 提供轨迹统计信息
 */
class TrajectoryTracker {
public:
    // 最大轨迹点数量
    static constexpr int MAX_TRAJECTORY_POINTS = 1000;

    TrajectoryTracker();
    ~TrajectoryTracker() = default;

    /**
     * 添加新的运动状态（主要函数）
     * @param state 运动状态
     */
    void addMotionState(const MotionState& state);

    /**
     * 获取总运动距离 (米)
     */
    float getTotalDistance() const { return stats_.totalDistance; }

    /**
     * 获取轨迹统计信息
     */
    TrajectoryStats getStats() const { return stats_; }

    /**
     * 获取轨迹点数组
     */
    const TrajectoryPoint* getTrajectoryPoints() const { return trajectoryPoints_; }

    /**
     * 获取轨迹点数量
     */
    int getTrajectoryPointCount() const { return trajectoryPointCount_; }

    /**
     * 重置追踪器
     */
    void reset();

    /**
     * 设置最小记录间隔（米）
     * 距离上次记录点超过此距离才记录新点
     */
    void setMinRecordDistance(float distance) {
        minRecordDistance_ = distance;
    }

    /**
     * 设置最小记录间隔（毫秒）
     * 时间上次记录点超过此时间才记录新点
     */
    void setMinRecordInterval(unsigned long interval) {
        minRecordInterval_ = interval;
    }

private:
    // 轨迹点数组（环形缓冲区）
    TrajectoryPoint trajectoryPoints_[MAX_TRAJECTORY_POINTS];
    int trajectoryPointCount_;

    // 统计信息
    TrajectoryStats stats_;

    // 记录控制
    float minRecordDistance_;        // 最小记录距离 (米)
    unsigned long minRecordInterval_; // 最小记录时间间隔 (毫秒)
    unsigned long lastRecordTime_;   // 上次记录时间
    GPSPosition lastRecordPos_;      // 上次记录位置

    // GPS处理器
    GPSProcessor gpsProcessor_;

    /**
     * 初始化统计信息
     */
    void initStats();

    /**
     * 更新统计信息
     */
    void updateStats(const MotionState& state);

    /**
     * 判断是否应该记录新的轨迹点
     */
    bool shouldRecordPoint(const MotionState& state) const;
};

#endif // __TRAJECTORY_TRACKER_H__
