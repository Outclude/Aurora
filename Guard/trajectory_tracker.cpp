#include "trajectory_tracker.h"

// 默认参数
constexpr float DEFAULT_MIN_RECORD_DISTANCE = 5.0f;      // 默认最小记录距离 (米)
constexpr unsigned long DEFAULT_MIN_RECORD_INTERVAL = 1000;  // 默认最小记录间隔 (毫秒)

TrajectoryTracker::TrajectoryTracker()
    : trajectoryPointCount_(0)
    , minRecordDistance_(DEFAULT_MIN_RECORD_DISTANCE)
    , minRecordInterval_(DEFAULT_MIN_RECORD_INTERVAL)
    , lastRecordTime_(0)
{
    initStats();
    lastRecordPos_.valid = false;
}

void TrajectoryTracker::reset() {
    trajectoryPointCount_ = 0;
    lastRecordTime_ = 0;
    lastRecordPos_.valid = false;
    initStats();
}

void TrajectoryTracker::addMotionState(const MotionState& state) {
    // 更新统计信息
    updateStats(state);

    // 判断是否应该记录新的轨迹点
    if (!shouldRecordPoint(state)) {
        return;
    }

    // 检查数组是否已满
    if (trajectoryPointCount_ >= MAX_TRAJECTORY_POINTS) {
        // 数组已满，可以覆盖最旧的点（环形缓冲区）或忽略
        // 这里选择忽略
        return;
    }

    // 添加新的轨迹点
    TrajectoryPoint& point = trajectoryPoints_[trajectoryPointCount_];
    point.latitude = state.latitude;
    point.longitude = state.longitude;
    point.timestamp = state.timestamp;
    point.distance_from_start = stats_.totalDistance;
    point.speed = state.speed_fused;
    point.cadence = state.cadence;
    point.valid = state.position_valid;

    // 更新上次记录信息
    lastRecordTime_ = state.timestamp;
    lastRecordPos_.latitude = state.latitude;
    lastRecordPos_.longitude = state.longitude;
    lastRecordPos_.valid = state.position_valid;

    trajectoryPointCount_++;
}

void TrajectoryTracker::initStats() {
    memset(&stats_, 0, sizeof(TrajectoryStats));
    stats_.startPoint.valid = false;
    stats_.endPoint.valid = false;
    stats_.currentPoint.valid = false;
}

void TrajectoryTracker::updateStats(const MotionState& state) {
    // 更新总距离
    stats_.totalDistance += state.distance_increment;

    // 更新总时间
    if (stats_.startPoint.valid == false) {
        // 第一次，设置起点
        stats_.startPoint.latitude = state.latitude;
        stats_.startPoint.longitude = state.longitude;
        stats_.startPoint.timestamp = state.timestamp;
        stats_.startPoint.valid = state.position_valid;
        stats_.totalTime = 0;
    } else {
        // 更新总时间
        stats_.totalTime = state.timestamp - stats_.startPoint.timestamp;
    }

    // 更新终点和当前点
    stats_.endPoint.latitude = state.latitude;
    stats_.endPoint.longitude = state.longitude;
    stats_.endPoint.timestamp = state.timestamp;
    stats_.endPoint.valid = state.position_valid;

    stats_.currentPoint.latitude = state.latitude;
    stats_.currentPoint.longitude = state.longitude;
    stats_.currentPoint.timestamp = state.timestamp;
    stats_.currentPoint.valid = state.position_valid;

    // 更新速度统计
    float speed = state.speed_fused;
    if (speed > stats_.maxSpeed) {
        stats_.maxSpeed = speed;
    }

    // 更新平均速度
    if (stats_.totalTime > 0) {
        stats_.averageSpeed = stats_.totalDistance / (stats_.totalTime / 1000.0f);
    }

    // 更新步频统计（简单累加）
    if (state.step_detected && state.cadence > 0.0f) {
        stats_.totalSteps++;

        // 更新平均步频（滑动平均）
        if (stats_.averageCadence == 0.0f) {
            stats_.averageCadence = state.cadence;
        } else {
            // 使用0.1的更新率
            stats_.averageCadence = 0.9f * stats_.averageCadence + 0.1f * state.cadence;
        }
    }
}

bool TrajectoryTracker::shouldRecordPoint(const MotionState& state) const {
    // 检查位置是否有效
    if (!state.position_valid) {
        return false;
    }

    // 第一次记录
    if (trajectoryPointCount_ == 0) {
        return true;
    }

    // 检查时间间隔
    if (state.timestamp - lastRecordTime_ < minRecordInterval_) {
        return false;
    }

    // 检查距离间隔
    if (lastRecordPos_.valid) {
        float distance = gpsProcessor_.calculateDistance(
            lastRecordPos_.latitude,
            lastRecordPos_.longitude,
            state.latitude,
            state.longitude
        );
        if (distance < minRecordDistance_) {
            return false;
        }
    }

    return true;
}
