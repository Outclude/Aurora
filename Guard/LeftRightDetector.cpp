/*
 * 左右脚检测器实现
 *
 * 原理说明：
 * 跑步时，人的上半身会产生周期性的横向摆动：
 *
 * 1. 横向加速度(ay)模式：
 *    - 左脚着地：身体重心向右转移 → 产生向右加速度(ay > 0)
 *    - 右脚着地：身体重心向左转移 → 产生向左加速度(ay < 0)
 *
 * 2. 垂直加速度(az)模式：
 *    - 着地时产生冲击峰值
 *    - 用于检测步态周期
 *
 * 3. 俯仰角速度(gy)模式：
 *    - 辅助检测上身前后摆动
 *
 * 4. 融合策略：
 *    - 使用卡尔曼滤波器平滑噪声
 *    - 检测ay的极值点作为步态转换点
 *    - 结合az的峰值验证着地时刻
 */

#include "LeftRightDetector.h"
#include <math.h>

// 构造函数
LeftRightDetector::LeftRightDetector()
    : kalman_ay_(0.01f, 0.1f, 0.0f)
    , kalman_az_(0.01f, 0.1f, 0.0f)
    , kalman_gy_(0.005f, 0.05f, 0.0f)
    , stepCount(0)
    , inStepPhase(false)
    , last_filtered_ay(0.0f)
    , last_filtered_az(0.0f)
    , last_filtered_gy(0.0f)
    , lastStepTime(0)
    , stepIntervalSum(0)
    , validStepCount(0)
    , localMax_ay(-999.0f)
    , localMin_ay(999.0f)
    , localMax_az(-999.0f)
    , localMin_az(999.0f)
    , searchingMax(true)
    , currentState(STATE_UNKNOWN)
{
}

/**
 * 检测左右脚步态（主函数）
 * @param acc IMU输入结构体
 * @return StepEvent 步态事件
 */
StepEvent LeftRightDetector::detectStep(const AccEvent& acc) {
    StepEvent event = {0, false, 0.0f, false, 0.0f};
    event.timestamp = acc.timestamp;

    // 保存原始数据用于调试
    event.raw_ay = acc.ay;
    event.raw_az = acc.az;
    event.raw_gy = acc.gy;

    // 1. 应用卡尔曼滤波
    float filtered_ay = kalman_ay_.update(acc.ay);
    float filtered_az = kalman_az_.update(acc.az);
    float filtered_gy = kalman_gy_.update(acc.gy);

    // 保存滤波后数据用于调试
    event.filtered_ay = filtered_ay;
    event.filtered_az = filtered_az;
    event.filtered_gy = filtered_gy;

    // 2. 检测步态转换点
    bool stepDetected = detectStepTransition(filtered_ay, filtered_az, filtered_gy, acc.timestamp);

    if (stepDetected) {
        // 3. 融合ay, az, gy判断左右脚
        event.isLeftFoot = classifyLeftRight(filtered_ay, filtered_az, filtered_gy);

        // 4. 计算置信度
        event.confidence = computeConfidence(filtered_ay, filtered_az, filtered_gy, event.isLeftFoot);
        event.isStepValid = true;

        // 5. 更新步频统计
        if (lastStepTime > 0) {
            unsigned long interval = acc.timestamp - lastStepTime;
            if (interval >= MIN_STEP_INTERVAL) {
                stepIntervalSum += interval;
                validStepCount++;

                // 计算平均步频（步/分钟）
                if (validStepCount > 0) {
                    float avgInterval = (float)stepIntervalSum / validStepCount;
                    event.cadence = 60000.0f / avgInterval;
                }
            }
        }
        lastStepTime = acc.timestamp;
        stepCount++;
    } else {
        // 没有检测到新步态，返回当前状态
        event.isLeftFoot = (currentState == STATE_LEFT_FORWARD);
        event.confidence = 0.0f;
        event.isStepValid = false;
    }

    // 更新历史值
    last_filtered_ay = filtered_ay;
    last_filtered_az = filtered_az;
    last_filtered_gy = filtered_gy;

    return event;
}

/**
 * 低通滤波器（备用）
 */
float LeftRightDetector::lowPassFilter(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1.0f - alpha) * oldValue;
}

/**
 * 检测步态转换点（通过ay的极值点检测）
 * 原理：跑步时横向加速度ay呈现周期性变化，极值点对应步态转换
 */
bool LeftRightDetector::detectStepTransition(float ay, float az, float gy, unsigned long timestamp) {
    // 检查最小时间间隔
    if (lastStepTime > 0 && (timestamp - lastStepTime) < MIN_STEP_INTERVAL) {
        return false;
    }

    // 更新局部极值
    if (searchingMax) {
        if (ay > localMax_ay) {
            localMax_ay = ay;
        } else if (ay < localMax_ay - AY_THRESHOLD) {
            // 检测到局部最大值后的下降
            searchingMax = false;
            localMin_ay = ay;

            // 检查垂直加速度峰值（验证着地）
            if (fabs(az) > AZ_THRESHOLD) {
                // 更新状态机
                if (localMax_ay > 0) {
                    // ay > 0 → 左脚在前
                    if (currentState != STATE_LEFT_FORWARD) {
                        currentState = STATE_LEFT_FORWARD;
                        localMax_ay = -999.0f;
                        return true;
                    }
                } else {
                    // ay < 0 → 右脚在前
                    if (currentState != STATE_RIGHT_FORWARD) {
                        currentState = STATE_RIGHT_FORWARD;
                        localMax_ay = -999.0f;
                        return true;
                    }
                }
            }
            localMax_ay = -999.0f;
        }
    } else {
        if (ay < localMin_ay) {
            localMin_ay = ay;
        } else if (ay > localMin_ay + AY_THRESHOLD) {
            // 检测到局部最小值后的上升
            searchingMax = true;
            localMax_ay = ay;
            localMin_ay = 999.0f;
        }
    }

    return false;
}

/**
 * 融合ay, az, gy判断左右脚
 * @return true = 左脚在前，false = 右脚在前
 */
bool LeftRightDetector::classifyLeftRight(float ay, float az, float gy) {
    // 方法1：基于横向加速度
    // ay > 0 → 身体向右 → 左脚着地（左脚在前）
    // ay < 0 → 身体向左 → 右脚着地（右脚在前）
    bool ayBasedLeft = (ay > 0);

    // 方法2：基于俯仰角速度（辅助）
    // 跑步时上身会前后摆动，可以辅助判断
    bool gyBasedLeft = (gy > 0);

    // 方法3：基于垂直加速度的变化趋势
    bool azTrendingUp = (az > last_filtered_az);

    // 融合策略：主要使用ay信号，其他作为辅助
    float ayWeight = fabs(ay);
    float gyWeight = fabs(gy) * 0.3f;  // 降低gy权重
    float azWeight = fabs(az) * 0.2f;  // 降低az权重

    // 如果ay信号足够强，直接使用ay判断
    if (ayWeight > AY_THRESHOLD) {
        return ayBasedLeft;
    }

    // 否则使用加权投票
    int leftVotes = 0;
    int rightVotes = 0;

    if (ayBasedLeft) leftVotes++; else rightVotes++;
    if (gyBasedLeft) leftVotes++; else rightVotes++;
    if (azTrendingUp) leftVotes++; else rightVotes++;

    return (leftVotes > rightVotes);
}

/**
 * 计算置信度
 */
float LeftRightDetector::computeConfidence(float ay, float az, float gy, bool isLeft) {
    float ayMag = fabs(ay);
    float azMag = fabs(az);
    float gyMag = fabs(gy);

    // 归一化信号强度
    float ayConf = min(1.0f, ayMag / AY_THRESHOLD);
    float azConf = min(1.0f, azMag / AZ_THRESHOLD);
    float gyConf = min(1.0f, gyMag / GY_THRESHOLD);

    // 加权平均
    float confidence = ayConf * 0.6f + azConf * 0.3f + gyConf * 0.1f;

    // 限制范围
    return max(0.0f, min(1.0f, confidence));
}

/**
 * 重置检测器
 */
void LeftRightDetector::reset() {
    stepCount = 0;
    inStepPhase = false;
    last_filtered_ay = 0.0f;
    last_filtered_az = 0.0f;
    last_filtered_gy = 0.0f;
    lastStepTime = 0;
    stepIntervalSum = 0;
    validStepCount = 0;
    localMax_ay = -999.0f;
    localMin_ay = 999.0f;
    localMax_az = -999.0f;
    localMin_az = 999.0f;
    searchingMax = true;
    currentState = STATE_UNKNOWN;

    kalman_ay_.reset(0.0f);
    kalman_az_.reset(0.0f);
    kalman_gy_.reset(0.0f);
}
