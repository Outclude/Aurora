/*
 * 左右脚检测器实现
 *
 * 原理说明：
 * 跑步时，人的上半身会产生周期性的横向摆动和扭转：
 *
 * 1. 横向加速度(accY)模式：
 *    - 左脚着地：身体重心向右转移 → 产生向右加速度(accY > 0)
 *    - 右脚着地：身体重心向左转移 → 产生向左加速度(accY < 0)
 *
 * 2. 偏航角速度(omegaR)模式：
 *    - 左脚向前摆动时：上身自然向右扭转 → 正向偏航角速度(omegaR > 0)
 *    - 右脚向前摆动时：上身自然向左扭转 → 负向偏航角速度(omegaR < 0)
 *
 * 3. 融合策略：
 *    - 同时使用accY和omegaR进行检测，提高可靠性
 *    - 当两者信号一致时，置信度高
 *    - 当两者信号矛盾时，置信度低，可能处于步态过渡期
 */

#include "LeftRightDetector.h"

// 构造函数
LeftRightDetector::LeftRightDetector()
    : prevLateralAcc(0.0f)
    , prevYawRate(0.0f)
    , prevVerticalAcc(0.0f)
    , stepCount(0)
    , peakAcc(0.0f)
    , peakTime(0.0f)
    , inStep(false)
    , lastStepTime(0)
    , stepIntervalSum(0)
    , validStepCount(0)
    , filteredLateralAcc(0.0f)
    , filteredYawRate(0.0f)
{
}

/**
 * 检测左右脚步态（主函数）
 * @param acc IMU输入结构体
 * @return StepEvent 步态事件
 */
StepEvent LeftRightDetector::detectStep(const AccEvent& acc) {
    StepEvent event = {0, false, 0.0f, false, 0.0f};
    event.timestamp = acc.omegaP; // 使用任意字段作为临时时间戳（实际应传入真实时间戳）

    // 1. 低通滤波处理传感器噪声
    filteredLateralAcc = lowPassFilter(acc.accY, filteredLateralAcc, ALPHA);
    filteredYawRate = lowPassFilter(acc.omegaR, filteredYawRate, ALPHA);

    // 2. 检测是否为着地时刻（通过垂直加速度峰值）
    bool isLanding = detectLanding(acc);

    if (isLanding) {
        // 3. 融合accY和omegaR判断左右脚
        event.isLeftFoot = classifyLeftRight(acc);
        event.isStepValid = true;

        // 4. 计算置信度（基于信号强度和一致性）
        float lateralMag = fabs(filteredLateralAcc);
        float yawMag = fabs(filteredYawRate);
        float totalMag = lateralMag + yawMag;

        // 检查accY和omegaR信号方向是否一致
        bool signalsConsistent = (filteredLateralAcc * filteredYawRate) > 0;

        // 置信度计算：信号强度 + 一致性
        if (signalsConsistent) {
            event.confidence = min(1.0f, totalMag / 2.0f); // 信号一致时置信度高
        } else {
            event.confidence = min(0.6f, totalMag / 3.0f); // 信号矛盾时置信度降低
        }

        // 5. 更新步频统计
        unsigned long currentTime = (unsigned long)event.timestamp;
        if (lastStepTime > 0) {
            unsigned long interval = currentTime - lastStepTime;
            stepIntervalSum += interval;
            validStepCount++;

            // 计算平均步频（步/分钟）
            if (validStepCount > 0) {
                float avgInterval = (float)stepIntervalSum / validStepCount;
                event.cadence = 60000.0f / avgInterval; // 毫秒转分钟
            }
        }
        lastStepTime = currentTime;

        stepCount++;
    }

    // 更新状态变量
    prevLateralAcc = filteredLateralAcc;
    prevYawRate = filteredYawRate;
    prevVerticalAcc = acc.accZ;

    return event;
}

/**
 * 低通滤波器
 * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 */
float LeftRightDetector::lowPassFilter(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1.0f - alpha) * oldValue;
}

/**
 * 检测着地时刻
 * 通过垂直加速度峰值检测，当合加速度超过阈值时判定为着地
 */
bool LeftRightDetector::detectLanding(const AccEvent& acc) {
    // 计算合加速度
    float totalAcc = sqrt(acc.accX * acc.accX +
                          acc.accY * acc.accY +
                          acc.accZ * acc.accZ);

    // 着地检测：合加速度超过阈值且之前不在步态中
    const float LANDING_THRESHOLD = 1.5f * 9.8f; // 1.5g阈值
    const float RECOVERY_THRESHOLD = 1.2f * 9.8f; // 1.2g恢复阈值

    if (totalAcc > LANDING_THRESHOLD && !inStep) {
        inStep = true;
        peakAcc = totalAcc;
        peakTime = (unsigned long)acc.omegaP; // 临时使用omegaP存储时间
        return true;
    }

    // 当合加速度降低时，重置步态状态
    if (totalAcc < RECOVERY_THRESHOLD) {
        inStep = false;
    }

    return false;
}

/**
 * 融合横向加速度和偏航角速度判断左右脚
 * @return true = 左脚在前，false = 右脚在前
 */
bool LeftRightDetector::classifyLeftRight(const AccEvent& acc) {
    // 方法1：基于横向加速度
    // accY > 0 → 身体向右 → 左脚着地（左脚在前）
    // accY < 0 → 身体向左 → 右脚着地（右脚在前）
    bool lateralBasedLeft = (filteredLateralAcc > 0);

    // 方法2：基于偏航角速度
    // omegaR > 0 → 上身右转 → 左脚向前（左脚在前）
    // omegaR < 0 → 上身左转 → 右脚向前（右脚在前）
    bool yawBasedLeft = (filteredYawRate > 0);

    // 融合策略：投票法
    // 两种方法一致时直接返回，不一致时使用权重较大的信号
    float lateralWeight = fabs(filteredLateralAcc);
    float yawWeight = fabs(filteredYawRate);

    if (lateralBasedLeft == yawBasedLeft) {
        // 信号一致，返回统一结果
        return lateralBasedLeft;
    } else {
        // 信号矛盾，选择权重较大的信号
        return (lateralWeight > yawWeight) ? lateralBasedLeft : yawBasedLeft;
    }
}

/**
 * 重置检测器
 */
void LeftRightDetector::reset() {
    prevLateralAcc = 0.0f;
    prevYawRate = 0.0f;
    prevVerticalAcc = 0.0f;
    stepCount = 0;
    peakAcc = 0.0f;
    peakTime = 0.0f;
    inStep = false;
    lastStepTime = 0;
    stepIntervalSum = 0;
    validStepCount = 0;
    filteredLateralAcc = 0.0f;
    filteredYawRate = 0.0f;
}
