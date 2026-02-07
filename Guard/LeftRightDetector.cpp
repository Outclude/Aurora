/*
利用左右步加速度不对称性（或融合算法或机器学习），通过IMU判断当前姿态是迈左步还是迈右步：

    - 左脚着地：身体重心向右偏移，产生向右的横向加速度
    - 右脚着地：身体重心向左偏移，产生向左的横向加速度

*/

#include "LeftRightDetector.h"

// 构造函数
LeftRightDetector::LeftRightDetector() 
    : prevLateralAcc(0.0f)
    , prevVerticalAcc(0.0f)
    , stepCount(0)
    , peakAcc(0.0f)
    , peakTime(0.0f)
    , inStep(false)
{
}

// 检测步态
StepEvent LeftRightDetector::detectStep(float accX, float accY, float accZ, float timestamp) {
    StepEvent event = {0.0f, false, 0.0f};
    
    // 计算合加速度（用于检测步态峰值）
    float totalAcc = sqrt(accX * accX + accY * accY + accZ * accZ);
    
    // 着地检测逻辑（简化版）
    if (totalAcc > 1.5f * 9.8f && !inStep) {  // 阈值：1.5g
        inStep = true;
        peakAcc = totalAcc;
        peakTime = timestamp;
        
        // ⭐关键：判断左右
        // 左脚着地 → 身体右倾 → accY > 0 (假设右为正)
        // 右脚着地 → 身体左倾 → accY < 0
        
        float lateralPeak = accY;  // 着地时的横向加速度
        
        event.timestamp = timestamp;
        event.isLeftFoot = (lateralPeak > 0.0f);  // 根据坐标系调整符号
        event.confidence = fabs(lateralPeak) / totalAcc;  // 置信度
        
        stepCount++;
    }
    
    if (totalAcc < 1.2f * 9.8f) {
        inStep = false;
    }
    
    return event;
}

// 获取步数
int LeftRightDetector::getStepCount() const {
    return stepCount;
}

// 重置检测器
void LeftRightDetector::reset() {
    prevLateralAcc = 0.0f;
    prevVerticalAcc = 0.0f;
    stepCount = 0;
    peakAcc = 0.0f;
    peakTime = 0.0f;
    inStep = false;
}