#ifndef LEFT_RIGHT_DETECTOR_H
#define LEFT_RIGHT_DETECTOR_H

#include <math.h>

// IMU输入结构体
struct AccEvent {
    float accX;    // 前进方向加速度
    float accY;    // 横向加速度（左右）- 正值表示向右，负值表示向左
    float accZ;    // 垂直方向加速度
    float omegaP;  // roll - 横滚角速度（x轴）
    float omegaQ;  // pitch - 俯仰角速度（y轴）
    float omegaR;  // yaw - 偏航角速度（z轴）⭐关键：检测上身扭转
};

// 步态事件结构体(输出)
struct StepEvent {
    float timestamp;       // 事件时间戳
    bool isLeftFoot;       // true = 左脚在前，false = 右脚在前
    float confidence;      // 检测置信度 [0-1]
    bool isStepValid;      // 是否检测到有效步态
    float cadence;         // 估计的步频（步/分钟）
};

/**
 * 左右脚检测器
 * 根据IMU加速度和角速度数据，实时检测跑步姿态（哪只脚在前哪只脚在后）
 *
 * 原理：
 * 1. 横向加速度 (accY): 左脚着地时身体重心右移→正向加速度；右脚着地时身体重心左移→负向加速度
 * 2. 偏航角速度 (omegaR): 左脚向前时上身右转→正向角速度；右脚向前时上身左转→负向角速度
 * 3. 融合accY和omegaR提高检测可靠性
 */
class LeftRightDetector {
private:
    // 状态变量
    float prevLateralAcc;     // 上一时刻横向加速度
    float prevYawRate;        // 上一时刻偏航角速度
    float prevVerticalAcc;    // 上一时刻垂直加速度

    // 步态检测变量
    int stepCount;
    float peakAcc;            // 步态峰值加速度
    float peakTime;           // 峰值时间
    bool inStep;              // 是否在步态周期中

    // 步频计算变量
    unsigned long lastStepTime;   // 上一次检测到步态的时间
    unsigned long stepIntervalSum; // 步态间隔累计（用于计算平均步频）
    int validStepCount;           // 有效步态计数

    // 滤波器变量（用于平滑噪声）
    float filteredLateralAcc;
    float filteredYawRate;
    static constexpr float ALPHA = 0.3f;  // 低通滤波系数

public:
    LeftRightDetector();

    /**
     * 检测左右脚步态
     * @param acc IMU输入结构体，包含加速度和角速度
     * @return StepEvent 步态事件，包含哪只脚在前、置信度、步频等信息
     */
    StepEvent detectStep(const AccEvent& acc);

    // 获取步数
    int getStepCount() const { return stepCount; }

    // 重置检测器
    void reset();

private:
    /**
     * 低通滤波器，用于平滑传感器噪声
     */
    float lowPassFilter(float newValue, float oldValue, float alpha);

    /**
     * 检测是否为着地时刻
     * 通过垂直加速度峰值检测
     */
    bool detectLanding(const AccEvent& acc);

    /**
     * 融合横向加速度和偏航角速度判断左右脚
     */
    bool classifyLeftRight(const AccEvent& acc);
};

#endif // LEFT_RIGHT_DETECTOR_H
