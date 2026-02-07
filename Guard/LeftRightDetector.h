#ifndef LEFT_RIGHT_DETECTOR_H
#define LEFT_RIGHT_DETECTOR_H

#include <math.h>

// 步态事件结构体
struct StepEvent {
    float timestamp;
    bool isLeftFoot;  // true = 左脚, false = 右脚
    float confidence;
};

class LeftRightDetector {
private:
    float prevLateralAcc;   // 横向加速度 (Y轴，假设X轴向前，Z轴向上)
    float prevVerticalAcc;
    int stepCount;
    
    // static 变量需要在类外定义，或者改为非 static
    // 这里改为成员变量，避免多实例冲突
    float peakAcc;
    float peakTime;
    bool inStep;

public:
    // 构造函数
    LeftRightDetector();
    
    /**
     * 检测左右脚
     * @param accX 前进方向加速度
     * @param accY 横向加速度（左右）⭐关键
     * @param accZ 垂直加速度
     * @param timestamp 时间戳
     * @return StepEvent 步态事件
     */
    StepEvent detectStep(float accX, float accY, float accZ, float timestamp);
    
    // 获取步数
    int getStepCount() const;
    
    // 重置检测器
    void reset();
};

#endif // LEFT_RIGHT_DETECTOR_H