#ifndef LEFT_RIGHT_DETECTOR_H
#define LEFT_RIGHT_DETECTOR_H

#include <Arduino.h>

// IMU输入结构体（对应CSV的ax, ay, az, gx, gy, gz）
struct AccEvent {
    unsigned long timestamp;  // 时间戳(ms)
    float ax;    // X轴加速度 (g)
    float ay;    // Y轴加速度 (g) - 横向摆动
    float az;    // Z轴加速度 (g) - 垂直方向
    float gx;    // X轴角速度 (deg/s)
    float gy;    // Y轴角速度 (deg/s)
    float gz;    // Z轴角速度 (deg/s)
};

// 步态事件结构体(输出)
struct StepEvent {
    unsigned long timestamp;  // 事件时间戳(ms)
    bool isLeftFoot;          // true = 左脚在前，false = 右脚在前
    float confidence;         // 检测置信度 [0-1]
    bool isStepValid;         // 是否检测到有效步态
    float cadence;            // 估计的步频（步/分钟）

    // 用于调试的原始和滤波后数据
    float raw_ay, raw_az, raw_gy;
    float filtered_ay, filtered_az, filtered_gy;
};

/**
 * 简单的卡尔曼滤波器（单变量）
 */
class KalmanFilter {
public:
    KalmanFilter(float process_noise = 0.01f, float measurement_noise = 0.1f, float initial_value = 0.0f)
        : Q(process_noise), R(measurement_noise), P(1.0f), X(initial_value) {}

    float update(float measurement) {
        // 预测步骤
        P = P + Q;

        // 更新步骤
        float K = P / (P + R);  // 卡尔曼增益
        X = X + K * (measurement - X);
        P = (1 - K) * P;

        return X;
    }

    void reset(float initial_value = 0.0f) {
        X = initial_value;
        P = 1.0f;
    }

    float getValue() const { return X; }

private:
    float Q;      // 过程噪声
    float R;      // 测量噪声
    float P;      // 估计误差协方差
    float X;      // 状态估计值
};

/**
 * 左右脚检测器
 * 根据IMU加速度和角速度数据，实时检测跑步姿态（哪只脚在前哪只脚在后）
 *
 * 原理：
 * 1. 横向加速度 (ay): 左脚着地时身体重心右移→正向加速度；右脚着地时身体重心左移→负向加速度
 * 2. 垂直加速度 (az): 检测着地冲击，用于步态周期识别
 * 3. 俯仰角速度 (gy): 上身前后摆动，辅助步态检测
 * 4. 融合ay和gy提高检测可靠性
 */
class LeftRightDetector {
private:
    // 滤波器
    KalmanFilter kalman_ay_;
    KalmanFilter kalman_az_;
    KalmanFilter kalman_gy_;

    // 步态检测变量
    int stepCount;
    bool inStepPhase;
    float last_filtered_ay;
    float last_filtered_az;
    float last_filtered_gy;

    // 步频计算变量
    unsigned long lastStepTime;
    unsigned long stepIntervalSum;
    int validStepCount;

    // 峰值检测（用于检测步态转换点）
    float localMax_ay;
    float localMin_ay;
    float localMax_az;
    float localMin_az;
    bool searchingMax;

    // 状态机
    enum State {
        STATE_UNKNOWN,
        STATE_LEFT_FORWARD,
        STATE_RIGHT_FORWARD
    };
    State currentState;

    // 参数
    static constexpr float AY_THRESHOLD = 0.15f;      // 横向加速度阈值
    static constexpr float AZ_THRESHOLD = 0.3f;       // 垂直加速度阈值
    static constexpr float GY_THRESHOLD = 0.05f;      // 角速度阈值
    static constexpr float MIN_STEP_INTERVAL = 200;   // 最小步态间隔(ms)

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
     * 低通滤波器（备用，简单移动平均）
     */
    float lowPassFilter(float newValue, float oldValue, float alpha = 0.3f);

    /**
     * 检测步态转换点（通过ay的极值点）
     * @return 检测到有效的步态转换
     */
    bool detectStepTransition(float ay, float az, float gy, unsigned long timestamp);

    /**
     * 融合ay, az, gy判断左右脚
     * @return true = 左脚在前，false = 右脚在前
     */
    bool classifyLeftRight(float ay, float az, float gy);

    /**
     * 计算置信度
     */
    float computeConfidence(float ay, float az, float gy, bool isLeft);
};

#endif // LEFT_RIGHT_DETECTOR_H
