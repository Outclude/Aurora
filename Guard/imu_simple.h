#ifndef __IMU_SIMPLE_H__
#define __IMU_SIMPLE_H__

#include <Arduino.h>

// IMU原始数据结构
struct IMURawData {
    float ax;  // X轴加速度 (g)
    float ay;  // Y轴加速度 (g)
    float az;  // Z轴加速度 (g)
    float gx;  // X轴角速度 (deg/s)
    float gy;  // Y轴角速度 (deg/s)
    float gz;  // Z轴角速度 (deg/s)

    IMURawData() : ax(0.0f), ay(0.0f), az(0.0f), gx(0.0f), gy(0.0f), gz(0.0f) {}
};

// 滤波后的IMU数据
struct IMUFilteredData {
    float ax;  // 滤波后X轴加速度 (g)
    float ay;  // 滤波后Y轴加速度 (g)
    float az;  // 滤波后Z轴加速度 (g)
    float gx;  // 滤波后X轴角速度 (deg/s)
    float gy;  // 滤波后Y轴角速度 (deg/s)
    float gz;  // 滤波后Z轴角速度 (deg/s)

    IMUFilteredData() : ax(0.0f), ay(0.0f), az(0.0f), gx(0.0f), gy(0.0f), gz(0.0f) {}
};

/**
 * 简单的卡尔曼滤波器（单变量）
 */
class SimpleKalmanFilter {
public:
    SimpleKalmanFilter(float process_noise = 0.01f, float measurement_noise = 0.1f)
        : Q(process_noise), R(measurement_noise), P(1.0f), X(0.0f) {}

    float update(float measurement) {
        // 预测步骤
        P = P + Q;

        // 更新步骤
        float K = P / (P + R);  // 卡尔曼增益
        X = X + K * (measurement - X);
        P = (1.0f - K) * P;

        return X;
    }

    void reset() {
        X = 0.0f;
        P = 1.0f;
    }

    float getValue() const { return X; }

private:
    float Q;  // 过程噪声
    float R;  // 测量噪声
    float P;  // 估计误差协方差
    float X;  // 状态估计值
};

/**
 * IMU滤波器类
 * 提供卡尔曼滤波处理IMU原始数据
 */
class IMUSimpleFilter {
public:
    IMUSimpleFilter();

    /**
     * 滤波处理（主入口）
     * @param raw 原始IMU数据
     * @return 滤波后的数据
     */
    IMUFilteredData filter(const IMURawData& raw);

    /**
     * 重置滤波器状态
     */
    void reset();

    /**
     * 获取当前滤波后的数据（不更新）
     */
    IMUFilteredData getCurrentFiltered() const { return last_filtered_; }

private:
    // 6个卡尔曼滤波器，分别处理6轴数据
    SimpleKalmanFilter kalman_ax_;
    SimpleKalmanFilter kalman_ay_;
    SimpleKalmanFilter kalman_az_;
    SimpleKalmanFilter kalman_gx_;
    SimpleKalmanFilter kalman_gy_;
    SimpleKalmanFilter kalman_gz_;

    IMUFilteredData last_filtered_;
};

#endif // __IMU_SIMPLE_H__
