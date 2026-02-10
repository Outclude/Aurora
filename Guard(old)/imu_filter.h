#ifndef __IMU_FILTER_H__
#define __IMU_FILTER_H__

// 平台兼容性：支持 Arduino 和标准 C++
#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <cstdint>
    #include <cstring>
    #include <cmath>

    // Arduino 兼容宏
    #ifndef PI
        #define PI 3.14159265358979323846f
    #endif
#endif

// IMU原始数据结构
struct IMURawData {
    float ax;  // X轴加速度 (g)
    float ay;  // Y轴加速度 (g)
    float az;  // Z轴加速度 (g)
    float gx;  // X轴角速度 (deg/s)
    float gy;  // Y轴角速度 (deg/s)
    float gz;  // Z轴角速度 (deg/s)
};

// 滤波后的IMU数据
struct IMUFilteredData {
    float ax;  // 滤波后X轴加速度
    float ay;  // 滤波后Y轴加速度
    float az;  // 滤波后Z轴加速度
    float gx;  // 滤波后X轴角速度
    float gy;  // 滤波后Y轴角速度
    float gz;  // 滤波后Z轴角速度

    // 衍生数据
    float acc_magnitude;  // 合加速度大小
    float acc_horizontal; // 水平加速度分量
};

// 滤波器类型
enum FilterType {
    FILTER_NONE,        // 无滤波
    FILTER_KALMAN,      // 卡尔曼滤波
    FILTER_LOW_PASS,    // 低通滤波
    FILTER_MOVING_AVG   // 移动平均滤波
};

/**
 * IMU滤波器类
 * 提供多种滤波算法处理IMU原始数据
 */
class IMUFilter {
public:
    IMUFilter(FilterType type = FILTER_KALMAN);
    ~IMUFilter();

    /**
     * 滤波处理
     * @param raw 原始IMU数据
     * @return 滤波后的数据
     */
    IMUFilteredData filter(const IMURawData& raw);

    /**
     * 重置滤波器状态
     */
    void reset();

    /**
     * 设置滤波器类型
     */
    void setFilterType(FilterType type);

    /**
     * 获取当前滤波后的数据（不更新）
     */
    IMUFilteredData getCurrentFiltered() const;

private:
    FilterType filterType_;

    // 卡尔曼滤波器参数（6轴）
    struct KalmanState {
        float x;  // 状态估计值
        float p;  // 估计误差协方差
        float q;  // 过程噪声
        float r;  // 测量噪声
    };

    KalmanState kalman_ax_, kalman_ay_, kalman_az_;
    KalmanState kalman_gx_, kalman_gy_, kalman_gz_;

    // 低通滤波器状态
    struct LowPassState {
        float y;  // 上次输出值
        float alpha;  // 滤波系数
    };

    LowPassState lowpass_ax_, lowpass_ay_, lowpass_az_;
    LowPassState lowpass_gx_, lowpass_gy_, lowpass_gz_;

    // 移动平均滤波器状态
    static constexpr int MOVING_AVG_SIZE = 5;
    struct MovingAvgState {
        float buffer[MOVING_AVG_SIZE];
        int index;
        bool filled;
    };

    MovingAvgState moving_ax_, moving_ay_, moving_az_;
    MovingAvgState moving_gx_, moving_gy_, moving_gz_;

    IMUFilteredData lastFiltered_;

    // 内部滤波函数
    float kalmanUpdate(KalmanState& state, float measurement);
    float lowPassUpdate(LowPassState& state, float measurement);
    float movingAvgUpdate(MovingAvgState& state, float measurement);
    void resetKalman(KalmanState& state);
    void resetLowPass(LowPassState& state, float alpha = 0.3f);
    void resetMovingAvg(MovingAvgState& state);

    // 计算衍生数据
    void computeDerivedData(IMUFilteredData& data);
};

#endif // __IMU_FILTER_H__
