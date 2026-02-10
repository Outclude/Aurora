#include "imu_filter.h"

// 地球重力加速度 (g)
#ifndef GRAVITY
    constexpr float GRAVITY = 9.81f;
#endif

// 确保 PI 已定义
#ifndef PI
    #define PI 3.14159265358979323846f
#endif

IMUFilter::IMUFilter(FilterType type) : filterType_(type) {
    reset();
}

IMUFilter::~IMUFilter() {}

void IMUFilter::reset() {
    // 重置卡尔曼滤波器
    resetKalman(kalman_ax_);
    resetKalman(kalman_ay_);
    resetKalman(kalman_az_);
    resetKalman(kalman_gx_);
    resetKalman(kalman_gy_);
    resetKalman(kalman_gz_);

    // 重置低通滤波器
    resetLowPass(lowpass_ax_);
    resetLowPass(lowpass_ay_);
    resetLowPass(lowpass_az_);
    resetLowPass(lowpass_gx_);
    resetLowPass(lowpass_gy_);
    resetLowPass(lowpass_gz_);

    // 重置移动平均滤波器
    resetMovingAvg(moving_ax_);
    resetMovingAvg(moving_ay_);
    resetMovingAvg(moving_az_);
    resetMovingAvg(moving_gx_);
    resetMovingAvg(moving_gy_);
    resetMovingAvg(moving_gz_);

    // 清空上次输出
    memset(&lastFiltered_, 0, sizeof(IMUFilteredData));
}

void IMUFilter::setFilterType(FilterType type) {
    filterType_ = type;
}

IMUFilteredData IMUFilter::filter(const IMURawData& raw) {
    IMUFilteredData result;

    switch (filterType_) {
        case FILTER_KALMAN:
            result.ax = kalmanUpdate(kalman_ax_, raw.ax);
            result.ay = kalmanUpdate(kalman_ay_, raw.ay);
            result.az = kalmanUpdate(kalman_az_, raw.az);
            result.gx = kalmanUpdate(kalman_gx_, raw.gx);
            result.gy = kalmanUpdate(kalman_gy_, raw.gy);
            result.gz = kalmanUpdate(kalman_gz_, raw.gz);
            break;

        case FILTER_LOW_PASS:
            result.ax = lowPassUpdate(lowpass_ax_, raw.ax);
            result.ay = lowPassUpdate(lowpass_ay_, raw.ay);
            result.az = lowPassUpdate(lowpass_az_, raw.az);
            result.gx = lowPassUpdate(lowpass_gx_, raw.gx);
            result.gy = lowPassUpdate(lowpass_gy_, raw.gy);
            result.gz = lowPassUpdate(lowpass_gz_, raw.gz);
            break;

        case FILTER_MOVING_AVG:
            result.ax = movingAvgUpdate(moving_ax_, raw.ax);
            result.ay = movingAvgUpdate(moving_ay_, raw.ay);
            result.az = movingAvgUpdate(moving_az_, raw.az);
            result.gx = movingAvgUpdate(moving_gx_, raw.gx);
            result.gy = movingAvgUpdate(moving_gy_, raw.gy);
            result.gz = movingAvgUpdate(moving_gz_, raw.gz);
            break;

        case FILTER_NONE:
        default:
            result.ax = raw.ax;
            result.ay = raw.ay;
            result.az = raw.az;
            result.gx = raw.gx;
            result.gy = raw.gy;
            result.gz = raw.gz;
            break;
    }

    computeDerivedData(result);
    lastFiltered_ = result;
    return result;
}

IMUFilteredData IMUFilter::getCurrentFiltered() const {
    return lastFiltered_;
}

// ============ 私有函数实现 ============

float IMUFilter::kalmanUpdate(KalmanState& state, float measurement) {
    // 预测步骤
    state.p = state.p + state.q;

    // 更新步骤
    float K = state.p / (state.p + state.r);  // 卡尔曼增益
    state.x = state.x + K * (measurement - state.x);
    state.p = (1.0f - K) * state.p;

    return state.x;
}

float IMUFilter::lowPassUpdate(LowPassState& state, float measurement) {
    // 一阶低通滤波: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    state.y = state.alpha * measurement + (1.0f - state.alpha) * state.y;
    return state.y;
}

float IMUFilter::movingAvgUpdate(MovingAvgState& state, float measurement) {
    state.buffer[state.index] = measurement;
    state.index = (state.index + 1) % MOVING_AVG_SIZE;

    if (!state.filled && state.index == 0) {
        state.filled = true;
    }

    int count = state.filled ? MOVING_AVG_SIZE : state.index;
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += state.buffer[i];
    }

    return sum / count;
}

void IMUFilter::resetKalman(KalmanState& state) {
    state.x = 0.0f;
    state.p = 1.0f;
    state.q = 0.01f;   // 过程噪声
    state.r = 0.1f;    // 测量噪声
}

void IMUFilter::resetLowPass(LowPassState& state, float alpha) {
    state.y = 0.0f;
    state.alpha = alpha;
}

void IMUFilter::resetMovingAvg(MovingAvgState& state) {
    memset(state.buffer, 0, sizeof(state.buffer));
    state.index = 0;
    state.filled = false;
}

void IMUFilter::computeDerivedData(IMUFilteredData& data) {
    // 计算合加速度大小
    data.acc_magnitude = sqrtf(data.ax * data.ax + data.ay * data.ay + data.az * data.az);

    // 计算水平加速度分量 (假设z轴垂直于地面)
    // 这里简单使用x-y平面加速度
    data.acc_horizontal = sqrtf(data.ax * data.ax + data.ay * data.ay);
}
