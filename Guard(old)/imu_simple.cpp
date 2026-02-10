#include "imu_simple.h"

IMUSimpleFilter::IMUSimpleFilter()
    : kalman_ax_(0.01f, 0.1f)  // (process_noise, measurement_noise)
    , kalman_ay_(0.01f, 0.1f)
    , kalman_az_(0.01f, 0.1f)
    , kalman_gx_(0.01f, 0.1f)
    , kalman_gy_(0.01f, 0.1f)
    , kalman_gz_(0.01f, 0.1f)
{
    // 初始化last_filtered_
    memset(&last_filtered_, 0, sizeof(IMUFilteredData));
}

IMUFilteredData IMUSimpleFilter::filter(const IMURawData& raw) {
    IMUFilteredData result;

    // 对6轴数据分别进行卡尔曼滤波
    result.ax = kalman_ax_.update(raw.ax);
    result.ay = kalman_ay_.update(raw.ay);
    result.az = kalman_az_.update(raw.az);
    result.gx = kalman_gx_.update(raw.gx);
    result.gy = kalman_gy_.update(raw.gy);
    result.gz = kalman_gz_.update(raw.gz);

    // 保存当前滤波结果
    last_filtered_ = result;

    return result;
}

void IMUSimpleFilter::reset() {
    kalman_ax_.reset();
    kalman_ay_.reset();
    kalman_az_.reset();
    kalman_gx_.reset();
    kalman_gy_.reset();
    kalman_gz_.reset();

    memset(&last_filtered_, 0, sizeof(IMUFilteredData));
}
