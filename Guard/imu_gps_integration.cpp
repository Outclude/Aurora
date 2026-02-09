#include "imu_gps_integration.h"
#include "data.h"

IMUGPSIntegration::IMUGPSIntegration(char forward_acc_key, float fusion_alpha, uint8_t min_gps_sats)
    : gps_fusion_(fusion_alpha, min_gps_sats)
    , forward_acc_key_(forward_acc_key)
{
    // 初始化result
    memset(&last_result_, 0, sizeof(FusionResult));
    memset(&last_imu_, 0, sizeof(IMUFilteredData));
}

IMUFilteredData IMUGPSIntegration::processIMU(const IMURawData& raw) {
    last_imu_ = imu_filter_.filter(raw);
    return last_imu_;
}

FusionResult IMUGPSIntegration::processGPS(float imu_forward_acc, const GPSData& gps, float current_time) {
    last_result_ = gps_fusion_.update(imu_forward_acc, gps, current_time);
    return last_result_;
}

FusionResult IMUGPSIntegration::update(const IMURawData& imu_raw, const GPSData& gps, float current_time) {
    // 1. IMU滤波
    last_imu_ = imu_filter_.filter(imu_raw);

    // 2. 提取前进方向加速度
    float forward_acc = getForwardAcc(last_imu_);

    // 3. GPS融合
    last_result_ = gps_fusion_.update(forward_acc, gps, current_time);

    return last_result_;
}

void IMUGPSIntegration::reset() {
    imu_filter_.reset();
    gps_fusion_.reset();
    memset(&last_result_, 0, sizeof(FusionResult));
    memset(&last_imu_, 0, sizeof(IMUFilteredData));
}

FusionResult IMUGPSIntegration::getResult() const {
    return last_result_;
}

IMUFilteredData IMUGPSIntegration::getFilteredIMU() const {
    return last_imu_;
}

void IMUGPSIntegration::setForwardAxis(char axis) {
    forward_acc_key_ = axis;
}

float IMUGPSIntegration::getForwardAcc(const IMUFilteredData& filtered) const {
    switch (forward_acc_key_) {
        case 'x':
        case 'X':
            return filtered.ax;
        case 'y':
        case 'Y':
            return filtered.ay;
        case 'z':
        case 'Z':
            return filtered.az;
        default:
            return filtered.ay;  // 默认使用Y轴
    }
}

void IMUGPSIntegration::printStatus() const {
    // 打印当前状态，类似于imu_gps_simp.py的输出
    Serial.println("========================================");
    Serial.print("Total Distance: ");
    Serial.print(last_result_.total_distance, 3);
    Serial.println(" m");

    Serial.print("Current Speed: ");
    Serial.print(last_result_.current_speed, 3);
    Serial.println(" m/s");

    Serial.print("IMU Speed: ");
    Serial.print(last_result_.imu_speed, 3);
    Serial.println(" m/s");

    Serial.print("GPS Speed: ");
    Serial.print(last_result_.gps_speed, 3);
    Serial.println(" m/s");

    Serial.println("--- IMU Filtered Data ---");
    Serial.print("ax: "); Serial.print(last_imu_.ax, 3);
    Serial.print("  ay: "); Serial.print(last_imu_.ay, 3);
    Serial.print("  az: "); Serial.println(last_imu_.az, 3);

    Serial.println("========================================");
}
