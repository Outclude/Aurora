#include "gps_fusion.h"
#include "data.h"

GPSFusion::GPSFusion(float alpha, uint8_t min_gps_sats)
    : alpha_(alpha)
    , min_gps_sats_(min_gps_sats)
    , imu_velocity_(0.0)
    , fused_velocity_(0.0)
    , total_distance_(0.0)
    , last_gps_lat_(0.0)
    , last_gps_lon_(0.0)
    , last_gps_time_(0.0f)
    , gps_initialized_(false)
    , last_imu_time_(0.0f)
    , imu_initialized_(false)
{
}

void GPSFusion::reset() {
    imu_velocity_ = 0.0;
    fused_velocity_ = 0.0;
    total_distance_ = 0.0;
    last_gps_lat_ = 0.0;
    last_gps_lon_ = 0.0;
    last_gps_time_ = 0.0f;
    last_imu_time_ = 0.0f;
    gps_initialized_ = false;
    imu_initialized_ = false;
}

FusionResult GPSFusion::update(float imu_forward_acc, const GPSData& gps, float current_time) {
    FusionResult result;

    // 1. 更新IMU积分速度
    result.imu_speed = computeIMUVelocity(imu_forward_acc, current_time);

    // 2. 尝试获取GPS速度
    result.gps_speed = 0.0;
    bool has_valid_gps = false;

    if (gps.valid && gps.satellites >= min_gps_sats_) {
        double gps_vel = computeGPSVelocity(gps, current_time);
        if (gps_vel >= 0.0) {  // 有效GPS速度
            result.gps_speed = gps_vel;
            has_valid_gps = true;

            // 累积GPS距离
            if (gps_initialized_) {
                double distance_increment = computeGPSDistance(
                    last_gps_lat_, last_gps_lon_,
                    gps.latitude, gps.longitude
                );
                total_distance_ += distance_increment;
            }
        }
    }

    // 3. 融合策略
    if (has_valid_gps) {
        // 有GPS数据：互补滤波融合
        // fused = alpha * imu + (1 - alpha) * gps
        fused_velocity_ = alpha_ * result.imu_speed + (1.0f - alpha_) * result.gps_speed;
    } else {
        // 无GPS数据：使用IMU积分速度
        fused_velocity_ = result.imu_speed;
    }

    // 4. 更新SystemData（通过data.h接口）
    SystemData::getInstance().setCurrentSpeed(fused_velocity_);
    SystemData::getInstance().setDistance(total_distance_);

    // 5. 返回结果
    result.total_distance = total_distance_;
    result.current_speed = fused_velocity_;

    return result;
}

double GPSFusion::computeGPSVelocity(const GPSData& gps, float current_time) {
    // 首次GPS：初始化但不计算速度
    if (!gps_initialized_) {
        last_gps_lat_ = gps.latitude;
        last_gps_lon_ = gps.longitude;
        last_gps_time_ = current_time;
        gps_initialized_ = true;
        return 0.0;  // 首次无速度
    }

    // 时间差
    float dt = current_time - last_gps_time_;
    if (dt <= 0.0f || dt > 10.0f) {  // 无效时间差或间隔过大
        // 更新位置但不计算速度
        last_gps_lat_ = gps.latitude;
        last_gps_lon_ = gps.longitude;
        last_gps_time_ = current_time;
        return 0.0;
    }

    // 计算距离
    double distance = computeGPSDistance(
        last_gps_lat_, last_gps_lon_,
        gps.latitude, gps.longitude
    );

    // 更新状态
    last_gps_lat_ = gps.latitude;
    last_gps_lon_ = gps.longitude;
    last_gps_time_ = current_time;

    // 速度 = 距离 / 时间
    return distance / static_cast<double>(dt);
}

double GPSFusion::computeIMUVelocity(float forward_acc, float current_time) {
    // 将加速度从g转换为m/s²
    double acc_ms2 = forward_acc * GRAVITY;

    // 计算时间步长
    float dt = DEFAULT_DT;  // 默认值
    if (imu_initialized_ && current_time > 0.0f && last_imu_time_ > 0.0f) {
        float actual_dt = current_time - last_imu_time_;
        if (actual_dt > 0.0f && actual_dt < 1.0f) {  // 限制dt在合理范围
            dt = actual_dt;
        }
    }

    // 积分：v = v0 + a * dt
    imu_velocity_ += acc_ms2 * static_cast<double>(dt);

    last_imu_time_ = current_time;
    imu_initialized_ = true;

    return imu_velocity_;
}

double GPSFusion::computeGPSDistance(double lat1, double lon1, double lat2, double lon2) {
    // Haversine公式计算球面距离
    double lat1_rad = degToRad(lat1);
    double lon1_rad = degToRad(lon1);
    double lat2_rad = degToRad(lat2);
    double lon2_rad = degToRad(lon2);

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return EARTH_RADIUS * c;
}

double GPSFusion::degToRad(double degrees) const {
    return degrees * PI / 180.0;
}
