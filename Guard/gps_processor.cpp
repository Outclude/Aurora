#include "gps_processor.h"

// 地球半径 (米)
constexpr float EARTH_RADIUS = 6371000.0f;

GPSProcessor::GPSProcessor()
    : minSatellites_(4)
    , maxHDOP_(2.0f)
{
}

bool GPSProcessor::isValid(const GPSRawData& gps) const {
    // 检查卫星数量
    if (gps.satellites < minSatellites_) {
        return false;
    }

    // 检查经纬度范围
    if (gps.latitude < -90.0 || gps.latitude > 90.0) {
        return false;
    }
    if (gps.longitude < -180.0 || gps.longitude > 180.0) {
        return false;
    }

    // 检查是否为0坐标（无GPS信号）
    if (gps.latitude == 0.0 && gps.longitude == 0.0) {
        return false;
    }

    return gps.valid;
}

float GPSProcessor::calculateDistance(double lat1, double lon1,
                                       double lat2, double lon2) const {
    // Haversine公式计算球面距离
    float lat1Rad = degToRad(lat1);
    float lat2Rad = degToRad(lat2);
    float deltaLat = degToRad(lat2 - lat1);
    float deltaLon = degToRad(lon2 - lon1);

    float a = sinf(deltaLat / 2) * sinf(deltaLat / 2) +
              cosf(lat1Rad) * cosf(lat2Rad) *
              sinf(deltaLon / 2) * sinf(deltaLon / 2);

    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    return EARTH_RADIUS * c;
}

float GPSProcessor::calculateDistance(const GPSPosition& pos1,
                                       const GPSPosition& pos2) const {
    if (!pos1.valid || !pos2.valid) {
        return 0.0f;
    }
    return calculateDistance(pos1.latitude, pos1.longitude,
                            pos2.latitude, pos2.longitude);
}

float GPSProcessor::calculateSpeed(const GPSPosition& pos1,
                                    const GPSPosition& pos2) const {
    if (!pos1.valid || !pos2.valid) {
        return 0.0f;
    }

    // 计算时间差 (秒)
    float dt = (pos2.timestamp - pos1.timestamp) / 1000.0f;
    if (dt <= 0.001f) {  // 避免除零
        return 0.0f;
    }

    // 计算距离
    float distance = calculateDistance(pos1, pos2);

    return distance / dt;
}

float GPSProcessor::calculateBearing(double lat1, double lon1,
                                      double lat2, double lon2) const {
    float lat1Rad = degToRad(lat1);
    float lat2Rad = degToRad(lat2);
    float deltaLon = degToRad(lon2 - lon1);

    float y = sinf(deltaLon) * cosf(lat2Rad);
    float x = cosf(lat1Rad) * sinf(lat2Rad) -
              sinf(lat1Rad) * cosf(lat2Rad) * cosf(deltaLon);

    float bearing = atan2f(y, x);
    bearing = radToDeg(bearing);

    // 归一化到 [0, 360)
    if (bearing < 0.0f) {
        bearing += 360.0f;
    }

    return bearing;
}

GPSPosition GPSProcessor::convertToPosition(const GPSRawData& gps) const {
    GPSPosition pos;
    pos.latitude = gps.latitude;
    pos.longitude = gps.longitude;
    pos.timestamp = gps.timestamp;
    pos.speed = 0.0f;      // 默认值，后续可更新
    pos.altitude = 0.0f;   // 默认值
    pos.valid = isValid(gps);
    return pos;
}

float GPSProcessor::degToRad(float deg) const {
    return deg * (PI / 180.0f);
}

float GPSProcessor::radToDeg(float rad) const {
    return rad * (180.0f / PI);
}
