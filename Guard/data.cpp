#include "data.h"

// 构造函数初始化
SystemData::SystemData() : cadence(0), pace_min(0), pace_sec(0) {}

// 获取单例实例
SystemData& SystemData::getInstance() {
    static SystemData instance;
    return instance;
}

// Getters and Setters
int SystemData::getCadence() const { return cadence; }
void SystemData::setCadence(int val) { cadence = val; }

int SystemData::getPaceMin() const { return pace_min; }
void SystemData::setPaceMin(int val) { pace_min = val; }

int SystemData::getPaceSec() const { return pace_sec; }
void SystemData::setPaceSec(int val) { pace_sec = val; }

int SystemData::getSumTime() const { return sum_time; }
void SystemData::setSumTime(int val) { sum_time = val; }

int SystemData::getDistance() const { return distance; }
void SystemData::setDistance(int val) { distance = val; }