#include "data.h"

// 构造函数初始化   
SystemData::SystemData() {}

// 获取单例实例
SystemData& SystemData::getInstance() {
    static SystemData instance;
    return instance;
}

// Getters and Setters
int SystemData::getCadence() const { return cadence; }
void SystemData::setCadence(int val) { cadence = val; }

double SystemData::getCurrentSpeed() const { return current_speed; }
void SystemData::setCurrentSpeed(double val) { current_speed = val; }

double SystemData::getGoalSpeed() const { return goal_speed; }
void SystemData::setGoalSpeed(double val) { goal_speed = val; }

double SystemData::getDistance() const { return distance; }
void SystemData::setDistance(double val) { distance = val; }

double SystemData::getLastDistance() const { return last_distance; }
void SystemData::setLastDistance(double val) { last_distance = val; }

//游戏模式
double SystemData::getSumDistance() const { return sumDistance; }
void SystemData::setSumDistance(double val) { sumDistance = val; }

double SystemData::getRewardDistance() const { return rewardDistance; }
void SystemData::setRewardDistance(double val) { rewardDistance = val; }

bool SystemData::getStopRun() const { return stopRun; }
void SystemData::setStopRun(bool val) { stopRun = val; }

int SystemData::getMode() const { return mode; }
void SystemData::setMode(int val) { mode = val; }