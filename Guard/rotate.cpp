#include "rotate.h"
#include <Arduino.h>
#include "data.h"

// 内部使用的辅助函数：获取数据并检查是否有更新
// 返回 true 表示有更新，false 表示无变化
// 如果有更新，将最新值通过引用参数传出
bool get_latest_params(int &pace_min, int &pace_sec) {
    // 1. 从数据中心获取最新参数
    int new_pace_min = SystemData::getInstance().getPaceMin();
    int new_pace_sec = SystemData::getInstance().getPaceSec();

    // 2. 状态监测
    static int last_pace_min = -1;
    static int last_pace_sec = -1;

    bool updated = false;

    // 检查参数是否发生变化
    if (new_pace_min != last_pace_min || new_pace_sec != last_pace_sec) {
        // 更新上次记录
        last_pace_min = new_pace_min;
        last_pace_sec = new_pace_sec;
        updated = true;
    }

    // 无论是否更新，都返回当前（最新）值
    pace_min = last_pace_min;
    pace_sec = last_pace_sec;

    return updated;
}

// 业务循环逻辑
// 这里的职责变为：主动从数据中心拉取数据 -> 执行业务
void rotate_loop() {
    int pace_min, pace_sec;

    // 调用独立函数获取数据并检查更新
    if (get_latest_params(pace_min, pace_sec)) {
        Serial.printf("Rotate module: Settings updated from SystemData - Pace: %d'%d''\n", 
            pace_min, pace_sec);
        
        // TODO: 在这里添加参数更新后的处理逻辑（如重置PID、调整目标速度等）
    }

    // TODO: 这里执行持续性的控制逻辑（如 PID 计算、电机驱动），每一轮 loop 都会执行
    // motor_control(cadence, pace_min, pace_sec);
}
