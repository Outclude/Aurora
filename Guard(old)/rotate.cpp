#include "rotate.h"
#include <Arduino.h>
#include "data.h"
#include "servo.h"

// 业务循环逻辑
// 这里的职责变为：主动从数据中心拉取数据 -> 执行业务
void rotate_loop() {
    double current_speed;
    double goal_speed;
    current_speed = SystemData::getInstance().getCurrentSpeed();
    goal_speed = yu_she_pei_su;

    int angle;
    // 防止除以0（嵌入式中必须处理，避免程序崩溃）
    if (goal_speed <= 0) {
        angle = 90; // 目标速度为0时，默认角度90
    } else {
        // 计算速度偏差比例：(目标速度 - 当前速度)/目标速度
        // 偏差为正（当前速度 < 目标速度）→ 角度增大；偏差为负（当前速度 > 目标速度）→ 角度减小
        double ratio = double(goal_speed - current_speed) / goal_speed;
        // 基础角度90，偏差比例乘以90作为增量/减量，范围刚好0~180
        angle = 90 + 90 * ratio;
    }
    servoWriteAngle(angle);
}
