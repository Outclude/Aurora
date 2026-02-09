#include "game_rotate.h"
#include <Arduino.h>
#include "data.h"
#include "servo.h"

void horn_1() {
    Serial.println("horn_1 triggered");
    // TODO: Add hardware buzzer code here
}

void horn_2() {
    Serial.println("horn_2 triggered");
    // TODO: Add hardware buzzer code here
}

void game_rotate_loop() {
    static double last_dist = 0;
    
    // Get data (using double for calculation precision)
    double current_dist = RunStats_t.total_distance_m;
    double reward_dist = SystemData::getInstance().getRewardDistance();
    double sum_dist = SystemData::getInstance().getSumDistance();

    // Safety check to avoid division by zero
    if (reward_dist <= 0) {
        reward_dist = 100; 
    }

    // 2. 检查是否达成奖励距离 (跨越了新的奖励段)
    int last_segment = (int)(last_dist / reward_dist);
    int current_segment = (int)(current_dist / reward_dist);

    // 3. 计算角度 (基于速度差异)
    double current_speed = SystemData::getInstance().getCurrentSpeed();
    double goal_speed = SystemData::getInstance().getGoalSpeed();
    
    int angle = 90;

    if (goal_speed > 0.01) {
        // 速度越快(current > goal)，角度越小
        // 假设 1m/s 差异对应 90度
        angle = 90 - (int)((current_speed - goal_speed) * 45.0);
    }

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    servoWriteAngle(angle);

    last_dist = current_dist;
}
