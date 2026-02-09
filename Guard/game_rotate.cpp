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
    double current_dist = SystemData::getInstance().getDistance();
    double reward_dist = SystemData::getInstance().getRewardDistance();
    double sum_dist = SystemData::getInstance().getSumDistance();

    // Safety check to avoid division by zero
    if (reward_dist <= 0) {
        reward_dist = 100; 
    }

    // 1. 检查是否达成总距离
    if (last_dist < sum_dist && current_dist >= sum_dist) {
        horn_2();
    }

    // 2. 检查是否达成奖励距离 (跨越了新的奖励段)
    int last_segment = (int)(last_dist / reward_dist);
    int current_segment = (int)(current_dist / reward_dist);

    if (current_segment > last_segment) {
        horn_1();
    }

    // 3. 计算角度 (180 -> 0)
    // 计算当前段内的进度距离
    double dist_in_segment = current_dist - (current_segment * reward_dist);
    
    // 映射：距离越近(ratio越大)，角度越小
    double ratio = dist_in_segment / reward_dist;
    
    // 限制范围并执行
    int angle = (int)(180.0 * (1.0 - ratio));

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    servoWriteAngle(angle);

    last_dist = current_dist;
}
