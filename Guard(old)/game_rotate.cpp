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
    double reward_dist = game_rewardDistance;
    double sum_dist = game_sumDistance;

    // Safety check to avoid division by zero
    if (reward_dist <= 0) {
        reward_dist = 100; 
    }

    double current_speed = SystemData::getInstance().getCurrentSpeed();
    double goal_speed = yu_she_pei_su;
    
    int angle = 90;

    if (goal_speed > 0.01) {
        // 速度越快(current > goal)，角度越小
        // 假设 1m/s 差异对应 90度
        angle = 90 - (int)((current_speed - goal_speed) * 45.0);
    }

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    g_servo_angle = angle;
    last_dist = current_dist;
}
