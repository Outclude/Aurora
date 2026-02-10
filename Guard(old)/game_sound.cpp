#include "game_sound.h"
#include "Music.h"
#include "data.h"
#include <Arduino.h>

void reward_sound(){
    static bool oncePlay = false;
    static double last_dist = 0;
    // Get data (using double for calculation precision)
    double current_dist = RunStats_t.total_distance_m;
    double reward_dist = game_rewardDistance;
    double sum_dist = game_sumDistance;

    if (current_dist < last_dist) {
        last_dist = 0;
    }
    // Safety check to avoid division by zero
    if (reward_dist <= 0) {
        reward_dist = 100; 
    }
    oncePlay = false;
    // 1. 检查是否达成总距离
    if (last_dist < sum_dist && current_dist >= sum_dist) {
        MusicPlayAsync(1);
        oncePlay = true; 
    }

    // 2. 检查是否达成奖励距离 (跨越了新的奖励段)
    int last_segment = (int)(last_dist / reward_dist);
    int current_segment = (int)(current_dist / reward_dist);

    if (current_segment > last_segment && !oncePlay) {
        MusicPlayAsync(0);
    }
    Serial.printf("current_dist: %f, last_dist: %f, reward_dist: %f, sum_dist: %f\n",current_dist, last_dist, reward_dist, sum_dist);
    last_dist = current_dist;
}