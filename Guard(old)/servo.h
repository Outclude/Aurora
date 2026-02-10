#ifndef __MY_SERVO_H__
#define __MY_SERVO_H__

#include <Arduino.h>

#define SERVO_PIN      23
#define SERVO_FREQ     50      
#define SERVO_RES      16
#define SERVO_CHANNEL  1

#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500

void servo_setup();
void servo_loop();
// 新增声明
void servoWriteAngle(int angle); 

#endif // __MY_SERVO_H__