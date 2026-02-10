#include "servo.h"
#include <Arduino.h>

/* ========= 舵机参数 ========= */
#define SERVO_PIN 5
#define SERVO_FREQ 50
#define SERVO_RES 16
#define SERVO_CH 0

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

/* ========= 内部变量 ========= */
static uint32_t dutyMin;
static uint32_t dutyMax;

/* ========= 初始化 ========= */
void Servo_Init(void)
{
    ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
    ledcAttachPin(SERVO_PIN, SERVO_CH);

    dutyMin = (uint32_t)((SERVO_MIN_US / 20000.0f) * ((1 << SERVO_RES) - 1));
    dutyMax = (uint32_t)((SERVO_MAX_US / 20000.0f) * ((1 << SERVO_RES) - 1));

    /* 上电回中 */
    Servo_SetAngle(90);
}

/* ========= 设定角度 ========= */
void Servo_SetAngle(int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    uint32_t duty = map(angle, 0, 180, dutyMin, dutyMax);
    ledcWrite(SERVO_CH, duty);
}
