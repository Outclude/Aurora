#include "servo.h"
#include <Arduino.h>

static uint32_t dutyMin;
static uint32_t dutyMax;

// 去掉 static
void servoWriteAngle(int angle) {
    angle = constrain(angle, 0, 180);
    uint32_t duty = map(angle, 0, 180, dutyMin, dutyMax);
    ledcWrite(SERVO_CHANNEL, duty);
    Serial.print("Angle: ");
    Serial.println(angle);
}

void servo_setup() {
    // Configure PWM channel
    ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RES);
    // Attach pin to channel
    ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);

    dutyMin = (uint32_t)((SERVO_MIN_US / 20000.0) * ((1 << SERVO_RES) - 1));
    dutyMax = (uint32_t)((SERVO_MAX_US / 20000.0) * ((1 << SERVO_RES) - 1));

    servoWriteAngle(90);   // Center position
    delay(1000);
}

void servo_loop() {
    for (int angle = 0; angle <= 180; angle++) {
        servoWriteAngle(angle);
        delay(30);
    }
    delay(1000);
    for (int angle = 180; angle >= 0; angle--) {
        servoWriteAngle(angle);
        delay(30);
    }
    delay(1000);
}