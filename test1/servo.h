#ifndef __MY_SERVO_H__
#define __MY_SERVO_H__

#include <Arduino.h>

#define SERVO_PIN      23
#define SERVO_FREQ     50      
#define SERVO_RES      16

#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500


class MyServo {
public:
    MyServo();
    ~MyServo();
    

private:

};

#endif // __MY_SERVO_H__