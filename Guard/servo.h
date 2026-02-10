#ifndef __MY_SERVO_H__
#define __MY_SERVO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========= 接口 ========= */
    void Servo_Init(void);
    void Servo_SetAngle(int angle); // 0~180°

#ifdef __cplusplus
}
#endif

#endif
