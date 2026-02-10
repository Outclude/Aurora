#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========= 全局 IMU 数据（单位：g / deg/s） ========= */
extern volatile float mpu_ax;
extern volatile float mpu_ay;
extern volatile float mpu_az;

extern volatile float mpu_gx;
extern volatile float mpu_gy;
extern volatile float mpu_gz;

/* ========= 接口 ========= */
void MPU6050_Init(void);
void MPU6050_Read(void);

#ifdef __cplusplus
}
#endif

#endif
