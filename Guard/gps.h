#ifndef __GPS_H__
#define __GPS_H__

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========= 全局 GPS 数据 ========= */
    extern volatile double gps_lat; // 纬度
    extern volatile double gps_lng; // 经度
    extern volatile int gps_sat;    // 卫星数

    /* ========= 接口 ========= */
    void GPS_Init(void);
    void GPS_Task(void); // 周期调用（RTOS task）

#ifdef __cplusplus
}
#endif

#endif
