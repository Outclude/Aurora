#include "GPS.h"
#include <Arduino.h>
#include <TinyGPS++.h>
#include <string.h>
/* ========= 硬件配置 ========= */
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 115200

/* ========= TinyGPS 对象 ========= */
static TinyGPSPlus gps;

/* ========= 全局变量定义 ========= */
volatile double gps_lat = 0.0;
volatile double gps_lng = 0.0;
volatile int gps_sat = 0;

/* ========= 初始化 ========= */
void GPS_Init(void)
{
    Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

/* ========= GPS 解析 ========= */
void GPS_Task(void)
{
    /* 吃串口数据 */
    while (Serial1.available())
    {
        gps.encode(Serial1.read());
    }

    /* 更新经纬度 */
    if (gps.location.isUpdated())
    {
        gps_lat = gps.location.lat();
        gps_lng = gps.location.lng();
    }

    /* 更新卫星数 */
    if (gps.satellites.isUpdated())
    {
        gps_sat = gps.satellites.value();
    }
}
