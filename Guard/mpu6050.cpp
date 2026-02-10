#include "MPU6050.h"
#include <Wire.h>
#include <Arduino.h>
#include <string.h>

/* ========= MPU6050 I2C 地址 ========= */
#define MPU6050_ADDR 0x68
#define I2C_SCL_PIN 18
#define I2C_SDA_PIN 19
/* ========= 原始寄存器 ========= */
static int16_t accX, accY, accZ;
static int16_t gyroX, gyroY, gyroZ;

/* ========= 全局变量定义 ========= */
volatile float mpu_ax = 0;
volatile float mpu_ay = 0;
volatile float mpu_az = 0;

volatile float mpu_gx = 0;
volatile float mpu_gy = 0;
volatile float mpu_gz = 0;

/* ========= 初始化 ========= */
void MPU6050_Init(void)
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(50);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // ±2g
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // ±250 dps
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);
}

/* ========= 读取六轴 ========= */
void MPU6050_Read(void)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14);

    accX = (Wire.read() << 8) | Wire.read();
    accY = (Wire.read() << 8) | Wire.read();
    accZ = (Wire.read() << 8) | Wire.read();

    Wire.read(); // temp
    Wire.read();

    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();

    /* ---- 单位换算 ---- */
    mpu_ax = accX / 16384.0f;
    mpu_ay = accY / 16384.0f;
    mpu_az = accZ / 16384.0f;

    mpu_gx = gyroX / 131.0f;
    mpu_gy = gyroY / 131.0f;
    mpu_gz = gyroZ / 131.0f;
}
