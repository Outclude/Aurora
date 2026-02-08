#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "servo.h"

// 新增：定义PWM通道号（避免硬编码，选1号通道，0~15均可）
#define SERVO_CHANNEL 1

uint32_t dutyMin;
uint32_t dutyMax;

/* ================= 舵机角度控制 ================= */
void servoWriteAngle(int angle) {
    angle = constrain(angle, 0, 180);

    uint32_t duty = map(angle, 0, 180, dutyMin, dutyMax);

    // 关键修改1：ledcWrite第一个参数从SERVO_PIN改为SERVO_CHANNEL
    ledcWrite(SERVO_CHANNEL, duty);

    Serial.print("Angle: ");
    Serial.println(angle);
}

MyBLEServer bleServer;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Server...");

  // 初始化BLE设备，设置设备名称
  NimBLEDevice::init("lxy_ESP32_Server");

  // 设置BLE服务器
  bleServer.setup();

  // 关键修改2：替换废弃的ledcAttach为新版PWM初始化方式
  // 原ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES) 移除
  // 步骤1：配置PWM通道（通道号、频率、分辨率）
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RES);
  // 步骤2：将引脚绑定到该通道
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);

  dutyMin = (uint32_t)((SERVO_MIN_US / 20000.0) * ((1 << SERVO_RES) - 1));
  dutyMax = (uint32_t)((SERVO_MAX_US / 20000.0) * ((1 << SERVO_RES) - 1));

  servoWriteAngle(90);   // 上电回中
  delay(1000);
}

void loop() {
  // 示例：在循环中每1秒发送一个测试字符串
  bleServer.sendString("Hello from ESP32!");
  delay(1000);
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