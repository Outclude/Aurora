#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server.h"

MyBLEServer bleServer;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Server...");

  // 初始化BLE设备，设置设备名称
  NimBLEDevice::init("My_ESP32E_Server");

  // 设置BLE服务器
  bleServer.setup();
}

void loop() {
  // 示例：在循环中每1秒发送一个测试字符串
  bleServer.sendString("Hi from ESP32!");
  delay(1000);
}
