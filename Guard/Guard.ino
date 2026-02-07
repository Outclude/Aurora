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
  bleServer.sendString("Hello from ESP32!");
  // TODO: 根据IMU提供的抖动信息，计算并返回左右灯的开关信息
  bool leftFirst;
  int left_time, right_time, interval; 
  leftFirst, l_time, r_time, interval = vibrate.compute_freq();
  // TODO: Decide which light to first turn on
  for (int i = 0; i < 5; i++) {
    cout << i << " ";  // Output: 0 1 2 3 4
    delay(50);
  }
  delay()
  for (int i = 0; i < 5; i++) {
    cout << i << " ";  // Output: 0 1 2 3 4
    delay(50);
  }
  // TODO: 
  delay(3000); //3秒内完成一个完整的左右臂摆动周期。
}
