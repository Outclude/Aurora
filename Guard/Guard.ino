#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "cJSON.h"

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
  // 可调超参


  // 示例：在循环中每1秒发送一个测试字符串
  bleServer.sendString("Hello from ESP32!");

  cJSON *data = cJSON_CreateObject();
  // TODO: 从硬件收到json格式的数据到data： 步频

  // 根据IMU提供的抖动信息，计算并返回左右灯的开关信息
  vibrate_data  = vibrate.compute_freq(data);
  cJSON *leftFirst = cJSON_GetNumberValue(vibrate_data, "leftFirst");
  cJSON *l_time = cJSON_GetNumberValue(vibrate_data, "l_time");
  cJSON *r_time = cJSON_GetNumberValue(vibrate_data, "r_time");
  cJSON *interval = cJSON_GetNumberValue(vibrate_data, "interval");

  // 
  
  // TODO: 
  delay(3000); //3秒内完成一个完整的左右臂摆动周期。
}
