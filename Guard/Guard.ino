#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "statistics.h"
#include "rotate.h"
#include "data.h"
#include "servo.h"

MyBLEServer bleServer;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Server...");

    // 初始化BLE设备，设置设备名称
    NimBLEDevice::init("lqy_Server");
    // 设置BLE服务器
    bleServer.setup();
    servo_setup();
}

void loop() {
    // 业务模块自驱执行（内部已集成数据获取）
    //rotate_loop();
    // 示例：在循环中每1秒发送一个测试字符串
    bleServer.sendString("Hi from ESP32!");
    sendStatistics();

    // if (Serial.available() > 0) {
    //     // 方式1：读取字符串（直到换行，新手推荐）
    //     String receivedMsg = Serial.readStringUntil('\n');
    //     // 去除字符串两端的空白（回车、空格等）
    //     receivedMsg.trim();
    //     if (receivedMsg.length() > 0) {
    //         // 方式2：逐字节读取（适合处理单个字符/二进制数据）
    //         // char receivedChar = (char)Serial.read();

    //         // 板子回应：打印收到的消息
    //         Serial.print("ESP32已收到：");
    //         Serial.println(receivedMsg);
            
    //         int sec=receivedMsg.toInt();
    //         if (sec>=100)
    //             SystemData::getInstance().setCurrentPaceSec(sec);
    //     }
    
    // }
    delay(1000);
}
