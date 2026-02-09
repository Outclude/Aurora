#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "cJSON.h"
#include "statistics.h"
#include "rotate.h"
#include "data.h"
#include "servo.h"
#include "Music.h"
#include "I2S.h"
#include "game_sound.h"

MyBLEServer bleServer;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Server...");

    // 初始化BLE设备，设置设备名称
    NimBLEDevice::init("lqy_Server");
    // 设置BLE服务器
    bleServer.setup();
    servo_setup();
    I2S_Init();
}

void loop() {
    int mode = SystemData::getInstance().getMode();
    if (mode == 0) {
        // 配速模式
        rotate_loop();
    } else {
        // 游戏模式
        game_rotate_loop();
    }
}
