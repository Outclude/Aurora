// ble_server.cpp
#include "ble_server.h"
#include <Arduino.h>

// 定义特性回调类
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override {
        std::string value = pCharacteristic->getValue();
        if (!value.empty()) {
            Serial.printf("Received string: %s\n", value.c_str());
        } else {
            Serial.println("Received empty string.");
        }
    }
};

void MyBLEServer::setup() { // 移除参数
    // 创建服务器
    pServer = NimBLEDevice::createServer();
    //pServer->setCallbacks(new ServerCallbacks()); // 添加服务器回调
    
    // 创建服务
    NimBLEService* pService = pServer->createService(SERVER_SERVICE_UUID);
    
    // 创建读特性（添加 WRITE 和 NOTIFY 以支持写入和通知）
    pReadCharacteristic = pService->createCharacteristic(
        SERVER_CHARREAD_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    pReadCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    
    // 创建写/通知特性（保持原样）
    pWriteCharacteristic = pService->createCharacteristic(
        SERVER_CHARWRIT_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    
    // 设置回调以处理写入事件
    pWriteCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    
    // 启动服务
    pService->start();
    
    // 配置广播
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVER_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setAppearance(0x00); // 添加appearance
    pAdvertising->start();
    
    Serial.println("BLE Server started successfully");
}

void MyBLEServer::sendString(const std::string& str) {
    if (pReadCharacteristic != nullptr) {  // 修改为使用 read 特性发送和通知
        // 设置特性值
        pReadCharacteristic->setValue(str);

        // 如果有订阅者，发送通知
        if (pReadCharacteristic->getSubscribedCount() > 0) {
            pReadCharacteristic->notify();
            Serial.printf("Sent string: %s\n", str.c_str());
        } 
        else {
            Serial.println("No client subscribed, string updated but not notified.");
        }
    }
}
