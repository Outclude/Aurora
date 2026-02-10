// ble_server.cpp
#include "ble_server.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct
{
    char data[BLE_RX_MAX_LEN];
} BleRxMsg_t;

static QueueHandle_t rxQueue = nullptr;
// 定义特性回调类
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic,
                 ble_gap_conn_desc *desc) override
    {
        if (!rxQueue)
            return;

        std::string value = pCharacteristic->getValue();
        if (value.empty())
            return;

        BleRxMsg_t msg{};
        size_t len = value.length();
        if (len >= BLE_RX_MAX_LEN)
            len = BLE_RX_MAX_LEN - 1;

        memcpy(msg.data, value.data(), len);
        msg.data[len] = '\0';

        xQueueSend(rxQueue, &msg, 0);
    }
};

void MyBLEServer::setup()
{ // 移除参数

    rxQueue = xQueueCreate(8, sizeof(BleRxMsg_t));
    if (!rxQueue)
    {
        Serial.println("BLE rxQueue create failed");
        return;
    }
    // 创建服务器
    pServer = NimBLEDevice::createServer();
    // pServer->setCallbacks(new ServerCallbacks()); // 添加服务器回调

    // 创建服务
    NimBLEService *pService = pServer->createService(SERVER_SERVICE_UUID);

    // 创建读特性（添加 WRITE 和 NOTIFY 以支持写入和通知）
    pReadCharacteristic = pService->createCharacteristic(
        SERVER_CHARREAD_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pReadCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // 创建写/通知特性（保持原样）
    pWriteCharacteristic = pService->createCharacteristic(
        SERVER_CHARWRIT_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

    // 设置回调以处理写入事件
    pWriteCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // 启动服务
    pService->start();

    // 配置广播
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVER_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setAppearance(0x00); // 添加appearance
    pAdvertising->start();

    Serial.println("BLE Server started successfully");
}

void MyBLEServer::sendString(const std::string &str)
{
    if (pReadCharacteristic != nullptr)
    { // 修改为使用 read 特性发送和通知
        // 设置特性值
        pReadCharacteristic->setValue(str);

        // 如果有订阅者，发送通知
        if (pReadCharacteristic->getSubscribedCount() > 0)
        {
            pReadCharacteristic->notify();
            // Serial.printf("Sent string: %s\n", str.c_str());
        }
        else
        {
            // Serial.println("No client subscribed, string updated but not notified.");
        }
    }
}

bool MyBLEServer::recvString(char *buf,
                             uint32_t bufSize,
                             uint32_t timeoutMs)
{
    if (!rxQueue || !buf || bufSize == 0)
        return false;

    BleRxMsg_t msg;

    TickType_t t = (timeoutMs == 0)
                       ? 0
                       : pdMS_TO_TICKS(timeoutMs);

    if (xQueueReceive(rxQueue, &msg, t) == pdTRUE)
    {
        strncpy(buf, msg.data, bufSize - 1);
        buf[bufSize - 1] = '\0';
        return true;
    }
    return false;
}
