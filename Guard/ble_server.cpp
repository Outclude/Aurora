// ble_server.cpp
#include "ble_server.h"
#include <Arduino.h>
#include "cJSON.h"
#include "data.h"

// 定义特性回调类
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    MyBLEServer* pServer;
public:
    MyCharacteristicCallbacks(MyBLEServer* server) : pServer(server) {}
    void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override {
        std::string value = pCharacteristic->getValue();
        
        if (value.length() > 0) {
            Serial.printf("Received raw value: %s\n", value.c_str());
            
            // 解析 JSON
            cJSON *root = cJSON_Parse(value.c_str());
            if (root != NULL) {
                // 解析成功，打印格式化后的 JSON 字符串以验证
                char *rendered = cJSON_Print(root);
                if (rendered != NULL) {
                    Serial.printf("Parsed JSON Object:\n%s\n", rendered);
                    

                    // 检查 type 是否为 1 (配置更新)
                    cJSON *type = cJSON_GetObjectItem(root, "type");
                    if (cJSON_IsNumber(type) && type->valueint == 1) {
                        // 获取并更新 cadence
                        cJSON *cadence = cJSON_GetObjectItem(root, "cadence");
                        if (cJSON_IsNumber(cadence)) {
                            SystemData::getInstance().setCadence(cadence->valueint);
                        }

                        // 获取并更新 pace_min
                        cJSON *paceMin = cJSON_GetObjectItem(root, "pace_min");
                        if (cJSON_IsNumber(paceMin)) {
                            SystemData::getInstance().setPaceMin(paceMin->valueint);
                        }

                        // 获取并更新 pace_sec
                        cJSON *paceSec = cJSON_GetObjectItem(root, "pace_sec");
                        if (cJSON_IsNumber(paceSec)) {
                            SystemData::getInstance().setPaceSec(paceSec->valueint);
                        }

                        Serial.printf("Updated settings: Cadence=%d, Pace=%d'%d''\n", 
                            SystemData::getInstance().getCadence(),
                            SystemData::getInstance().getPaceMin(),
                            SystemData::getInstance().getPaceSec());
                    }
                    
                    free(rendered); // 释放 cJSON_Print 分配的内存
                }
                cJSON_Delete(root); // 释放 cJSON 对象内存
            } else {
                // 解析失败处理
                const char *error_ptr = cJSON_GetErrorPtr();
                if (error_ptr != NULL) {
                    Serial.printf("JSON Parse Error before: %s\n", error_ptr);
                } else {
                    Serial.println("JSON Parse Error: Unknown error");
                }
            }
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
    pReadCharacteristic->setCallbacks(new MyCharacteristicCallbacks(this));
    
    // 创建写/通知特性（保持原样）
    pWriteCharacteristic = pService->createCharacteristic(
        SERVER_CHARWRIT_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    
    // 设置回调以处理写入事件
    pWriteCharacteristic->setCallbacks(new MyCharacteristicCallbacks(this));
    
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
            //Serial.printf("Sent string: %s\n", str.c_str());
        } 
        else {
            Serial.println("NN client subscribed, string updated but not notified.");
        }
    }
}
