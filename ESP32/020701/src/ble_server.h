// ble_server.h
#ifndef __MY_BLE_SERVER_H__
#define __MY_BLE_SERVER_H__
#include <NimBLEDevice.h>
#include <string>

// 自定义服务和特性UUID
//#define SERVER_SERVICE_UUID "12345678-ABCD-EFGH-IJKL-0123456789AB"
//#define SERVER_CHAR_UUID "00000001-ABCD-EFGH-IJKL-0123456789AB"

//#define SERVER_SERVICE_UUID "0000FFF0-0000-1000-8000-00805F9B34FB"
//#define SERVER_CHAR_UUID "0000FFF2-0000-1000-8000-00805F9B34FB"

// 使用16位UUID（十六进制格式）
#define SERVER_SERVICE_UUID    "FFF0"
//#define SERVER_CHAR_UUID       "FFF2"                    
#define SERVER_CHARREAD_UUID       "FFF2"
#define SERVER_CHARWRIT_UUID       "FFF1"
#define BLE_RX_MAX_LEN 64


// 修改类名，避免与NimBLE库冲突
class MyBLEServer {
public:
    MyBLEServer() : pServer(nullptr), pReadCharacteristic(nullptr), pWriteCharacteristic(nullptr) {}
    ~MyBLEServer() {}
    
    // 初始化BLE服务器，启动广播
    void setup();
    
    // 发送字符串到特性
    void sendString(const std::string& str);
    bool recvString(char *buf, uint32_t bufSize, uint32_t timeoutMs);


private:
    NimBLEServer* pServer;
    NimBLECharacteristic* pReadCharacteristic;
    NimBLECharacteristic* pWriteCharacteristic;
};
#endif
