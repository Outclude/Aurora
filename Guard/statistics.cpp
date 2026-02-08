#include "statistics.h"
#include "cJSON.h"
#include "ble_server.h"
#include "servo.h"
#include "data.h"

extern MyBLEServer bleServer;
//发送结束的数据到页面
void sendStatistics(){
    cJSON* json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "distance", SystemData::getInstance().getDistance());
    char* jsonStr = cJSON_Print(json);
    bleServer.sendString(jsonStr);
    cJSON_Delete(json);
    free(jsonStr);
    
}