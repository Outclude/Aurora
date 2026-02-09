#include "ble_handler.h"
#include "cJSON.h"
#include "data.h"
#include <Arduino.h>
#include "statistics.h"
#include "Music.h"

std::string BLEHandler::handleMessage(const std::string& message) {
    std::string response = "";
    int num = 0; // Count of successfully updated settings

    Serial.printf("Received raw value: %s\n", message.c_str());
    
    // Parse JSON
    cJSON *root = cJSON_Parse(message.c_str());
    if (root != NULL) {
        // Parse success, print formatted JSON for verification
        char *rendered = cJSON_Print(root);
        
        cJSON *typeItem = cJSON_GetObjectItem(root, "type");
        int type = -1;
        if (typeItem != NULL) {
            type = typeItem->valueint;
        }
        Serial.printf("Parsed JSON Object:\n%s\n", rendered);
        if (type == 0) {
            SystemData::getInstance().setMode(0);
            // Get and update cadence
            cJSON *cadence = cJSON_GetObjectItem(root, "cadence");
            if (cJSON_IsNumber(cadence)) {
                SystemData::getInstance().setCadence(cadence->valueint);
                num++;
            }

            // Get and update goal_speed
            cJSON *goalSpeed = cJSON_GetObjectItem(root, "goal_speed");
            if (cJSON_IsNumber(goalSpeed)) {
                if (goalSpeed->valuedouble != 0) {
                    SystemData::getInstance().setGoalSpeed(1000.0 / goalSpeed->valuedouble);
                } else {
                    SystemData::getInstance().setGoalSpeed(0);
                }
                num++;
            }
                
            // Create response message
            cJSON* json = cJSON_CreateObject();
            
            cJSON_AddStringToObject(json, "msg", num == 2 ? "Update_OK" : "Update_Error");
            char* jsonStr = cJSON_Print(json);
            if (jsonStr) {
                response = std::string(jsonStr);
                free(jsonStr);
            }
            cJSON_Delete(json);
        }else if (type == 1) {
            //开始跑步
            MusicPlayAsync(0);
            //重置距离
            SystemData::getInstance().setLastDistance(0);
            SystemData::getInstance().setDistance(0);
            SystemData::getInstance().setStopRun(false);
            cJSON* json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "msg", "Status_OK");
            char* jsonStr = cJSON_Print(json);
            if (jsonStr) {
                response = std::string(jsonStr);
                free(jsonStr);
            }
            cJSON_Delete(json);
        }else if (type == 2) {
            //暂停跑步，存储上一次距离
            SystemData::getInstance().setLastDistance(SystemData::getInstance().getDistance());
            SystemData::getInstance().setStopRun(true);
            sendStatistics();
            cJSON* json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "msg", "Status_OK");
            char* jsonStr = cJSON_Print(json);
            if (jsonStr) {
                response = std::string(jsonStr);
                free(jsonStr);
            }
            cJSON_Delete(json);
        }else if (type == 3) {
            //恢复跑步，读取上一次距离
            SystemData::getInstance().setDistance(SystemData::getInstance().getLastDistance());
            SystemData::getInstance().setStopRun(false);

            cJSON* json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "msg", "Status_OK");
            char* jsonStr = cJSON_Print(json);
            if (jsonStr) {
                response = std::string(jsonStr);
                free(jsonStr);
            }
            cJSON_Delete(json);
        }else if (type == 4) {
            //结束跑步，发送数据到前端
            SystemData::getInstance().setStopRun(true);
            MusicPlayAsync(1);
            sendStatistics();
            cJSON* json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "msg", "Status_OK");
            char* jsonStr = cJSON_Print(json);
            if (jsonStr) {
                response = std::string(jsonStr);
                free(jsonStr);
            }
            cJSON_Delete(json);
        }else if (type == 5) {
            SystemData::getInstance().setMode(1);
            //设置游戏模式目标
            cJSON *sumDistance = cJSON_GetObjectItem(root, "sumDistance");
            if (cJSON_IsNumber(sumDistance)) {
                SystemData::getInstance().setSumDistance(sumDistance->valueint);
                num++;
            }
            cJSON *rewardDistance = cJSON_GetObjectItem(root, "rewardDistance");
            if (cJSON_IsNumber(rewardDistance)) {
                SystemData::getInstance().setRewardDistance(rewardDistance->valueint);
                num++;
            }
            cJSON* json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "msg", num == 2 ? "Update_OK" : "Update_Error");
            char* jsonStr = cJSON_Print(json);
            if (jsonStr) {
                response = std::string(jsonStr);
                free(jsonStr);
            }
            cJSON_Delete(json);
        }
        
        free(rendered); // Free cJSON_Print memory
        cJSON_Delete(root); // Free cJSON object memory
    } else {
        // Parse failure handling
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            Serial.printf("JSON Parse Error before: %s\n", error_ptr);
        } else {
            Serial.println("JSON Parse Error: Unknown error");
        }
    }
    
    return response;
}
