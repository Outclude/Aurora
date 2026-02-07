#include "statistics.h"
#include "cJSON.h"
#include "ble_server.h"
#include "macro_defs.h"

extern MyBLEServer bleServer;

void sendStatistics(){
    statistics stats = {10, 20};
    cJSON* json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "type", RESPONSE_TYPE);
    cJSON_AddNumberToObject(json, "time", stats.time);
    cJSON_AddNumberToObject(json, "distance", stats.distance);
    char* jsonStr = cJSON_Print(json);
    bleServer.sendString(jsonStr);
    cJSON_Delete(json);
    free(jsonStr);
}