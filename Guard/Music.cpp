#include "Music.h"
#include "I2S.h"
#include "SoundData.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static TaskHandle_t music_task_handle = NULL;

void MusicPlay(int id){
    const unsigned char* currentData = NULL;
    int dataSize = 0;

    // 根据ID选择音频数据
    // 注意：需要在 SoundData.h 中包含对应的数据定义
    switch(id) {
        case 0:
            currentData = sound_data_0;
            dataSize = sound_data_len_0;
            break;
        case 1:
            currentData = sound_data_1;
            dataSize = sound_data_len_1;
            break;
        default:
            return; // 无效ID不播放
    }

    int bytesWritten = 0;
    int chunkSize = 512; // Write in chunks
  
    for (int i = 0; i < dataSize; i += chunkSize) {
        int remaining = dataSize - i;
        int toWrite = (remaining < chunkSize) ? remaining : chunkSize;
        I2S_Write((char*)(currentData + i), toWrite);
    }

    // 发送静音数据以刷新DMA缓冲区，防止播放结束后的噪声
    // 缓冲区大小约为 16 * 60 * 4 = 3840 字节
    char silence[512];
    memset(silence, 0, sizeof(silence));
    for (int i = 0; i < 10; i++) { // 发送足够多的静音数据 (5120 bytes)
        I2S_Write(silence, sizeof(silence));
    }
}

void music_task(void *pvParameters) {
    int id = (int)pvParameters;
    MusicPlay(id);
    music_task_handle = NULL;
    vTaskDelete(NULL);
}

void MusicPlayAsync(int id) {
    if (music_task_handle != NULL) {
        vTaskDelete(music_task_handle);
        music_task_handle = NULL;
    }
    xTaskCreate(music_task, "music_task", 4096, (void*)id, 5, &music_task_handle);
}