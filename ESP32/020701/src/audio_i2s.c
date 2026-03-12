#include "audio_i2s.h"
#include "driver/i2s.h"
#include "esp_system.h"

#define I2S_PORT I2S_NUM_0

/* ===== I2S 引脚配置（按你实际接线改） ===== */
#define I2S_BCK_PIN 26
#define I2S_WS_PIN 22
#define I2S_DOUT_PIN 25

/* ===== 音频参数 ===== */
#define SAMPLE_RATE 8000
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT

void audio_i2s_init(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = 8000, // ✅ 对齐 WAV
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0};

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE};

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
}

void audio_i2s_play(const int16_t *pcm, size_t samples)
{
    size_t bytes_written = 0;
    size_t bytes_to_write = samples * sizeof(int16_t);

    i2s_write(
        I2S_PORT,
        (const char *)pcm,
        bytes_to_write,
        &bytes_written,
        portMAX_DELAY);
}

void audio_i2s_deinit(void)
{
    i2s_driver_uninstall(I2S_PORT);
}
