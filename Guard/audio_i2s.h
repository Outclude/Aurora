#ifndef __AUDIO_I2S_H__
#define __AUDIO_I2S_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化 I2S（用于 MAX98357A）
     */
    void audio_i2s_init(void);

    /**
     * @brief 播放 PCM 数据
     *
     * @param pcm      PCM 数据指针（int16_t）
     * @param samples  采样点数（不是字节数）
     */
    void audio_i2s_play(const int16_t *pcm, size_t samples);

    /**
     * @brief 反初始化 I2S
     */
    void audio_i2s_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_I2S_H__ */
