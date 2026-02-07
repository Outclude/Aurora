#ifndef VIBERATE_H
#define VIBERATE_H

#include <math.h>
#include "LeftRightDetector.h"  // 包含AccEvent和StepEvent定义

// 灯闪输出结构体
struct LightEvent {
    bool leftFirst;      // true = 左灯先亮，false = 右灯先亮
    int l_time;          // 左灯持续时间（毫秒）
    int r_time;          // 右灯持续时间（毫秒）
    int interval;        // 左右灯之间的间隔时间（毫秒）
    bool isValid;        // 输出是否有效
};

/**
 * 跑步灯光控制器
 *
 * 功能：根据左右脚步态检测结果，计算左右投影灯的最佳开关时间
 *
 * 目标：通过控制左右两个灯的交替亮灭来：
 * 1. 让投影到地面的面积最大化重合
 * 2. 使中间间隔时间尽可能小
 * 3. 减弱跑步时上身横向摆动导致单一灯投影到地面周期性U型摇摆（"减振"作用）
 *
 * 算法原理：
 * - 当检测到左脚在前时，右侧投影灯效果更好（上身右倾）
 * - 当检测到右脚在前时，左侧投影灯效果更好（上身左倾）
 * - 通过动态调整亮灯顺序和持续时间，减少投影摇摆
 */
class Vibrate {
private:
    // 状态变量
    bool lastWasLeft;        // 上一次是否是左脚
    unsigned long lastLightTime;  // 上一次亮灯时间
    int cadenceHistory[5];   // 最近5次的步频历史
    int historyIndex;        // 历史数组索引

    // 可调参数
    float minLightTime;      // 最小亮灯时间（毫秒）
    float maxLightTime;      // 最大亮灯时间（毫秒）
    float minInterval;       // 最小间隔时间（毫秒）
    float maxInterval;       // 最大间隔时间（毫秒）

public:
    Vibrate();

    /**
     * 根据步态事件计算左右灯的最佳开关时间
     * @param stepEvent 步态事件，包含哪只脚在前、置信度、步频等信息
     * @return LightEvent 灯闪输出结构体，包含左右灯的开关时间和间隔
     */
    LightEvent compute_freq(const StepEvent& stepEvent);

    /**
     * 重置控制器状态
     */
    void reset();

private:
    /**
     * 根据步频计算单次亮灯持续时间
     * 步频越高，单次亮灯时间越短
     */
    int calculateLightTime(float cadence);

    /**
     * 根据步频计算左右灯之间的间隔时间
     * 步频越高，间隔时间越短
     */
    int calculateInterval(float cadence);

    /**
     * 更新步频历史记录（用于平滑）
     */
    void updateCadenceHistory(float cadence);

    /**
     * 获取平滑后的步频值
     */
    float getSmoothedCadence();
};

#endif
