<<<<<<< HEAD
=======
/*
 * 跑步灯光控制器实现
 *
 * 核心算法说明：
 *
 * 【问题背景】
 * 跑步时，人体上半身会产生周期性横向摆动，导致胸前单一投影灯投射到地面的光斑
 * 呈现周期性的U型摇摆，影响照明稳定性。
 *
 * 【解决方案】
 * 通过左右两个灯交替闪烁，并根据步态动态调整亮灯顺序和时序：
 * 1. 当左脚在前时，身体重心右移，此时右灯投影更稳定
 * 2. 当右脚在前时，身体重心左移，此时左灯投影更稳定
 * 3. 通过精确控制切换时机，使两个灯的投影区域最大化重叠
 *
 * 【时序控制】
 * - 根据步频动态调整亮灯时间和间隔
 * - 步频高时，减少亮灯时间和间隔
 * - 步频低时，增加亮灯时间和间隔
 * - 目标：在保证投影重叠的前提下，最小化间隔时间
 */

#include "vibrate.h"

// 构造函数
Vibrate::Vibrate()
    : lastWasLeft(false)
    , lastLightTime(0)
    , historyIndex(0)
    , minLightTime(30.0f)    // 最小亮灯时间 30ms
    , maxLightTime(100.0f)   // 最大亮灯时间 100ms
    , minInterval(10.0f)     // 最小间隔时间 10ms
    , maxInterval(50.0f)     // 最大间隔时间 50ms
{
    // 初始化步频历史数组
    for (int i = 0; i < 5; i++) {
        cadenceHistory[i] = 0;
    }
}

/**
 * 根据步态事件计算左右灯的最佳开关时间
 * @param stepEvent 步态事件
 * @return LightEvent 灯闪输出结构体
 */
LightEvent Vibrate::compute_freq(const StepEvent& stepEvent) {
    LightEvent event = {false, 0, 0, 0, false};

    // 如果没有检测到有效步态，返回无效事件
    if (!stepEvent.isStepValid) {
        event.isValid = false;
        return event;
    }

    // 更新步频历史（用于平滑）
    if (stepEvent.cadence > 0) {
        updateCadenceHistory(stepEvent.cadence);
    }

    // 获取平滑后的步频
    float smoothedCadence = getSmoothedCadence();

    // 如果步频数据无效，使用默认值
    if (smoothedCadence < 60 || smoothedCadence > 200) {
        smoothedCadence = 160; // 默认步频 160步/分钟
    }

    // 计算亮灯时间和间隔时间
    int lightTime = calculateLightTime(smoothedCadence);
    int intervalTime = calculateInterval(smoothedCadence);

    // 【核心算法】根据步态确定亮灯顺序
    // 原理：当左脚在前时，上身右倾，先亮右灯可以获得更好的投影效果
    //       当右脚在前时，上身左倾，先亮左灯可以获得更好的投影效果
    if (stepEvent.isLeftFoot) {
        // 左脚在前 → 上身右倾 → 先亮右灯
        event.leftFirst = false;
        event.r_time = lightTime;       // 右灯先亮
        event.l_time = lightTime;       // 左灯后亮
        event.interval = intervalTime;
    } else {
        // 右脚在前 → 上身左倾 → 先亮左灯
        event.leftFirst = true;
        event.l_time = lightTime;       // 左灯先亮
        event.r_time = lightTime;       // 右灯后亮
        event.interval = intervalTime;
    }

    // 根据置信度调整参数
    // 置信度低时，增加间隔时间以降低误判影响
    if (stepEvent.confidence < 0.5f) {
        event.interval = (int)(event.interval * 1.5f);
        event.l_time = (int)(event.l_time * 0.8f);
        event.r_time = (int)(event.r_time * 0.8f);
    }

    event.isValid = true;
    lastWasLeft = stepEvent.isLeftFoot;

    return event;
}

/**
 * 根据步频计算单次亮灯持续时间
 *
 * 计算公式：
 * - 步频 60步/分钟时，单步时间 = 1000ms，亮灯时间 = 100ms
 * - 步频 180步/分钟时，单步时间 = 333ms，亮灯时间 = 33ms
 * - 使用线性插值计算
 */
int Vibrate::calculateLightTime(float cadence) {
    // 线性插值：步频越高，亮灯时间越短
    float time = maxLightTime - (cadence - 60) * (maxLightTime - minLightTime) / (180 - 60);

    // 限制在最小和最大值之间
    if (time < minLightTime) time = minLightTime;
    if (time > maxLightTime) time = maxLightTime;

    return (int)time;
}

/**
 * 根据步频计算左右灯之间的间隔时间
 *
 * 计算公式：
 * - 步频 60步/分钟时，间隔时间 = 50ms
 * - 步频 180步/分钟时，间隔时间 = 10ms
 * - 使用线性插值计算
 */
int Vibrate::calculateInterval(float cadence) {
    // 线性插值：步频越高，间隔时间越短
    float interval = maxInterval - (cadence - 60) * (maxInterval - minInterval) / (180 - 60);

    // 限制在最小和最大值之间
    if (interval < minInterval) interval = minInterval;
    if (interval > maxInterval) interval = maxInterval;

    return (int)interval;
}

/**
 * 更新步频历史记录
 */
void Vibrate::updateCadenceHistory(float cadence) {
    cadenceHistory[historyIndex] = (int)cadence;
    historyIndex = (historyIndex + 1) % 5;
}

/**
 * 获取平滑后的步频值
 * 使用移动平均滤波
 */
float Vibrate::getSmoothedCadence() {
    int sum = 0;
    int count = 0;

    for (int i = 0; i < 5; i++) {
        if (cadenceHistory[i] > 0) {
            sum += cadenceHistory[i];
            count++;
        }
    }

    if (count == 0) {
        return 0.0f;
    }

    return (float)sum / count;
}

/**
 * 重置控制器状态
 */
void Vibrate::reset() {
    lastWasLeft = false;
    lastLightTime = 0;
    historyIndex = 0;

    for (int i = 0; i < 5; i++) {
        cadenceHistory[i] = 0;
    }
}
>>>>>>> 0b01d6c0a944cfe943829dbfc16ac771660c740d
