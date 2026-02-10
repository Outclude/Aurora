/**
 * @file motion_tracker_example.c
 * @brief 运动追踪 C API 使用示例
 *
 * 纯 C 实现，用于底层嵌入式系统
 * 功能：IMU+GPS融合计算速度、步频、距离
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "motion_tracker_c.h"

/* ==================== 全局变量（模拟传感器数据） ==================== */

static volatile float ax = 0.0f;
static volatile float ay = 0.0f;
static volatile float az = 0.0f;
static volatile float gx = 0.0f;
static volatile float gy = 0.0f;
static volatile float gz = 0.0f;

static volatile double gps_lat = 0.0;
static volatile double gps_lng = 0.0;
static volatile int gps_sat = 0;

/* 模块句柄 */
static IMUFilter_C* g_imu_filter = NULL;
static MotionSensorFusion_C* g_motion_fusion = NULL;
static TrajectoryTracker_C* g_trajectory_tracker = NULL;

/* ==================== 平台相关函数（需要根据实际平台实现） ==================== */

/**
 * @brief 获取当前系统时间（毫秒）
 * @note 需要根据实际平台实现
 */
uint32_t millis(void) {
    /* TODO: 实现获取系统时间的函数 */
    /* STM32示例：return HAL_GetTick(); */
    /* ESP32示例：return xTaskGetTickCount() * portTICK_PERIOD_MS; */
    static uint32_t time = 0;
    return time++;
}

/**
 * @brief 延时函数（毫秒）
 * @note 需要根据实际平台实现
 */
void delay(uint32_t ms) {
    /* TODO: 实现延时函数 */
    /* STM32示例：HAL_Delay(ms); */
    /* ESP32示例：vTaskDelay(pdMS_TO_TICKS(ms)); */
    (void)ms;
}

/* ==================== 传感器读取函数 ==================== */

/**
 * @brief 读取 IMU 原始数据
 * @param raw 输出IMU数据
 * @return true=成功, false=失败
 */
bool read_imu_raw_data(IMURawData_C* raw) {
    if (raw == NULL) {
        return false;
    }

    /* 从全局变量读取（实际应用中从硬件读取） */
    raw->ax = ax;
    raw->ay = ay;
    raw->az = az;
    raw->gx = gx;
    raw->gy = gy;
    raw->gz = gz;
    raw->timestamp = millis();

    return true;
}

/**
 * @brief 读取 GPS 原始数据
 * @param raw 输出GPS数据
 * @return true=成功, false=失败
 */
bool read_gps_raw_data(GPSRawData_C* raw) {
    if (raw == NULL) {
        return false;
    }

    /* 从全局变量读取（实际应用中从GPS模块读取） */
    raw->latitude = gps_lat;
    raw->longitude = gps_lng;
    raw->satellites = gps_sat;
    raw->timestamp = millis();

    /* 简单验证数据有效性 */
    raw->valid = (raw->satellites >= 4) &&
                 (raw->latitude != 0.0) &&
                 (raw->longitude != 0.0);

    return true;
}

/* ==================== 调试打印函数 ==================== */

/**
 * @brief 打印运动状态（调试用）
 */
void print_motion_state(const MotionState_C* state) {
    if (state == NULL) return;

    printf("Speed: %.2f m/s (IMU: %.2f, GPS: %.2f) | Cadence: %.1f spm | Distance: %.1f m",
           state->speed_fused,
           state->speed_imu,
           state->speed_gps,
           state->cadence,
           MotionSensorFusion_GetTotalDistance(g_motion_fusion));

    if (state->step_detected) {
        printf(" [STEP]");
    }

    printf("\n");
}

/**
 * @brief 打印轨迹统计信息
 */
void print_trajectory_stats(void) {
    TrajectoryStats_C stats;
    TrajectoryTracker_GetStats(g_trajectory_tracker, &stats);

    printf("\n=== Trajectory Statistics ===\n");
    printf("Total Distance: %.2f m\n", stats.total_distance);
    printf("Total Time: %.1f s\n", stats.total_time / 1000.0f);
    printf("Average Speed: %.2f m/s\n", stats.average_speed);
    printf("Max Speed: %.2f m/s\n", stats.max_speed);
    printf("Total Steps: %d\n", (int)stats.total_steps);
    printf("Average Cadence: %.1f spm\n", stats.average_cadence);
    printf("============================\n");
}

/* ==================== API 封装函数 ==================== */

/**
 * @brief 初始化运动追踪模块
 * @return true=成功, false=失败
 */
bool motion_tracker_init(void) {
    /* 创建 IMU 滤波器（使用卡尔曼滤波） */
    g_imu_filter = IMUFilter_Create(FILTER_KALMAN_C);
    if (g_imu_filter == NULL) {
        printf("Failed to create IMU filter\n");
        return false;
    }

    /* 创建运动传感器融合对象 */
    g_motion_fusion = MotionSensorFusion_Create();
    if (g_motion_fusion == NULL) {
        printf("Failed to create motion fusion\n");
        IMUFilter_Destroy(g_imu_filter);
        return false;
    }

    /* 创建轨迹追踪器 */
    g_trajectory_tracker = TrajectoryTracker_Create();
    if (g_trajectory_tracker == NULL) {
        printf("Failed to create trajectory tracker\n");
        MotionSensorFusion_Destroy(g_motion_fusion);
        IMUFilter_Destroy(g_imu_filter);
        return false;
    }

    /* 设置参数 */
    MotionSensorFusion_SetStepPeakThreshold(g_motion_fusion, 1.5f);
    MotionSensorFusion_SetStationaryThreshold(g_motion_fusion, 0.3f);
    MotionSensorFusion_SetFusionWeights(g_motion_fusion, 0.3f, 0.7f);

    TrajectoryTracker_SetMinRecordDistance(g_trajectory_tracker, 5.0f);
    TrajectoryTracker_SetMinRecordInterval(g_trajectory_tracker, 1000);

    printf("Motion tracker initialized successfully\n");
    return true;
}

/**
 * @brief 反初始化运动追踪模块
 */
void motion_tracker_deinit(void) {
    if (g_trajectory_tracker != NULL) {
        TrajectoryTracker_Destroy(g_trajectory_tracker);
        g_trajectory_tracker = NULL;
    }

    if (g_motion_fusion != NULL) {
        MotionSensorFusion_Destroy(g_motion_fusion);
        g_motion_fusion = NULL;
    }

    if (g_imu_filter != NULL) {
        IMUFilter_Destroy(g_imu_filter);
        g_imu_filter = NULL;
    }
}

/**
 * @brief 更新运动追踪（主循环调用）
 */
void motion_tracker_update(void) {
    IMURawData_C imu_raw;
    IMURawData_C imu_filtered;
    GPSRawData_C gps_raw;
    MotionState_C motion_state;

    /* 1. 读取 IMU 原始数据 */
    if (!read_imu_raw_data(&imu_raw)) {
        return;
    }

    /* 2. 读取 GPS 原始数据 */
    read_gps_raw_data(&gps_raw);  /* GPS 无效也可以继续 */

    /* 3. IMU 数据滤波 */
    IMUFilter_Filter(g_imu_filter, &imu_raw, &imu_filtered);

    /* 4. 融合 IMU 和 GPS 数据，计算运动状态 */
    MotionSensorFusion_Update(g_motion_fusion, &imu_filtered, &gps_raw, &motion_state);

    /* 5. 记录轨迹点 */
    TrajectoryTracker_AddMotionState(g_trajectory_tracker, &motion_state);

    /* 6. 打印实时数据（调试） */
    print_motion_state(&motion_state);
}

/**
 * @brief 获取实时速度 (m/s)
 */
float get_real_time_speed(void) {
    return MotionSensorFusion_GetSpeed(g_motion_fusion);
}

/**
 * @brief 获取总运动距离 (米)
 */
float get_total_distance(void) {
    return MotionSensorFusion_GetTotalDistance(g_motion_fusion);
}

/**
 * @brief 获取实时步频 (步/分钟)
 */
float get_real_time_cadence(void) {
    return MotionSensorFusion_GetCadence(g_motion_fusion);
}

/**
 * @brief 重置运动追踪
 */
void reset_motion_tracking(void) {
    MotionSensorFusion_Reset(g_motion_fusion);
    TrajectoryTracker_Reset(g_trajectory_tracker);
    IMUFilter_Reset(g_imu_filter);
}

/* ==================== 主程序示例 ==================== */

/**
 * @brief 初始化
 */
void setup(void) {
    printf("\n=== Motion Tracker Example ===\n");
    printf("Initializing modules...\n");

    if (!motion_tracker_init()) {
        printf("Initialization failed!\n");
        return;
    }

    printf("Initialization complete!\n");
    printf("Waiting for sensor data...\n\n");
}

/**
 * @brief 主循环
 */
void loop(void) {
    /* 更新运动追踪 */
    motion_tracker_update();

    /* 定期打印统计信息（每10秒） */
    static uint32_t last_stats_print = 0;
    if (millis() - last_stats_print > 10000) {
        print_trajectory_stats();
        last_stats_print = millis();
    }

    /* 采样周期：10ms (100Hz) */
    delay(10);
}

/**
 * @brief 主函数
 */
int main(void) {
    /* 初始化 */
    setup();

    /* 主循环 */
    while (1) {
        loop();
    }

    /* 清理资源（实际不会执行到这里） */
    motion_tracker_deinit();

    return 0;
}

/* ==================== 测试代码示例 ==================== */

#ifdef ENABLE_TEST

/**
 * @brief 测试函数 - 模拟跑步数据
 */
void test_running_data(void) {
    /* 模拟跑步时的IMU数据 */
    ax = 0.5f;  /* 前进加速度 */
    ay = 0.2f;  /* 左右摆动 */
    az = 1.2f;  /* 垂直冲击 */
    gx = 0.1f;
    gy = 0.5f;
    gz = 0.05f;

    /* 模拟GPS数据（北京天安门坐标） */
    gps_lat = 39.9042f;
    gps_lng = 116.4074f;
    gps_sat = 8;

    /* 运行100次循环 */
    for (int i = 0; i < 100; i++) {
        loop();
    }
}

#endif /* ENABLE_TEST */
