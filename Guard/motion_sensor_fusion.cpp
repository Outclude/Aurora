#include "motion_sensor_fusion.h"

// 物理常量
constexpr float GRAVITY = 9.81f;           // 重力加速度 (m/s^2)
constexpr float G_TO_M_S2 = 9.81f;         // g 转 m/s^2

// 步频计算参数
constexpr float DEFAULT_PEAK_THRESHOLD = 1.5f;  // 步态峰值阈值 (g)
constexpr unsigned long DEFAULT_MIN_STEP_INTERVAL = 200;  // 最小步态间隔(ms)
constexpr int CADENCE_WINDOW_SIZE = 5;            // 步频计算窗口大小

// 速度融合参数
constexpr float DEFAULT_STATIONARY_THRESHOLD = 0.3f;  // 静止阈值 (m/s)
constexpr float SPEED_IMU_WEIGHT_DEFAULT = 0.3f;     // IMU速度默认权重
constexpr float SPEED_GPS_WEIGHT_DEFAULT = 0.7f;     // GPS速度默认权重

// IMU速度漂移补偿
constexpr float DRIFT_COMPENSATION = 0.01f;  // 漂移补偿系数

MotionSensorFusion::MotionSensorFusion()
    : totalDistance_(0.0f)
    , currentSpeed_(0.0f)
    , currentCadence_(0.0f)
    , lastUpdateTime_(0)
    , stationaryThreshold_(DEFAULT_STATIONARY_THRESHOLD)
    , imuWeight_(SPEED_IMU_WEIGHT_DEFAULT)
    , gpsWeight_(SPEED_GPS_WEIGHT_DEFAULT)
{
    // 初始化GPS位置
    lastGPSPos_.valid = false;

    // 初始化步态检测状态
    stepState_.lastStepTime = 0;
    stepState_.stepCount = 0;
    stepState_.lastAccMagnitude = 0.0f;
    stepState_.rising = true;
    stepState_.peakThreshold = DEFAULT_PEAK_THRESHOLD;
    stepState_.minStepInterval = DEFAULT_MIN_STEP_INTERVAL;

    // 清空上次IMU数据
    memset(&lastIMUData_, 0, sizeof(IMUFilteredData));
}

void MotionSensorFusion::reset() {
    totalDistance_ = 0.0f;
    currentSpeed_ = 0.0f;
    currentCadence_ = 0.0f;
    lastUpdateTime_ = 0;
    lastGPSPos_.valid = false;
    memset(&lastIMUData_, 0, sizeof(IMUFilteredData));

    stepState_.lastStepTime = 0;
    stepState_.stepCount = 0;
    stepState_.lastAccMagnitude = 0.0f;
    stepState_.rising = true;
}

void MotionSensorFusion::setFusionWeights(float imuWeight, float gpsWeight) {
    // 归一化权重
    float total = imuWeight + gpsWeight;
    if (total > 0.0f) {
        imuWeight_ = imuWeight / total;
        gpsWeight_ = gpsWeight / total;
    }
}

MotionState MotionSensorFusion::update(const IMUFilteredData& imuFiltered,
                                        const GPSRawData& gpsRaw) {
    MotionState state;
    memset(&state, 0, sizeof(MotionState));

    unsigned long currentTime = imuFiltered.timestamp > 0 ? imuFiltered.timestamp : millis();

    // 计算时间间隔
    float dt = 0.0f;
    if (lastUpdateTime_ > 0) {
        dt = (currentTime - lastUpdateTime_) / 1000.0f;
    }
    lastUpdateTime_ = currentTime;
    state.timestamp = currentTime;

    // ===== 1. 步态检测 =====
    state.cadence = detectStepCadence(imuFiltered);
    state.step_detected = (state.cadence > 0.0f);

    // ===== 2. 速度计算 =====

    // 从IMU计算速度
    float speedIMU = 0.0f;
    if (dt > 0.0f) {
        speedIMU = calculateIMUSpeed(imuFiltered, dt);
    }
    state.speed_imu = speedIMU;

    // 从GPS计算速度
    float speedGPS = 0.0f;
    bool gpsValid = false;
    GPSProcessor gpsProc;
    GPSPosition currentGPSPos = gpsProc.convertToPosition(gpsRaw);

    if (lastGPSPos_.valid && currentGPSPos.valid) {
        speedGPS = gpsProc.calculateSpeed(lastGPSPos_, currentGPSPos);
        gpsValid = true;
    }
    state.speed_gps = speedGPS;
    lastGPSPos_ = currentGPSPos;

    // 融合速度
    state.speed_fused = fuseSpeed(speedIMU, speedGPS, gpsValid);
    currentSpeed_ = state.speed_fused;

    // ===== 3. 距离计算 =====
    if (dt > 0.0f) {
        state.distance_increment = calculateDistanceIncrement(state.speed_fused, dt);
        totalDistance_ += state.distance_increment;
    }

    // ===== 4. 位置信息 =====
    state.latitude = gpsRaw.latitude;
    state.longitude = gpsRaw.longitude;
    state.position_valid = currentGPSPos.valid;

    // ===== 5. 传感器权重（调试用） =====
    state.imu_weight = imuWeight_;
    state.gps_weight = gpsWeight_;

    // 保存本次IMU数据
    lastIMUData_ = imuFiltered;

    return state;
}

// ============ 私有函数实现 ============

float MotionSensorFusion::calculateIMUSpeed(const IMUFilteredData& imuFiltered, float dt) {
    // 方法1: 使用前进方向加速度积分
    // 假设设备穿戴正确，ax为前进方向加速度

    // 去除重力分量（假设静止时ax接近0）
    // 简化处理：使用合加速度的水平分量
    float forwardAccel = imuFiltered.acc_horizontal;

    // 转换单位：g -> m/s^2
    forwardAccel *= G_TO_M_S2;

    // 积分计算速度增量
    float speedIncrement = forwardAccel * dt;

    // 漂移补偿：长期静止时速度应趋向于0
    if (fabs(currentSpeed_) < stationaryThreshold_ && fabs(forwardAccel) < 0.1f) {
        currentSpeed_ *= (1.0f - DRIFT_COMPENSATION);
    }

    // 更新速度
    float newSpeed = currentSpeed_ + speedIncrement;

    // 速度不为负（假设不倒退）
    if (newSpeed < 0.0f) {
        newSpeed = 0.0f;
    }

    // 限制最大速度（跑步一般不超过12 m/s）
    constexpr float MAX_SPEED = 15.0f;
    if (newSpeed > MAX_SPEED) {
        newSpeed = MAX_SPEED;
    }

    return newSpeed;
}

float MotionSensorFusion::detectStepCadence(const IMUFilteredData& imuFiltered) {
    unsigned long currentTime = millis();

    // 使用合加速度检测步态峰值
    float accMagnitude = imuFiltered.acc_magnitude;
    float cadence = 0.0f;

    // 简单峰值检测算法
    if (stepState_.rising) {
        // 上升沿，寻找峰值
        if (accMagnitude > stepState_.lastAccMagnitude) {
            stepState_.lastAccMagnitude = accMagnitude;
        } else {
            // 检测到峰值
            if (stepState_.lastAccMagnitude > stepState_.peakThreshold) {
                // 检查最小间隔
                if (currentTime - stepState_.lastStepTime >= stepState_.minStepInterval) {
                    stepState_.stepCount++;
                    stepState_.lastStepTime = currentTime;

                    // 计算步频
                    if (stepState_.lastStepTime > 0) {
                        unsigned long stepInterval = currentTime - stepState_.lastStepTime;
                        if (stepInterval > 0) {
                            cadence = 60000.0f / stepInterval;  // 转换为步/分钟
                        }
                    }

                    currentCadence_ = cadence;
                }
            }
            stepState_.rising = false;
        }
    } else {
        // 下降沿，寻找谷值
        if (accMagnitude < stepState_.lastAccMagnitude) {
            stepState_.lastAccMagnitude = accMagnitude;
        } else {
            // 检测到谷值，准备检测下一个峰值
            stepState_.rising = true;
            stepState_.lastAccMagnitude = accMagnitude;
        }
    }

    return cadence;
}

float MotionSensorFusion::fuseSpeed(float speedIMU, float speedGPS, bool gpsValid) {
    // 融合策略：
    // 1. 如果GPS无效，使用IMU速度
    // 2. 如果速度很低（静止状态），倾向使用GPS速度（更准确）
    // 3. 如果速度较高，融合两者

    if (!gpsValid) {
        // GPS不可用，使用IMU
        return speedIMU;
    }

    if (isStationary(speedGPS)) {
        // GPS显示静止，认为静止
        return 0.0f;
    }

    // 动态调整权重
    float dynamicIMUWeight = imuWeight_;
    float dynamicGPSWeight = gpsWeight_;

    // 低速时增加GPS权重（GPS低速时更准确）
    if (speedGPS < 1.0f) {
        dynamicGPSWeight = 0.8f;
        dynamicIMUWeight = 0.2f;
    }
    // 高速时增加IMU权重（GPS延迟较大）
    else if (speedGPS > 5.0f) {
        dynamicGPSWeight = 0.5f;
        dynamicIMUWeight = 0.5f;
    }

    // 加权融合
    float fusedSpeed = dynamicIMUWeight * speedIMU + dynamicGPSWeight * speedGPS;

    return fusedSpeed;
}

float MotionSensorFusion::calculateDistanceIncrement(float speed, float dt) {
    return speed * dt;
}

bool MotionSensorFusion::isStationary(float speed) const {
    return speed < stationaryThreshold_;
}
