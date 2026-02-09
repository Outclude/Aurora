#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "LeftRightDetector.h"
#include "cJSON.h"

MyBLEServer bleServer;
LeftRightDetector detector;

// IMU数据读取（需要根据实际硬件连接实现）
// 这里提供一个模拟数据的示例，实际使用时需要替换为真实的IMU读取函数

/**
 * 从IMU读取6轴数据
 * TODO: 根据实际硬件实现此函数
 * 示例：使用MPU6050、BMI160等IMU传感器
 */
bool readIMUData(AccEvent& acc) {
    // TODO: 替换为真实的IMU数据读取代码
    // 例如：
    // // 读取MPU6050
    // Wire.beginTransmission(MPU6050_ADDR);
    // Wire.write(0x3B);  // 加速度寄存器起始地址
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU6050_ADDR, 14);
    // acc.ax = (Wire.read() << 8 | Wire.read()) / 16384.0f;  // 转换为g
    // acc.ay = (Wire.read() << 8 | Wire.read()) / 16384.0f;
    // acc.az = (Wire.read() << 8 | Wire.read()) / 16384.0f;
    // // ... 读取陀螺仪数据

    // 模拟数据（用于测试）- 从CSV数据中选取的典型值
    static int simIndex = 0;
    static float simData[][6] = {
        {-9.197f, 0.615f, 0.421f, 0.1462f, 0.3218f, -0.1949f},
        {-9.317f, 0.438f, 0.423f, 0.1292f, 0.3104f, -0.1547f},
        {-9.343f, 0.383f, 0.383f, 0.1074f, 0.2341f, -0.0983f},
        {-9.939f, 0.156f, -0.282f, -0.016f, 0.2345f, -0.0596f},
        {-10.097f, 0.057f, -0.412f, 0.0597f, 0.1412f, 0.0112f},
        {-9.594f, 0.297f, -0.328f, 0.0935f, 0.1256f, -0.0827f},
        {-9.173f, 0.502f, -0.34f, 0.0902f, 0.1472f, -0.0626f},
        {-9.381f, 0.44f, -0.309f, 0.0991f, 0.1276f, 0.0913f},
        {-9.728f, 0.613f, -0.213f, 0.0886f, 0.1335f, 0.0216f},
        {-9.446f, 0.653f, -0.16f, 0.0438f, 0.1657f, -0.059f},
    };

    acc.timestamp = millis();
    acc.ax = simData[simIndex][0];
    acc.ay = simData[simIndex][1];
    acc.az = simData[simIndex][2];
    acc.gx = simData[simIndex][3];
    acc.gy = simData[simIndex][4];
    acc.gz = simData[simIndex][5];

    simIndex = (simIndex + 1) % 10;

    return true;
}

/**
 * 发送AccEvent和StepEvent到BLE
 * 格式：JSON字符串
 */
void sendIMUData(const AccEvent& acc, const StepEvent& step) {
    cJSON* root = cJSON_CreateObject();

    // 添加时间戳
    cJSON_AddNumberToObject(root, "ts", acc.timestamp);

    // 添加原始IMU数据
    cJSON* imu = cJSON_CreateObject();
    cJSON_AddNumberToObject(imu, "ax", acc.ax);
    cJSON_AddNumberToObject(imu, "ay", acc.ay);
    cJSON_AddNumberToObject(imu, "az", acc.az);
    cJSON_AddNumberToObject(imu, "gx", acc.gx);
    cJSON_AddNumberToObject(imu, "gy", acc.gy);
    cJSON_AddNumberToObject(imu, "gz", acc.gz);
    cJSON_AddItemToObject(root, "imu", imu);

    // 添加滤波后的数据（用于调试）
    cJSON* filtered = cJSON_CreateObject();
    cJSON_AddNumberToObject(filtered, "ay", step.filtered_ay);
    cJSON_AddNumberToObject(filtered, "az", step.filtered_az);
    cJSON_AddNumberToObject(filtered, "gy", step.filtered_gy);
    cJSON_AddItemToObject(root, "filtered", filtered);

    // 添加步态检测结果
    cJSON* gait = cJSON_CreateObject();
    cJSON_AddStringToObject(gait, "foot", step.isLeftFoot ? "LEFT" : "RIGHT");
    cJSON_AddNumberToObject(gait, "conf", step.confidence);
    cJSON_AddNumberToObject(gait, "valid", step.isStepValid ? 1 : 0);
    cJSON_AddNumberToObject(gait, "cadence", step.cadence);
    cJSON_AddItemToObject(root, "gait", gait);

    // 转换为JSON字符串
    char* jsonStr = cJSON_Print(root);

    // 发送BLE
    bleServer.sendString(jsonStr);

    // 同时输出到串口（方便调试）
    Serial.println(jsonStr);

    // 释放内存
    cJSON_Delete(root);
    free(jsonStr);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Server...");

    // 初始化BLE设备，设置设备名称
    NimBLEDevice::init("EMP32_IMU");

    // 设置BLE服务器
    bleServer.setup();

    Serial.println("System initialized!");
    Serial.println("Format: {\"ts\":timestamp,\"imu\":{\"ax\":x,\"ay\":y,...},\"gait\":{\"foot\":\"LEFT/RIGHT\",\"conf\":x,...}}");
}

void loop() {
    // 1. 读取IMU数据
    AccEvent acc;
    if (readIMUData(acc)) {
        // 2. 检测步态
        StepEvent step = detector.detectStep(acc);

        // 3. 发送数据到BLE和串口
        sendIMUData(acc, step);

        // 4. 如果检测到有效步态，打印详细信息
        if (step.isStepValid) {
            Serial.print("Step detected: ");
            Serial.print(step.isLeftFoot ? "LEFT" : "RIGHT");
            Serial.print(" | Confidence: ");
            Serial.print(step.confidence, 2);
            Serial.print(" | Cadence: ");
            Serial.print(step.cadence, 1);
            Serial.println(" spm");
        }
    }

    // 采样周期：10ms（100Hz采样率）
    delay(10);
}
