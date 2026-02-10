/**
 * @file arduino_test_imu_gps.ino
 * @brief Arduino IMU + GPS 传感器测试程序（纯净版）
 *
 * 这个文件只包含 Arduino 需要的代码
 * 不包含 Python 绑定或其他平台相关的代码
 */

#include <Arduino.h>

// ==================== 配置参数 ====================
#define SERIAL_BAUD 115200
#define SEND_INTERVAL_MS 50  // 发送间隔（毫秒）

// ==================== 启用真实传感器 ====================
#define USE_REAL_IMU
#define USE_REAL_GPS  // 如果没有 GPS 模块，注释掉这一行

// ==================== 传感器库引入 ====================
#ifdef USE_REAL_IMU
  // 根据你的传感器选择合适的库
  #include <Wire.h>
  // #include <MPU6050.h>
  // #include <BMI160.h>
  // 取消你使用的传感器库的注释
#endif

#ifdef USE_REAL_GPS
  #include <TinyGPS++.h>
  // 或者其他 GPS 库
#endif

// ==================== 数据结构 ====================
struct IMURawData {
    float ax, ay, az;  // 加速度 (g)
    float gx, gy, gz;  // 角速度 (deg/s)
    unsigned long timestamp;
};

struct GPSRawData {
    double latitude;   // 纬度
    double longitude;  // 经度
    int satellites;    // 卫星数量
    unsigned long timestamp;
    bool valid;
};

// ==================== 全局变量 ====================
unsigned long lastSendTime = 0;

#ifdef USE_REAL_IMU
  // TODO: 声明你的传感器对象
  // MPU6050 mpu;
  // BMI160 bmi;
#endif

#ifdef USE_REAL_GPS
  TinyGPSPlus gps;
  // 如果使用硬件串口1
  // #define GPS_SERIAL Serial1
#endif

// ==================== 函数声明 ====================
bool readIMUData(IMURawData& imu);
bool readGPSData(GPSRawData& gps);
void sendIMUData(const IMURawData& imu);
void sendGPSData(const GPSRawData& gps);

// ==================== 初始化 ====================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println(" Arduino IMU + GPS 传感器测试");
    Serial.println("========================================\n");

#ifdef USE_REAL_IMU
    Serial.println("初始化 IMU 传感器...");
    Wire.begin();

    // TODO: 初始化你的 IMU 传感器
    // 示例：MPU6050
    // mpu.initialize();
    // if (mpu.testConnection()) {
    //     Serial.println("✓ MPU6050 连接成功");
    // } else {
    //     Serial.println("✗ MPU6050 连接失败");
    // }

    Serial.println("✓ IMU 初始化完成\n");
#else
    Serial.println("模式: 模拟 IMU 数据\n");
#endif

#ifdef USE_REAL_GPS
    Serial.println("初始化 GPS 模块...");

    // TODO: 初始化 GPS 串口
    // GPS_SERIAL.begin(9600);

    Serial.println("✓ GPS 初始化完成\n");
#else
    Serial.println("模式: 模拟 GPS 数据\n");
#endif

    Serial.println("========================================");
    Serial.println("开始发送数据...");
    Serial.println("数据格式:");
    Serial.println("  IMU: IMU:ax=x ay=y az=z gx=x gy=y gz=z");
    Serial.println("  GPS: GPS:lat=x lng=y sat=z");
    Serial.println("========================================\n");
}

// ==================== 主循环 ====================
void loop() {
    unsigned long currentTime = millis();

    // 定时发送数据
    if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = currentTime;

        // 1. 读取并发送 IMU 数据
        IMURawData imu;
        if (readIMUData(imu)) {
            sendIMUData(imu);
        }

        // 2. 读取并发送 GPS 数据
        GPSRawData gps;
        if (readGPSData(gps)) {
            sendGPSData(gps);
        }
    }

#ifdef USE_REAL_GPS
    // 持续读取 GPS 数据（使用硬件串口）
    // while (GPS_SERIAL.available() > 0) {
    //     gps.encode(GPS_SERIAL.read());
    // }
#endif
}

// ==================== IMU 数据读取 ====================
bool readIMUData(IMURawData& imu) {
#ifdef USE_REAL_IMU
    // TODO: 从你的 IMU 传感器读取数据
    // 示例：MPU6050
    // int16_t ax, ay, az, gx, gy, gz;
    // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // imu.ax = ax / 16384.0;  // 转换为 g
    // imu.ay = ay / 16384.0;
    // imu.az = az / 16384.0;
    // imu.gx = gx / 131.0;    // 转换为 deg/s
    // imu.gy = gy / 131.0;
    // imu.gz = gz / 131.0;
    // imu.timestamp = millis();

    // return true;

    // 临时返回 false，需要你实现真实的读取
    return false;

#else
    // ========== 模拟数据 ==========
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

        // 添加一些跑步时的数据（更大冲击）
        {-10.5f, 0.8f, 1.5f, 0.2f, 0.5f, -0.1f},
        {-9.8f, 0.3f, 0.8f, 0.1f, 0.3f, -0.05f},
        {-11.2f, 0.5f, 2.1f, 0.3f, 0.6f, -0.15f},
        {-9.5f, 0.4f, 0.6f, 0.08f, 0.25f, -0.03f},
    };

    imu.ax = simData[simIndex][0];
    imu.ay = simData[simIndex][1];
    imu.az = simData[simIndex][2];
    imu.gx = simData[simIndex][3];
    imu.gy = simData[simIndex][4];
    imu.gz = simData[simIndex][5];
    imu.timestamp = millis();

    simIndex = (simIndex + 1) % (sizeof(simData) / sizeof(simData[0]));

    // 添加一些随机噪声
    imu.ax += (float)random(-100, 100) / 10000.0f;
    imu.ay += (float)random(-100, 100) / 10000.0f;
    imu.az += (float)random(-100, 100) / 10000.0f;
    imu.gx += (float)random(-50, 50) / 1000.0f;
    imu.gy += (float)random(-50, 50) / 1000.0f;
    imu.gz += (float)random(-50, 50) / 1000.0f;

    return true;
#endif
}

// ==================== GPS 数据读取 ====================
bool readGPSData(GPSRawData& gps) {
#ifdef USE_REAL_GPS
    // TODO: 从你的 GPS 模块读取数据
    // 示例：TinyGPS++
    // if (gps.location.isValid()) {
    //     gps.latitude = gps.location.lat();
    //     gps.longitude = gps.location.lng();
    //     gps.satellites = gps.satellites.value();
    //     gps.timestamp = millis();
    //     gps.valid = (gps.satellites.value() >= 4);
    //     return true;
    // }

    // 临时返回 false，需要你实现真实的读取
    return false;

#else
    // ========== 模拟 GPS 数据 ==========
    static double baseLat = 39.9042;  // 北京天安门
    static double baseLng = 116.4074;
    static int stepCount = 0;

    // 模拟行走路线（每100步更新一次位置）
    if (stepCount % 100 == 0) {
        baseLng += 0.00005;
        baseLat += 0.00003;
    }

    gps.latitude = baseLat + (float)random(-50, 50) / 1000000.0;
    gps.longitude = baseLng + (float)random(-50, 50) / 1000000.0;
    gps.satellites = 8 + random(-2, 2);
    gps.timestamp = millis();
    gps.valid = (gps.satellites >= 4);

    stepCount++;

    return true;
#endif
}

// ==================== 数据发送 ====================
void sendIMUData(const IMURawData& imu) {
    Serial.print("IMU:");
    Serial.print("ax="); Serial.print(imu.ax, 4);
    Serial.print(" ay="); Serial.print(imu.ay, 4);
    Serial.print(" az="); Serial.print(imu.az, 4);
    Serial.print(" gx="); Serial.print(imu.gx, 4);
    Serial.print(" gy="); Serial.print(imu.gy, 4);
    Serial.print(" gz="); Serial.println(imu.gz, 4);
}

void sendGPSData(const GPSRawData& gps) {
    Serial.print("GPS:");
    Serial.print("lat="); Serial.print(gps.latitude, 6);
    Serial.print(" lng="); Serial.print(gps.longitude, 6);
    Serial.print(" sat="); Serial.println(gps.satellites);
}
