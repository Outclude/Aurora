/*
 * IMU和灯光控制测试用例 (Arduino版本)
 *
 * 功能：验证数据流程 IMU → LeftRightDetector → Vibrate → LightEvent
 *
 * 测试内容：
 * 1. AccEvent 数据能否正常传入
 * 2. detectStep 能否正确输出 StepEvent
 * 3. compute_freq 能否正确输出 LightEvent
 * 4. 整个数据链路是否通畅
 *
 * 使用方法：
 * 1. 上传到ESP32开发板
 * 2. 打开串口监视器 (波特率 115200)
 * 3. 观察测试结果
 */

#include <Arduino.h>
#include "LeftRightDetector.h"
#include "vibrate.h"

// 创建检测器和控制器实例
LeftRightDetector detector;
Vibrate vibrate;

// 测试结果统计
int totalTests = 0;
int passedTests = 0;

// 辅助函数：格式化浮点数到字符串
void formatFloat(char* buf, float val, int width, int precision) {
    snprintf(buf, 20, "%*.*f", width, precision, val);
}

// 打印AccEvent
void printAccEvent(const AccEvent& acc) {
    Serial.println("  AccEvent: {");
    Serial.print("    accX="); Serial.print(acc.accX); Serial.println(" (前进方向)");
    Serial.print("    accY="); Serial.print(acc.accY); Serial.println(" (横向)");
    Serial.print("    accZ="); Serial.print(acc.accZ); Serial.println(" (垂直)");
    Serial.print("    omegaP="); Serial.println(acc.omegaP);
    Serial.print("    omegaQ="); Serial.println(acc.omegaQ);
    Serial.print("    omegaR="); Serial.println(acc.omegaR);
    Serial.println("  }");
}

// 打印StepEvent
void printStepEvent(const StepEvent& step) {
    Serial.println("  StepEvent: {");
    Serial.print("    isStepValid=");
    Serial.println(step.isStepValid ? "true" : "false");

    if (step.isStepValid) {
        Serial.print("    isLeftFoot=");
        Serial.println(step.isLeftFoot ? "左脚在前" : "右脚在前");

        char buf[20];
        Serial.print("    confidence=");
        formatFloat(buf, step.confidence, 4, 2);
        Serial.println(buf);

        Serial.print("    cadence=");
        formatFloat(buf, step.cadence, 5, 1);
        Serial.print(buf);
        Serial.println(" 步/分钟");
    }
    Serial.println("  }");
}

// 打印LightEvent
void printLightEvent(const LightEvent& light) {
    Serial.println("  LightEvent: {");

    if (light.isValid) {
        Serial.println("    isValid=true");
        Serial.print("    leftFirst=");
        Serial.println(light.leftFirst ? "是" : "否");
        Serial.print("    l_time=");
        Serial.print(light.l_time);
        Serial.println(" ms");
        Serial.print("    r_time=");
        Serial.print(light.r_time);
        Serial.println(" ms");
        Serial.print("    interval=");
        Serial.print(light.interval);
        Serial.println(" ms");
    } else {
        Serial.println("    isValid=false (无有效步态)");
    }
    Serial.println("  }");
}

// 测试用例1：左脚着地场景
void testCase_LeftFootLanding() {
    Serial.println("\n========== 测试用例1：左脚着地 ==========");
    Serial.println("场景：跑步时左脚着地，身体重心向右转移，上身向右扭转");

    // 模拟左脚着地时的IMU数据
    AccEvent acc;
    acc.accX = 2.0f;      // 前进方向有加速度
    acc.accY = 1.5f;      // 横向向右加速度（左脚着地时重心右移）
    acc.accZ = 18.0f;     // 垂直方向峰值（着地冲击）
    acc.omegaP = 0.1f;    // roll角速度（较小）
    acc.omegaQ = 0.2f;    // pitch角速度
    acc.omegaR = 0.8f;    // yaw角速度为正（上身右转）

    Serial.println("\n【输入】");
    printAccEvent(acc);

    // 检测步态
    StepEvent stepEvent = detector.detectStep(acc);
    Serial.println("\n【StepEvent 输出】");
    printStepEvent(stepEvent);

    // 验证StepEvent
    Serial.println("\n【验证 StepEvent】");
    totalTests++;

    if (stepEvent.isStepValid) {
        if (stepEvent.isLeftFoot) {
            Serial.println("  ✓ 正确：检测到左脚在前");
            passedTests++;
        } else {
            Serial.println("  ✗ 错误：应检测到左脚在前，但检测到右脚");
        }

        if (stepEvent.confidence > 0.5f) {
            Serial.print("  ✓ 置信度良好：");
            Serial.println(stepEvent.confidence);
        } else {
            Serial.print("  ! 置信度较低：");
            Serial.println(stepEvent.confidence);
        }
    } else {
        Serial.println("  ✗ 未检测到有效步态");
    }

    // 计算灯光控制
    LightEvent lightEvent = vibrate.compute_freq(stepEvent);
    Serial.println("\n【LightEvent 输出】");
    printLightEvent(lightEvent);

    // 验证LightEvent
    Serial.println("\n【验证 LightEvent】");
    totalTests++;

    if (lightEvent.isValid) {
        if (!lightEvent.leftFirst) {
            Serial.println("  ✓ 正确：左脚在前时，右灯先亮");
            passedTests++;
        } else {
            Serial.println("  ✗ 错误：左脚在前时，应右灯先亮");
        }
        Serial.print("  ✓ 右灯时间：");
        Serial.print(lightEvent.r_time);
        Serial.println(" ms");
        Serial.print("  ✓ 左灯时间：");
        Serial.print(lightEvent.l_time);
        Serial.println(" ms");
        Serial.print("  ✓ 间隔时间：");
        Serial.print(lightEvent.interval);
        Serial.println(" ms");
    } else {
        Serial.println("  ✗ LightEvent 无效");
    }

    delay(1000);
}

// 测试用例2：右脚着地场景
void testCase_RightFootLanding() {
    Serial.println("\n========== 测试用例2：右脚着地 ==========");
    Serial.println("场景：跑步时右脚着地，身体重心向左转移，上身向左扭转");

    // 模拟右脚着地时的IMU数据
    AccEvent acc;
    acc.accX = 2.0f;      // 前进方向有加速度
    acc.accY = -1.5f;     // 横向向左加速度（右脚着地时重心左移）
    acc.accZ = 18.0f;     // 垂直方向峰值（着地冲击）
    acc.omegaP = -0.1f;   // roll角速度
    acc.omegaQ = 0.2f;    // pitch角速度
    acc.omegaR = -0.8f;   // yaw角速度为负（上身左转）

    Serial.println("\n【输入】");
    printAccEvent(acc);

    // 检测步态
    StepEvent stepEvent = detector.detectStep(acc);
    Serial.println("\n【StepEvent 输出】");
    printStepEvent(stepEvent);

    // 验证StepEvent
    Serial.println("\n【验证 StepEvent】");
    totalTests++;

    if (stepEvent.isStepValid) {
        if (!stepEvent.isLeftFoot) {
            Serial.println("  ✓ 正确：检测到右脚在前");
            passedTests++;
        } else {
            Serial.println("  ✗ 错误：应检测到右脚在前，但检测到左脚");
        }

        if (stepEvent.confidence > 0.5f) {
            Serial.print("  ✓ 置信度良好：");
            Serial.println(stepEvent.confidence);
        } else {
            Serial.print("  ! 置信度较低：");
            Serial.println(stepEvent.confidence);
        }
    } else {
        Serial.println("  ✗ 未检测到有效步态");
    }

    // 计算灯光控制
    LightEvent lightEvent = vibrate.compute_freq(stepEvent);
    Serial.println("\n【LightEvent 输出】");
    printLightEvent(lightEvent);

    // 验证LightEvent
    Serial.println("\n【验证 LightEvent】");
    totalTests++;

    if (lightEvent.isValid) {
        if (lightEvent.leftFirst) {
            Serial.println("  ✓ 正确：右脚在前时，左灯先亮");
            passedTests++;
        } else {
            Serial.println("  ✗ 错误：右脚在前时，应左灯先亮");
        }
        Serial.print("  ✓ 左灯时间：");
        Serial.print(lightEvent.l_time);
        Serial.println(" ms");
        Serial.print("  ✓ 右灯时间：");
        Serial.print(lightEvent.r_time);
        Serial.println(" ms");
        Serial.print("  ✓ 间隔时间：");
        Serial.print(lightEvent.interval);
        Serial.println(" ms");
    } else {
        Serial.println("  ✗ LightEvent 无效");
    }

    delay(1000);
}

// 测试用例3：无步态场景
void testCase_NoStep() {
    Serial.println("\n========== 测试用例3：无步态（空中阶段） ==========");
    Serial.println("场景：跑步时的空中悬停阶段，没有着地冲击");

    // 模拟空中阶段的IMU数据
    AccEvent acc;
    acc.accX = 0.5f;      // 前进方向加速度较小
    acc.accY = 0.2f;      // 横向加速度很小
    acc.accZ = 9.8f;      // 仅重力，无着地冲击
    acc.omegaP = 0.05f;
    acc.omegaQ = 0.1f;
    acc.omegaR = 0.1f;

    Serial.println("\n【输入】");
    printAccEvent(acc);

    // 检测步态
    StepEvent stepEvent = detector.detectStep(acc);
    Serial.println("\n【StepEvent 输出】");
    printStepEvent(stepEvent);

    // 验证StepEvent
    Serial.println("\n【验证 StepEvent】");
    totalTests++;

    if (!stepEvent.isStepValid) {
        Serial.println("  ✓ 正确：未检测到有效步态（符合空中阶段预期）");
        passedTests++;
    } else {
        Serial.println("  ! 检测到步态（可能是噪声）");
    }

    // 计算灯光控制
    LightEvent lightEvent = vibrate.compute_freq(stepEvent);
    Serial.println("\n【LightEvent 输出】");
    printLightEvent(lightEvent);

    // 验证LightEvent
    Serial.println("\n【验证 LightEvent】");
    totalTests++;

    if (!lightEvent.isValid) {
        Serial.println("  ✓ 正确：无有效LightEvent（符合预期）");
        passedTests++;
    } else {
        Serial.println("  ! LightEvent有效（可能使用了默认值）");
    }

    delay(1000);
}

// 测试用例4：连续步态序列
void testCase_ContinuousSteps() {
    Serial.println("\n========== 测试用例4：连续步态序列 ==========");
    Serial.println("场景：模拟一个完整的跑步周期（左-右-左-右）");

    int stepCount = 0;
    int correctDetection = 0;

    // 模拟4步：左-右-左-右
    bool expectedSequence[] = {true, false, true, false};
    float accY_values[] = {1.5f, -1.5f, 1.5f, -1.5f};
    float omegaR_values[] = {0.8f, -0.8f, 0.8f, -0.8f};

    for (int i = 0; i < 4; i++) {
        Serial.println("\n--- 第 ");
        Serial.print(i + 1);
        Serial.print(" 步 (预期: ");
        Serial.print(expectedSequence[i] ? "左脚" : "右脚");
        Serial.println(") ---");

        AccEvent acc;
        acc.accX = 2.0f;
        acc.accY = accY_values[i];
        acc.accZ = 18.0f;
        acc.omegaP = 0.1f;
        acc.omegaQ = 0.2f;
        acc.omegaR = omegaR_values[i];

        // 添加模拟时间戳
        acc.omegaP = 400.0f * (i + 1);

        StepEvent stepEvent = detector.detectStep(acc);
        LightEvent lightEvent = vibrate.compute_freq(stepEvent);

        if (stepEvent.isStepValid) {
            stepCount++;
            totalTests++;

            Serial.print("  检测结果: ");
            Serial.print(stepEvent.isLeftFoot ? "左脚" : "右脚");
            Serial.print(", 置信度: ");
            Serial.print(stepEvent.confidence);
            Serial.print(", 灯光: ");
            Serial.println(lightEvent.leftFirst ? "左灯先" : "右灯先");

            if (stepEvent.isLeftFoot == expectedSequence[i]) {
                correctDetection++;
                passedTests++;
                Serial.println("  ✓ 正确");
            } else {
                Serial.println("  ✗ 错误");
            }
        } else {
            Serial.println("  ! 未检测到步态");
        }

        delay(500);
    }

    Serial.println("\n【统计结果】");
    Serial.print("  检测步数: ");
    Serial.print(stepCount);
    Serial.println(" / 4");
    Serial.print("  正确率: ");
    Serial.print(correctDetection * 100 / 4);
    Serial.println("%");
}

// 打印测试总结
void printSummary() {
    Serial.println("\n========================================");
    Serial.println("         测试总结");
    Serial.println("========================================");
    Serial.print("总测试项: ");
    Serial.println(totalTests);
    Serial.print("通过: ");
    Serial.println(passedTests);
    Serial.print("失败: ");
    Serial.println(totalTests - passedTests);
    Serial.print("通过率: ");
    Serial.print(passedTests * 100 / totalTests);
    Serial.println("%");
    Serial.println("========================================");
}

// 控制引脚定义（根据实际硬件连接修改）
#define LEFT_LIGHT_PIN  25  // 左灯控制引脚
#define RIGHT_LIGHT_PIN 26  // 右灯控制引脚

// 实际应用示例（从真实IMU读取数据）
void realIMUExample() {
    Serial.println("\n========== 实际IMU数据示例 ==========");
    Serial.println("（需要连接真实IMU硬件）");

    // TODO: 从实际IMU读取数据
    // AccEvent acc;
    // acc.accX = readIMUAccX();
    // acc.accY = readIMUAccY();
    // acc.accZ = readIMUAccZ();
    // acc.omegaP = readIMUGyroX();
    // acc.omegaQ = readIMUGyroY();
    // acc.omegaR = readIMUGyroZ();
    //
    // StepEvent stepEvent = detector.detectStep(acc);
    // LightEvent lightEvent = vibrate.compute_freq(stepEvent);
    //
    // if (lightEvent.isValid) {
    //     if (lightEvent.leftFirst) {
    //         // 左灯先亮
    //         digitalWrite(LEFT_LIGHT_PIN, HIGH);
    //         delay(lightEvent.l_time);
    //         digitalWrite(LEFT_LIGHT_PIN, LOW);
    //         delay(lightEvent.interval);
    //         digitalWrite(RIGHT_LIGHT_PIN, HIGH);
    //         delay(lightEvent.r_time);
    //         digitalWrite(RIGHT_LIGHT_PIN, LOW);
    //     } else {
    //         // 右灯先亮
    //         digitalWrite(RIGHT_LIGHT_PIN, HIGH);
    //         delay(lightEvent.r_time);
    //         digitalWrite(RIGHT_LIGHT_PIN, LOW);
    //         delay(lightEvent.interval);
    //         digitalWrite(LEFT_LIGHT_PIN, HIGH);
    //         delay(lightEvent.l_time);
    //         digitalWrite(LEFT_LIGHT_PIN, LOW);
    //     }
    // }

    Serial.println("（待实现：连接IMU硬件后取消注释上述代码）");
}

void setup() {
    // 初始化串口
    Serial.begin(115200);
    delay(1000);

    Serial.println("========================================");
    Serial.println("   IMU 和灯光控制测试程序");
    Serial.println("   Arduino ESP32 版本");
    Serial.println("========================================");

    // 初始化LED引脚
    pinMode(LEFT_LIGHT_PIN, OUTPUT);
    pinMode(RIGHT_LIGHT_PIN, OUTPUT);
    digitalWrite(LEFT_LIGHT_PIN, LOW);
    digitalWrite(RIGHT_LIGHT_PIN, LOW);

    // 重置检测器和控制器
    detector.reset();
    vibrate.reset();

    delay(500);
}

bool testsCompleted = false;

void loop() {
    // 只运行一次测试
    if (!testsCompleted) {
        // 运行所有测试用例
        testCase_LeftFootLanding();
        testCase_RightFootLanding();
        testCase_NoStep();
        testCase_ContinuousSteps();

        // 显示测试示例
        realIMUExample();

        // 打印总结
        printSummary();

        testsCompleted = true;

        Serial.println("\n测试完成！");
        Serial.println("可重启设备重新运行测试，或连接真实IMU进行实际测试");
    }

    delay(1000);
}
