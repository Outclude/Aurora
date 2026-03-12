#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "servo.h"
#include <string.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "MPU6050.h"
#include "LeftRightDetector.h"

#define I2C_SCL_PIN 18
#define I2C_SDA_PIN 19
#define SERVO_PIN 23

#define SERVO_FREQ 50
#define SERVO_RES 16
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_CH 0

QueueHandle_t accQueue;
QueueHandle_t stepQueue;

LeftRightDetector detector;

typedef struct
{
  float roll, pitch, yaw;
  float ax, ay, az;
  float gx, gy, gz;
} IMU_Data_t;

uint32_t dutyMin;
uint32_t dutyMax;

int angle = 0;

void TaskBlink1(void *pvParameters);
void TaskBlink2(void *pvParameters);
void TaskBlink3(void *pvParameters);
void TaskBlink4(void *pvParameters);
void TaskStepDetect(void *pvParameters);
MyBLEServer bleServer;

void setup()
{

  Serial.begin(115200);
  delay(50);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(50);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(50);

  ledcSetup(0, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, 0);

  dutyMin = (uint32_t)((SERVO_MIN_US / 20000.0) * ((1 << SERVO_RES) - 1));
  dutyMax = (uint32_t)((SERVO_MAX_US / 20000.0) * ((1 << SERVO_RES) - 1));

  // 上电回中
  uint32_t dutyMid = (dutyMin + dutyMax) / 2;
  ledcWrite(0, (dutyMin + dutyMax) / 2);

  // 初始化BLE设备，设置设备名称
  NimBLEDevice::init("lxy");

  // 设置BLE服务器
  bleServer.setup();
  delay(1000);

  MPU6050_Init();
  delay(50);

  accQueue = xQueueCreate(10, sizeof(AccEvent));
  stepQueue = xQueueCreate(10, sizeof(StepEvent));

  Serial.println("System is ready.");

  xTaskCreatePinnedToCore(TaskBlink1, "TaskBlink1", 1024, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(TaskBlink2, "TaskBlink2", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskBlink3, "TaskBlink3", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskBlink4, "TaskBlink4", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskStepDetect, "StepDetect", 4096, NULL, 2, NULL, 0);
}

void loop()
{
  vTaskDelay(10000 / portTICK_PERIOD_MS);
}

void TaskBlink1(void *pvParameters)
{
  while (true)
  {
    digitalWrite(2, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(2, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskBlink2(void *pvParameters)
{
  uint32_t lastTick = xTaskGetTickCount();

  while (true)
  {
    uint32_t nowTick = xTaskGetTickCount();
    float dt = (nowTick - lastTick) * portTICK_PERIOD_MS / 1000.0f;
    lastTick = nowTick;

    MPU6050_Update(dt);
    MPU6050_Raw_t raw = MPU6050_GetRaw();

    AccEvent acc;
    acc.accX = raw.ax * 9.8f;
    acc.accY = raw.ay * 9.8f;
    acc.accZ = raw.az * 9.8f;

    acc.omegaP = raw.gx * DEG_TO_RAD;
    acc.omegaQ = raw.gy * DEG_TO_RAD;
    acc.omegaR = raw.gz * DEG_TO_RAD;

    xQueueSend(accQueue, &acc, 0);

    vTaskDelay(pdMS_TO_TICKS(5)); // 200 Hz
  }
}

void TaskBlink3(void *pvParameters)
{
  while (true)
  {
    for (int angle = 0; angle <= 180; angle += 5)
    {
      uint32_t duty = map(angle, 0, 180, dutyMin, dutyMax);
      ledcWrite(SERVO_CH, duty);
      vTaskDelay(pdMS_TO_TICKS(40));
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    for (int angle = 180; angle >= 0; angle -= 5)
    {
      uint32_t duty = map(angle, 0, 180, dutyMin, dutyMax);
      ledcWrite(SERVO_CH, duty);
      vTaskDelay(pdMS_TO_TICKS(40));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskBlink4(void *pvParameters)
{
  StepEvent ev;

  while (true)
  {
    if (xQueueReceive(stepQueue, &ev, portMAX_DELAY))
    {
      char buf[64];
      snprintf(buf, sizeof(buf),
               "STEP:%d,conf=%d,cad=%.1f",
               ev.isLeftFoot ? 1 : 0,
               (int)(ev.confidence * 100),
               ev.cadence);
      bleServer.sendString(buf);
    }
  }
}

void TaskStepDetect(void *pvParameters)
{
  AccEvent acc;

  while (true)
  {
    if (xQueueReceive(accQueue, &acc, portMAX_DELAY))
    {
      StepEvent ev = detector.detectStep(acc);

      if (ev.isStepValid)
      {
        ev.timestamp = millis();
        xQueueSend(stepQueue, &ev, 0);
      }
    }
  }
}
