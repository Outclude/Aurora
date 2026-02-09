#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "GPS.h"
#include "servo.h"
#include "MPU6050.h"
#include "ble_server.h"
#include "WiFiMulti.h"
#include "audio_i2s.h"
#include <math.h>
#include "audio_i2s.h"
#include "audio_data.h"
#include <Arduino.h>
#include <ArduinoJson.h>

#define PI 3.1415926
#define SAMPLE_RATE 8000
#define WAV_HEADER_SIZE 44
#define I2S_CHUNK 256

static int16_t audio_buf[I2S_CHUNK];

void TaskBlink1(void *pvParameters);
void TaskBlink2(void *pvParameters);
void TaskBlink3(void *pvParameters);
void TaskBlink4(void *pvParameters);
void TaskBlink5(void *pvParameters);
void TaskBlink6(void *pvParameters);
void TaskBlink7(void *pvParameters);
void TaskBlink8(void *pvParameters);
void TaskBlink9(void *pvParameters);
void TaskBlink10(void *pvParameters);
void play_wav_array(const unsigned char *wav, unsigned int wav_len);

MyBLEServer bleServer;
volatile int g_servo_angle = 90;
volatile bool g_light_enable = false;
volatile bool g_keep_running = false;
volatile bool g_game_state = false;
volatile int g_voice_value = 1;
volatile uint32_t g_uptime_sec = 0;
volatile int shan_shuo_jian_ge = 100; // ms
volatile double yu_she_pei_su = 3.00; // m/s

volatile int game_sumDistance = 1000;      // m
volatile int game_rewardDistance = 80;     // m
volatile double game_yu_she_pei_su = 3.00; // m/s

typedef struct
{
  uint32_t total_distance_m; // 本次跑步总路程（米，整数）
  uint32_t total_time_s;     // 本次跑步总时间（秒，整数）
  float avg_speed_mps;       // 平均配速（m/s，保留 2 位小数）
} RunStats_t;

volatile RunStats_t g_run_stats = {
    .total_distance_m = 0,
    .total_time_s = 0,
    .avg_speed_mps = 0.00f};

void setup()
{
  Serial.begin(115200);
  delay(100);
  pinMode(2, OUTPUT);
  pinMode(21, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(21, HIGH);
  delay(100);
  NimBLEDevice::init("lxy2005");
  bleServer.setup();
  delay(500);
  MPU6050_Init();
  delay(100);
  GPS_Init();
  delay(100);
  Servo_Init();
  delay(100);
  audio_i2s_init();
  delay(100);

  Serial.println("System is ready.");

  xTaskCreatePinnedToCore(TaskBlink1, "TaskBlink1", 1024, NULL, 0, NULL, 0);   // LED on board
  xTaskCreatePinnedToCore(TaskBlink2, "TaskBlink2", 8192, NULL, 1, NULL, 0);   // servo control
  xTaskCreatePinnedToCore(TaskBlink3, "TaskBlink3", 4096, NULL, 1, NULL, 0);   // MPU6050 read
  xTaskCreatePinnedToCore(TaskBlink4, "TaskBlink4", 8192, NULL, 2, NULL, 0);   // GPS read
  xTaskCreatePinnedToCore(TaskBlink5, "TaskBlink5", 4096, NULL, 1, NULL, 1);   // BLE send
  xTaskCreatePinnedToCore(TaskBlink6, "TaskBlink6", 4096, NULL, 1, NULL, 1);   // BLE read
  xTaskCreatePinnedToCore(TaskBlink7, "TaskBlink7", 4096, NULL, 1, NULL, 1);   // light control
  xTaskCreatePinnedToCore(TaskBlink8, "TaskBlink8", 4096, NULL, 1, NULL, 0);   // Serial print
  xTaskCreatePinnedToCore(TaskBlink9, "TaskBlink9", 4096, NULL, 1, NULL, 1);   // audio output
  xTaskCreatePinnedToCore(TaskBlink10, "TaskBlink10", 4096, NULL, 1, NULL, 1); // uptime counter
}

void loop()
{

  vTaskDelay(pdMS_TO_TICKS(1000));
}

void TaskBlink1(void *pvParameters)
{
  while (true)
  {
    digitalWrite(2, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(2, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskBlink2(void *pvParameters)
{
  int last_angle = -1;

  while (true)
  {
    int target = g_servo_angle;

    /* -------- 限幅 -------- */
    if (target < 0)
      target = 0;
    if (target > 180)
      target = 180;

    /* -------- 角度变化才执行 -------- */
    if (target != last_angle)
    {
      Servo_SetAngle(target);
      last_angle = target;
    }

    /* -------- 定时检测周期 -------- */
    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}

void TaskBlink3(void *pvParameters)
{
  while (true)
  {
    MPU6050_Read();
    // Serial.printf("ax=%.4f ay=%.4f az=%.4f | gx=%.4f gy=%.4f gz=%.4f\r\n", mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz);
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
  }
}

void TaskBlink4(void *pvParameters)
{
  while (true)
  {
    GPS_Task();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskBlink5(void *pvParameters)
{
  char buf[64];

  while (true)
  {
    snprintf(buf, sizeof(buf), "J%.8f\n", gps_lng);
    bleServer.sendString(buf);

    snprintf(buf, sizeof(buf), "W%.8f\n", gps_lat);
    bleServer.sendString(buf);

    snprintf(buf, sizeof(buf), "N%d\n", gps_sat);
    bleServer.sendString(buf);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskBlink6(void *pvParameters)
{
  char rxBuf[BLE_RX_MAX_LEN];

  while (true)
  {
    if (bleServer.recvString(rxBuf, sizeof(rxBuf), portMAX_DELAY))
    {
      Serial.print("[BLE RX] ");
      Serial.println(rxBuf);

      /* ========= JSON 解析 ========= */
      StaticJsonDocument<256> doc; // 根据数据大小调整
      DeserializationError err = deserializeJson(doc, rxBuf);

      if (err)
      {
        Serial.print("[JSON ERROR] ");
        Serial.println(err.c_str());
        continue;
      }

      /* ========= 读取 type ========= */
      if (!doc.containsKey("type"))
      {
        Serial.println("[JSON ERROR] no type field");
        continue;
      }

      int type = doc["type"];
      Serial.printf("Parsed type = %d\r\n", type);

      /* ========= 按 type 分发 ========= */
      switch (type)
      {
      case 0: // 设置配速模式目标
      {
        int cadence = doc["cadence"] | 0;
        if (cadence <= 0)
        {
          shan_shuo_jian_ge = 500;
        }
        else
        {
          shan_shuo_jian_ge = (int)(60000 / cadence); // 步频转间隔，单位 ms
        }

        int pace_sec = doc["pace_sec"] | 0;
        if (pace_sec > 0)
        {
          yu_she_pei_su = 1000.0 / pace_sec;
        }
        else
        {
          yu_she_pei_su = 3.00;
        }
        break;
      }

      case 1: // 开始跑步
      {
        g_keep_running = true;
        StaticJsonDocument<64> okDoc;
        okDoc["msg"] = "Status_OK";
        char okBuf[64];
        serializeJson(okDoc, okBuf, sizeof(okBuf));
        bleServer.sendString(okBuf);
        break;
      }

      case 2: // 暂停跑步
      {
        g_keep_running = false;
        StaticJsonDocument<64> okDoc;
        okDoc["msg"] = "Status_OK";
        char okBuf[64];
        serializeJson(okDoc, okBuf, sizeof(okBuf));
        bleServer.sendString(okBuf);
        break;
      }

      case 3: // 继续跑步
      {
        g_keep_running = true;
        StaticJsonDocument<64> okDoc;
        okDoc["msg"] = "Status_OK";
        char okBuf[64];
        serializeJson(okDoc, okBuf, sizeof(okBuf));
        bleServer.sendString(okBuf);
        break;
      }

      case 4: // 结束跑步
      {
        g_keep_running = false;
        StaticJsonDocument<64> okDoc;
        okDoc["msg"] = "Status_OK";
        char okBuf[64];
        serializeJson(okDoc, okBuf, sizeof(okBuf));
        bleServer.sendString(okBuf);
        break;
      }

      case 5: // 设置游戏模式目标
      {
        int sumDistance = doc["sumDistance"] | 0;
        int rewardDistance = doc["rewardDistance"] | 0.0;
        game_sumDistance = sumDistance;
        game_rewardDistance = rewardDistance;
        if (sumDistance <= 0 || rewardDistance <= 0) {
          StaticJsonDocument<64> errDoc;
          errDoc["msg"] = "Update_Error";
          char errBuf[64];
          serializeJson(errDoc, errBuf, sizeof(errBuf));
          bleServer.sendString(errBuf);
        } else {
          StaticJsonDocument<64> okDoc;
          okDoc["msg"] = "Update_OK";
          char okBuf[64];
          serializeJson(okDoc, okBuf, sizeof(okBuf));
          bleServer.sendString(okBuf);
        }
        break;
      }

      default:
        break;
      }
    }
  }
}

void TaskBlink7(void *pvParameters)
{
  while (true)
  {
    if (g_light_enable)
    {
      digitalWrite(21, HIGH);
    }
    else
    {
      digitalWrite(21, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskBlink8(void *pvParameters)
{
  while (true)
  { /*
     Serial.printf(
         "ax=%.4f ay=%.4f az=%.4f | "
         "gx=%.4f gy=%.4f gz=%.4f | "
         "lat=%.8f lng=%.8f sat=%d\r\n",
         mpu_ax, mpu_ay, mpu_az,
         mpu_gx, mpu_gy, mpu_gz,
         gps_lat, gps_lng, gps_sat);*/

    Serial.printf(
        "[GAME] SumDist=%d m | RewardDist=%d m | TargetSpeed=%.2f m/s\r\n",
        game_sumDistance,
        game_rewardDistance,
        game_yu_she_pei_su);

    vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz 打印
  }
}

void TaskBlink9(void *pvParameters)
{
  while (true)
  {
    if (g_voice_value == 1)
    {
      play_wav_array(sound_data_0, sound_data_len_0);
      g_voice_value = 0;
    }
    if (g_voice_value == 2)
    {
      play_wav_array(sound_data_1, sound_data_len_1);
      g_voice_value = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 播完歇一下
  }
}

void TaskBlink10(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    g_uptime_sec++;
  }
}

void play_wav_array(const unsigned char *wav, unsigned int wav_len)
{
  unsigned int pos = WAV_HEADER_SIZE;

  while (pos + 3 < wav_len)
  {
    int n = 0;

    while (n < I2S_CHUNK && pos + 3 < wav_len)
    {
      // 左声道 16bit little-endian
      int16_t sample =
          (int16_t)(wav[pos] | (wav[pos + 1] << 8));

      audio_buf[n++] = sample;

      // 跳过一个立体声采样（L + R = 4 字节）
      pos += 4;
    }

    audio_i2s_play(audio_buf, n);
  }
}