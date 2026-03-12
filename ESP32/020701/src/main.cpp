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
static constexpr double EARTH_RADIUS = 6371000.0;
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
void TaskBlink11(void *pvParameters);
void TaskBlink12(void *pvParameters);
void TaskBlink13(void *pvParameters);
void play_wav_array(const unsigned char *wav, unsigned int wav_len);

double degToRad(double degrees)
{
  return degrees * PI / 180.0;
}

double gps_comopute(double lat1, double lon1, double lat2, double lon2)
{
  // 将角度转换为弧度
  double lat1_rad = degToRad(lat1);
  double lon1_rad = degToRad(lon1);
  double lat2_rad = degToRad(lat2);
  double lon2_rad = degToRad(lon2);

  // 计算差值
  double dlat = lat2_rad - lat1_rad;
  double dlon = lon2_rad - lon1_rad;

  // Haversine公式
  double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
             cos(lat1_rad) * cos(lat2_rad) *
                 sin(dlon / 2.0) * sin(dlon / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return EARTH_RADIUS * c;
}

MyBLEServer bleServer;

volatile int servo_middle_angle = 100; // 舵机中立位置，单位：度
volatile int servo_max_angle = 20;     // 舵机活动角度，单位：度

volatile int g_servo_angle = 100;
volatile bool g_light_enable = true;
volatile bool g_keep_running = true;
volatile bool g_game_state = false;
volatile int g_voice_value = 2;
volatile uint32_t g_uptime_sec = 0;

volatile int shan_shuo_jian_ge = 200; // ms
volatile double yu_she_pei_su = 0.50; // m/s

volatile int g_ble_cmd = 3; // 1~5 对应指令

volatile double start_jingdu = 0.00; // 起始点的精度
volatile double start_weidu = 0.00;  // 起始点的纬度
volatile int start_time = 0;         // 起始点的时刻
volatile double distance = 0.00;     // 起始点到现在位置的距离

volatile int game_sumDistance = 150;       // m
volatile int game_rewardDistance = 30;     // m
volatile double game_yu_she_pei_su = 0.50; // m/s
volatile int lastDistance = 0;             // 上一次的暂停前的距离

volatile double last_dist = 0.0;     // 记住上一次距离
volatile bool finish_played = false; // 完成音是否已播

typedef struct
{
  uint32_t total_distance_m; // 本次跑步总路程（米，整数）
  uint32_t total_time_s;     // 本次跑步总时间（秒，整数）
  float avg_speed_mps;       // 上一段的瞬时速度
} RunStats_t;

volatile RunStats_t g_run_stats = {
    .total_distance_m = 0,
    .total_time_s = 0,
    .avg_speed_mps = 0.45};

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
  delay(1000);
  Servo_SetAngle(servo_middle_angle);
  g_servo_angle = servo_middle_angle;
  delay(1000);
  audio_i2s_init();
  delay(100);

  Serial.println("System is ready.");

  xTaskCreatePinnedToCore(TaskBlink1, "TaskBlink1", 1024, NULL, 0, NULL, 0); // LED on board
  xTaskCreatePinnedToCore(TaskBlink2, "TaskBlink2", 8192, NULL, 1, NULL, 0); // servo control
  xTaskCreatePinnedToCore(TaskBlink3, "TaskBlink3", 4096, NULL, 1, NULL, 0); // MPU6050 read
  xTaskCreatePinnedToCore(TaskBlink4, "TaskBlink4", 8192, NULL, 2, NULL, 0); // GPS read
  xTaskCreatePinnedToCore(TaskBlink5, "TaskBlink5", 4096, NULL, 1, NULL, 1); // BLE send
  xTaskCreatePinnedToCore(TaskBlink6, "TaskBlink6", 4096, NULL, 1, NULL, 1); // BLE read
  xTaskCreatePinnedToCore(TaskBlink7, "TaskBlink7", 4096, NULL, 1, NULL, 1); // light control
  // xTaskCreatePinnedToCore(TaskBlink8, "TaskBlink8", 4096, NULL, 1, NULL, 0);   // Serial print
  xTaskCreatePinnedToCore(TaskBlink9, "TaskBlink9", 4096, NULL, 1, NULL, 1);   // audio output
  xTaskCreatePinnedToCore(TaskBlink10, "TaskBlink10", 4096, NULL, 1, NULL, 1); // uptime counter
  xTaskCreatePinnedToCore(TaskBlink11, "TaskBlink11", 4096, NULL, 1, NULL, 1); // gps dis get
  xTaskCreatePinnedToCore(TaskBlink12, "TaskBlink12", 4096, NULL, 1, NULL, 1); // normal mode
  xTaskCreatePinnedToCore(TaskBlink13, "TaskBlink13", 4096, NULL, 1, NULL, 1); // game mode
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
  char buf[256];

  while (true)
  {
    switch (g_ble_cmd)
    {
      /* ========== 1. GPS 数据 ========== */
    case 1:
      // 纬度
      snprintf(buf, sizeof(buf),
               "lat=%.8f\r\n",
               gps_lat);
      bleServer.sendString(buf);

      // 经度
      snprintf(buf, sizeof(buf),
               "lng=%.8f\r\n",
               gps_lng);
      bleServer.sendString(buf);

      // 卫星数
      snprintf(buf, sizeof(buf),
               "sat=%d\r\n",
               gps_sat);
      bleServer.sendString(buf);

      break;

      /* ========== 2. IMU 数据 ========== */
    case 2:
      // 加速度
      snprintf(buf, sizeof(buf),
               "ax=%.2f,ay=%.2f\r\n",
               mpu_ax, mpu_ay);
      bleServer.sendString(buf);

      snprintf(buf, sizeof(buf),
               "az=%.2f,gx=%.2f\r\n",
               mpu_az, mpu_gx);
      bleServer.sendString(buf);

      // 角速度
      snprintf(buf, sizeof(buf),
               "gy=%.2f,gz=%.2f\r\n",
               mpu_gy, mpu_gz);
      bleServer.sendString(buf);

      break;

      /* ========== 3. 里程 + 舵机角度 ========== */
    case 3:
      // 距离
      snprintf(buf, sizeof(buf),
               "Dis=%u\r\n",
               g_run_stats.total_distance_m);
      bleServer.sendString(buf);

      // 时间
      snprintf(buf, sizeof(buf),
               "T=%u\r\n",
               g_run_stats.total_time_s);
      bleServer.sendString(buf);

      // 速度
      snprintf(buf, sizeof(buf),
               "v=%.2f\r\n",
               g_run_stats.avg_speed_mps);
      bleServer.sendString(buf);

      // 舵机角度
      snprintf(buf, sizeof(buf),
               "angle=%d\r\n",
               g_servo_angle);
      bleServer.sendString(buf);

      break;

    /* ========== 4. 里程清零（一次性） ========== */
    case 4:
      g_run_stats.total_distance_m = 0;
      g_run_stats.total_time_s = 0;

      bleServer.sendString("RUN reset OK\r\n");

      g_ble_cmd = 3;
      break;

      /* ========== 5. 舵机参数回显 ========== */
    case 5:
      // 舵机中立角
      snprintf(buf, sizeof(buf),
               "mid=%d\r\n",
               servo_middle_angle);
      bleServer.sendString(buf);

      // 舵机活动角
      snprintf(buf, sizeof(buf),
               "range=%d\r\n",
               servo_max_angle);
      bleServer.sendString(buf);

      // 当前舵机角度
      snprintf(buf, sizeof(buf),
               "cur=%d\r\n",
               g_servo_angle);
      bleServer.sendString(buf);

      // 发完回到默认显示模式
      g_ble_cmd = 3;
      break;

    default:
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
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
      /* ========= 6 / 7：舵机微调（不改变模式） ========= */
      if ((rxBuf[0] == '6' || rxBuf[0] == '7') &&
          (rxBuf[1] == '\0' || rxBuf[1] == '\n' || rxBuf[1] == '\r'))
      {
        if (rxBuf[0] == '6')
        {
          g_servo_angle += 1;
        }
        else // '7'
        {
          g_servo_angle -= 1;
        }

        // 限幅保护（非常重要）
        if (g_servo_angle < 0)
          g_servo_angle = 0;
        if (g_servo_angle > 180)
          g_servo_angle = 180;

        continue; // ⚠ 即时控制，不往下走
      }

      /* ========= 5abcde：舵机参数设置 ========= */
      /* 8：进入游戏模式 */
      if (rxBuf[0] == '8' &&
          (rxBuf[1] == '\0' || rxBuf[1] == '\n' || rxBuf[1] == '\r'))
      {
        g_game_state = true;
        finish_played = false; // 建议：重新进入时允许播完成音
        last_dist = 0.0;       // 建议：奖励段重新计数
        continue;
      }

      /* 9：退出游戏模式 */
      if (rxBuf[0] == '9' &&
          (rxBuf[1] == '\0' || rxBuf[1] == '\n' || rxBuf[1] == '\r'))
      {
        g_game_state = false;
        continue;
      }
      if (rxBuf[0] == '5' && strlen(rxBuf) >= 6)
      {
        // 2-4 位：中立角 abc
        int middle =
            (rxBuf[1] - '0') * 100 +
            (rxBuf[2] - '0') * 10 +
            (rxBuf[3] - '0');

        // 5-6 位：活动角 de
        int max =
            (rxBuf[4] - '0') * 10 +
            (rxBuf[5] - '0');

        // 合法性检查（非常重要）
        if (middle >= 0 && middle <= 180 &&
            max >= 0 && max <= 90)
        {
          servo_middle_angle = middle;
          servo_max_angle = max;

          // 同步当前舵机角度到中立
          g_servo_angle = servo_middle_angle;

          // 切到 5 模式，TaskBlink5 会回显
          g_ble_cmd = 5;
        }

        continue; // ⚠ 一定要结束，不能再往下走
      }

      /* ========= 纯数字指令优先 ========= */

      if (rxBuf[0] >= '1' && rxBuf[0] <= '5' &&
          (rxBuf[1] == '\0' || rxBuf[1] == '\n' || rxBuf[1] == '\r'))
      {
        g_ble_cmd = rxBuf[0] - '0';
        continue; //  单字符指令，直接结束
      }

      /* ========= JSON 解析 ========= */
      StaticJsonDocument<256> doc; // 根据数据大小调整
      DeserializationError err = deserializeJson(doc, rxBuf);

      if (err)
      {
        // Serial.print("[JSON ERROR] ");
        // Serial.println(err.c_str());
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
        g_game_state = false;
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
        lastDistance = 0;
        g_run_stats.total_distance_m = 0;
        g_keep_running = true;
        StaticJsonDocument<64> okDoc;
        okDoc["msg"] = "Status_OK";
        char okBuf[64];
        serializeJson(okDoc, okBuf, sizeof(okBuf));
        bleServer.sendString(okBuf);
        g_keep_running = true;
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
        g_keep_running = false;
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
        g_keep_running = true;
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
        g_keep_running = false;
        StaticJsonDocument<128> distDoc;
        distDoc["distance"] = g_run_stats.total_distance_m;
        char distBuf[128];
        serializeJson(distDoc, distBuf, sizeof(distBuf));
        bleServer.sendString(distBuf);
        break;
      }

      case 5: // 设置游戏模式目标
      {
        int sumDistance = doc["sumDistance"] | 0;
        int rewardDistance = doc["rewardDistance"] | 0.0;
        game_sumDistance = sumDistance;
        game_rewardDistance = rewardDistance;
        if (sumDistance <= 0 || rewardDistance <= 0)
        {
          StaticJsonDocument<64> errDoc;
          errDoc["msg"] = "Update_Error";
          char errBuf[64];
          serializeJson(errDoc, errBuf, sizeof(errBuf));
          bleServer.sendString(errBuf);
        }
        else
        {
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
  bool led_state = false;

  while (true)
  {
    /* ========= 跑步模式：常亮 ========= */
    if (!g_game_state)
    {
      digitalWrite(21, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    /* ========= 游戏模式：按 shan_shuo_jian_ge 闪烁 ========= */
    led_state = !led_state;
    digitalWrite(21, led_state ? HIGH : LOW);

    // 闪烁间隔，单位 ms
    int interval = shan_shuo_jian_ge;
    if (interval <= 0)
      interval = 200; // 兜底保护

    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

void TaskBlink8(void *pvParameters)
{
  while (true)
  {
    Serial.printf(
        "[RUN] TotalDist=%u m | TotalTime=%u s | AvgSpeed=%.2f m/s\r\n",
        g_run_stats.total_distance_m,
        g_run_stats.total_time_s,
        g_run_stats.avg_speed_mps);
    Serial.printf("angel=%d", g_servo_angle);

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

void TaskBlink11(void *pvParameters)
{
  while (true)
  {
    distance = gps_comopute(start_weidu, start_jingdu, gps_lat, gps_lng);
    if (distance >= 25.000)
    {
      int delta_t = g_uptime_sec - start_time;
      g_run_stats.total_distance_m += (uint32_t)distance;
      if (g_run_stats.total_distance_m >= 500)
      {
        g_run_stats.total_distance_m = 0;
      }
      g_run_stats.total_time_s += delta_t;
      g_run_stats.avg_speed_mps = (float)distance / delta_t;
      if (g_run_stats.avg_speed_mps >= 10.00)
      {
        g_run_stats.avg_speed_mps = 0.0;
      }
      start_jingdu = gps_lng;
      start_weidu = gps_lat;
      start_time = g_uptime_sec;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskBlink12(void *pvParameters)
{
  while (true)
  {
    if (!g_game_state && g_keep_running)
    {
      double current_speed;
      double goal_speed;
      current_speed = g_run_stats.avg_speed_mps;
      goal_speed = yu_she_pei_su;

      int angle;
      // 防止除以0（嵌入式中必须处理，避免程序崩溃）
      if (goal_speed <= 0)
      {
        angle = servo_middle_angle; // 目标速度为0时，默认角度90
      }
      else
      {
        // 计算速度偏差比例：(目标速度 - 当前速度)/目标速度
        // 偏差为正（当前速度 < 目标速度）→ 角度增大；偏差为负（当前速度 > 目标速度）→ 角度减小
        double ratio = double(goal_speed - current_speed) / goal_speed;
        // 基础角度90，偏差比例乘以90作为增量/减量，范围刚好0~180
        angle = servo_middle_angle + servo_max_angle * ratio;
      }
      g_servo_angle = angle;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
void TaskBlink13(void *pvParameters)
{

  while (true)
  {
    if (g_game_state && g_keep_running)
    {
      double current_dist = g_run_stats.total_distance_m;
      double reward_dist = game_rewardDistance;
      double sum_dist = game_sumDistance;

      if (reward_dist <= 0)
        reward_dist = 30;

      /* ========= 1. 总距离完成，只播一次 ========= */
      if (!finish_played && current_dist >= sum_dist)
      {
        g_voice_value = 1; // 完成音
        finish_played = true;
      }

      /* ========= 2. 奖励距离，只在“跨段”时播 ========= */
      int last_segment = (int)(last_dist / reward_dist);
      int current_segment = (int)(current_dist / reward_dist);

      if (current_segment > last_segment && !finish_played)
      {
        g_voice_value = 2; // 奖励音
      }

      /* ========= 3. 舵机角度 ========= */
      double dist_in_segment = current_dist - current_segment * reward_dist;
      double ratio = dist_in_segment / reward_dist;

      // 限幅，防止异常
      if (ratio < 0.0)
        ratio = 0.0;
      if (ratio > 1.0)
        ratio = 1.0;

      // 起始和终止角度
      int start_angle = servo_middle_angle + servo_max_angle;
      int end_angle = servo_middle_angle - servo_max_angle;

      // 线性插值
      int angle = start_angle - (int)((start_angle - end_angle) * ratio);

      // 最终保护
      if (angle < 0)
        angle = 0;
      if (angle > 180)
        angle = 180;

      g_servo_angle = angle;

      /* ========= 4. 更新历史距离 ========= */
      last_dist = current_dist;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
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