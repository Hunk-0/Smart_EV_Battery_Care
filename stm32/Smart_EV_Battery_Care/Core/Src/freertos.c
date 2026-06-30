/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : freertos.c
  * @brief          : 배터리 관리 시스템(BMS) 메인 제어 루틴
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "mk_dht11.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern dht11_t dht;
extern float g_raw_temp;
extern float g_display_temp;
extern void Servo_SetAngle(uint8_t angle);

uint8_t g_valve_status = 0;
uint8_t g_soh = 98; // SOH 3색 테마 연동용

osThreadId_t BMSMonitorHandle;
const osThreadAttr_t BMSMonitor_attributes = {
  .name = "BMSMonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartBMSMonitorTask(void *argument);
void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void) {
  BMSMonitorHandle = osThreadNew(StartBMSMonitorTask, NULL, &BMSMonitor_attributes);
}

/* ===================================================================
   🟩 TASK: 히스테리시스 BMS 제어 및 ESP32 데이터 송신
   =================================================================== */
void StartBMSMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartBMSMonitorTask */
  for(;;)
  {
    /* 1. DHT11 센서로부터 온도 수집 */
    if (readDHT11(&dht) == 1) {
        g_display_temp = (float)dht.temperature;
    }

    /* 2. 히스테리시스 냉각 밸브 제어
       - 40도 이상: 과열, 밸브 개방 (OPEN)
       - 35도 이하: 안정, 밸브 폐쇄 (CLOSED)
    */
    if (g_display_temp >= 40.0f) {
        g_valve_status = 1;
        Servo_SetAngle(90);
    } else if (g_display_temp <= 35.0f) {
        g_valve_status = 0;
        Servo_SetAngle(0);
    }

    /* 3. SOH(배터리 수명) 진단 기믹 */
    if (g_display_temp > 45.0f && g_soh > 60) {
        g_soh--;
    }

    /* 4. [ESP32 웹 대시보드 연동] JSON 송신
       테라텀(시리얼 모니터)에 찍히는 이 데이터가
       ESP32로 넘어가 웹 대시보드 그래프를 그립니다.
    */
    printf("{\"temp\":%.1f,\"valve\":%d,\"soh\":%d}\r\n",
           g_display_temp, g_valve_status, g_soh);

    osDelay(1000); // 1초 주기로 스트리밍
  }
  /* USER CODE END StartBMSMonitorTask */
}
