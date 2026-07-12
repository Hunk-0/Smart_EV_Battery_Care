/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS Application Core (최종 빌드 성공 완결판)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mk_dht11.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern dht11_t dht;               // main.c의 dht 구조체 공유
extern UART_HandleTypeDef huart6; // ESP32 통신용 UART6 공유

// 서보모터 하드웨어 연동 핸들러 및 제어 상태 전역 공유
extern TIM_HandleTypeDef htim1;
extern volatile int target_valve_state;

// 클로드 분석 반영: 프리스케일러 83 기준 정밀 펄스 폭 (1.0ms = 0도, 2.0ms = 90도)
#define SERVO_PULSE_CLOSED   1000   // 0도 (냉각수 CLOSED)
#define SERVO_PULSE_OPEN     2000   // 90도 (냉각수 OPEN)
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// [★ 해결책 1] 컴파일러가 인식할 수 있도록 태스크 함수의 프로토타입 선언을 상단에 정밀 배치 (undeclared 에러 영구 소멸)
void DisplayTask_Entry(void *argument);
void ServoTask_Entry(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Display Task 생성 (조장님 원본 스펙 그대로 복구)
  static const osThreadAttr_t DisplayTask_attr = {
    .name = "DisplayTask",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityNormal,
  };
  osThreadNew(DisplayTask_Entry, NULL, &DisplayTask_attr);

  // Servo Task 생성 (조장님 원본 스펙 그대로 복구)
  static const osThreadAttr_t ServoTask_attr = {
    .name = "ServoTask",
    .stack_size = 512,
    .priority = (osPriority_t) osPriorityNormal,
  };
  osThreadNew(ServoTask_Entry, NULL, &ServoTask_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// [★ 해결책 2] 아래 구역에 중복 정의되어 빌드 오류(redefinition)를 강제로 일으키던 중복 코드는 깨끗하게 삭제했습니다.

void StartBmsTelemetryTask(void *argument) {
  for(;;) { osDelay(1000); }
}

// 조장님 고유의 온도 데이터 수집 및 ESP32(UART6) 전송 로직 완벽 유지
void DisplayTask_Entry(void *argument)
{
  for(;;)
  {
    if(readDHT11(&dht) == 1) {
        float temp = (float)dht.temperature;

        // 1. PC 터미널(UART2) 출력
        printf("DATA:TEMP=%.1f\r\n", temp);

        // 2. ESP32(UART6) 전송 -> 끝단 개행 기호를 \n으로 일치시켜 웹 대시보드가 정상 파싱됩니다.
        char buffer[32];
        int len = sprintf(buffer, "DATA:TEMP=%.1f\n", temp);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, len, 100);
    }
    osDelay(2000); // 2초 주기 반복
  }
}

// 비어있던 서보 태스크 내부에 클로드의 프리스케일러 83 기준 정밀 제어문 연동 완료!
void ServoTask_Entry(void *argument)
{
  for(;;)
  {
    // target_valve_state 상태에 맞춰 하드웨어 서보모터 밸브 정밀 구동
    if (target_valve_state == 1) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_PULSE_OPEN);   // 2000 (90도)
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_PULSE_CLOSED); // 1000 (0도)
    }
    osDelay(50); // 50ms 주기 제어 권한 유지
  }
}
/* USER CODE END Application */

