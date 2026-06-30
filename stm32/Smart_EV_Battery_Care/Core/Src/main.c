/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 스마트 EV 배터리 케어 시스템 - 메인 하드웨어 제어 및 OS 시동
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mk_dht11.h"
#include <stdio.h>

#define TEMP_THRESHOLD    35.0f
#define VIRTUAL_LOAD_ADD  15.0f
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
dht11_t dht; // freertos.c에서 이 센서 구조체를 가져가서 온도를 읽습니다.

// freertos.c 태스크들과 배터리 데이터를 공유할 실시간 전역 변수 세트
float g_raw_temp = 0.0f;
float g_display_temp = 0.0f;
uint8_t g_virtual_load = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
void Servo_SetAngle(uint8_t angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* PC 화면 테라텀 디버깅 로그용 UART 출력 매핑 (printf 활성화) */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* PA8(TIM1_CH1) 냉각수 밸브 제어용 서보모터 구동 함수 (0도 ~ 180도) */
void Servo_SetAngle(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint32_t pulse = 25 + ((uint32_t)angle * 100 / 180);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // 부트로더 점프 대기를 위한 벡터 테이블 오프셋 재배치 (0x8008000 영역)
  SCB->VTOR = 0x08008000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();       // 입출력 포트 초기화
  MX_CAN1_Init();       // 차량 내부 통신용 CAN 시동
  MX_TIM1_Init();       // 서보모터 PWM 제어용 타이머 시동
  MX_USART2_UART_Init(); // 테라텀 디버깅용 통신 시동
  MX_TIM6_Init();       // DHT11 센서 정밀 타이밍용 타이머 시동

  /* USER CODE BEGIN 2 */
  printf("\r\n=============================================\r\n");
  printf("  Smart EV Battery Care System (OS Mode Enabled)\r\n");
  printf("=============================================\r\n");

  /* TIM1 PWM 채널 가동 및 초기 상태로 밸브 완전 폐쇄(0도) 설정 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  Servo_SetAngle(0);

  /* PB0 데이터 핀과 정밀 타이머(TIM6) 기반으로 DHT11 온습도 센서 동기화 */
  init_dht11(&dht, &htim6, DHT11_DATA_GPIO_Port, DHT11_DATA_Pin);

  HAL_Delay(500); // 하드웨어 센서 소자 안정화 대기
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* freertos.c에 선언된 메인 제어 태스크들을 커널에 등록 */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* OS 스케줄러가 구동되면 제어권이 넘어가므로 아래 루프는 진입하지 않음 */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* OS 타임베이스로 사용되는 TIM7 콜백 함수 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
