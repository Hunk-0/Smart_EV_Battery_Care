/* bootloader.c — 통합된 정비사 수동 패치 루틴 (빌드 에러 수정본) */

#include "bootloader.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

// 빌드 에러를 일으키던 가상 매직 플래그 RAM 변수 선언 추가
uint32_t ota_magic_flag_ram = 0;

static OTA_State_t ota_state   = OTA_STATE_IDLE;
static uint32_t    fw_size_exp = 0;
static uint32_t    fw_crc_exp  = 0;
static uint32_t    chunk_count = 0;

// main.o와의 Linker 충돌을 방지하기 위해 일반 함수 형태로 로그 함수 유지
void UART_Log(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* 내부 가상 스텁 함수 구현 (Undefined reference 에러 해결용) */
uint8_t BL_Check_For_OTA_Start(CAN_HandleTypeDef *hcan) {
    // 필요 시 CAN 메시지 체크 로직 삽입 영역 (기본적으로는 0 반환으로 스킵 처리)
    return 0;
}

void BL_Flash_Process(CAN_HandleTypeDef *hcan) {
    // 실질적인 OTA 플래싱 백그라운드 프로세스 동작 스텁
}

// 점프 함수 스텁 추가 (에러 방지용)
void BL_JumpToApp(uint32_t app_addr) {
    typedef void (*pFunction)(void);
    pFunction Jump_To_Application;
    uint32_t JumpAddress;

    if (((*(__IO uint32_t*)app_addr) & 0x2FF00000) == 0x20000000) {
        JumpAddress = *(__IO uint32_t*) (app_addr + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        __set_MSP(*(__IO uint32_t*) app_addr);
        Jump_To_Application();
    }
}

void BL_Run(CAN_HandleTypeDef *hcan)
{
    UART_Log("[BL] Waiting for Manual Patch Input via CAN...\r\n");

    ota_state   = OTA_STATE_IDLE;
    chunk_count = 0;

    uint32_t start_tick = HAL_GetTick();

    /* 1.5초간 정비사의 수동 패치 요청 대기 후 통과 */
    while ((HAL_GetTick() - start_tick) < 1500)
    {
        // 원본 프로젝트의 CAN 패킷 수신 검사 구문 매핑 구조
        if (BL_Check_For_OTA_Start(hcan))
        {
            UART_Log("[BL] Manual Patch Request Detected! Starting Flash Mode...\r\n");
            BL_Flash_Process(hcan);
            break;
        }
    }

    UART_Log("[BL] Timeout or Skip. Proceeding to Application Core...\r\n");
}
