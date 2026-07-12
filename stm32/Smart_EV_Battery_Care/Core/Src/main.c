/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mk_dht11.h"
/* USER CODE END Includes */

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PV */
dht11_t dht;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim1;
extern CAN_HandleTypeDef hcan2;

/* ── CAN ID 정의 (문서 14번 스펙 + 이력 프레임 추가) ────── */
#define CAN_ID_BMS_STATUS   0x100   /* STM32 -> ESP32 실시간 상태 송신 */
#define CAN_ID_BMS_HISTORY  0x101   /* STM32 -> ESP32 이력 데이터 송신 (신규) */
#define CAN_ID_BMS_CTRL     0x200   /* ESP32 -> STM32 제어 명령 수신 */
#define CAN_ID_UDS_REQ      0x7A0   /* [추가] ESP32 -> STM32 UDS 진단 요청 (문서 15번) */
#define CAN_ID_UDS_RESP     0x7A8   /* [추가] STM32 -> ESP32 UDS 진단 응답 */

/* ── OTA 관련 CAN ID (ESP32 쪽에 이미 정의되어 있던 것과 동일) ── */
#define CAN_ID_OTA_START    0x300   /* ESP32 -> STM32: 파일 크기 + CRC32 전송 */
#define CAN_ID_OTA_DATA     0x301   /* ESP32 -> STM32: 펌웨어 조각 (8바이트씩 스트리밍) */
#define CAN_ID_OTA_DONE     0x302   /* ESP32 -> STM32: 전송 완료 신호 */
#define CAN_ID_OTA_RESULT   0x311   /* STM32 -> ESP32: 검증/반영 결과 */

/* ── OTA용 Flash 구역 (STM32F446RE 512KB, Sector 0~7) ──────
   Sector 0~4 (0x08000000~0x0801FFFF, 128KB): 지금 실행 중인 앱 코드
   Sector 5   (0x08020000, 128KB): OTA 임시 보관 공간 (Staging)
   Sector 6   (0x08040000, 128KB): 예비
   Sector 7   (0x08060000, 128KB): 기존 이력/설정 저장 공간 (그대로 유지) */
#define OTA_STAGING_ADDR    0x08020000UL
#define OTA_STAGING_SECTOR  FLASH_SECTOR_5
#define OTA_APP_START_ADDR  0x08000000UL
#define OTA_APP_SECTOR_COUNT 5   /* Sector 0~4 */
#define OTA_MAX_SIZE        (128UL * 1024UL)

/* UDS 서비스 ID (문서 15번) */
#define UDS_SID_DIAG_SESSION_CONTROL  0x10
#define UDS_SID_READ_DATA_BY_ID       0x22
#define UDS_SID_READ_DTC_INFO         0x19
#define UDS_SID_WRITE_DATA_BY_ID      0x2E
#define UDS_NEG_RESPONSE              0x7F

/* UDS DID (문서 15번) */
#define DID_SOFTWARE_VERSION   0xF190
#define DID_MAX_TEMPERATURE    0xF191
#define DID_SOH                0xF192
#define DID_VALVE_COUNT        0xF193
#define DID_TEMP_THRESHOLD     0xF194

/* Command 정의 (문서 14번 + 데모용 추가) */
#define CMD_LOAD_CHANGE   0x01
#define CMD_SUMMER_MODE   0x02
#define CMD_WINTER_MODE   0x03
#define CMD_DTC_CLEAR     0x04
#define CMD_CHARGE_FULL   0x05   /* 데모 편의 기능 */
#define CMD_DEMO_STRESS   0x06   /* [추가] 시연용 이력 강제 누적 (테스트 전용, SOH 공식 자체는 안 건드림) */
#define CMD_OTA_THRESHOLD 0x07   /* [추가] OTA 파라미터 업데이트: 임계치를 Flash에 영구 저장 */
#define CMD_HISTORY_RESET 0x08   /* [추가] 이력 초기화 (테스트 전용) */
#define CMD_OTA_ENTER     0x09   /* [추가] OTA 패널 진입 -> 대기 모드로 전환 */
#define CMD_OTA_CANCEL    0x0A   /* [추가] OTA 패널에서 뒤로가기 -> 평소 모드로 복귀 */

/* ══════════════════════════════════════════════════════════
   [보안] CAN 명령 인증 + OTA 서명용 공유 비밀값
   ══════════════════════════════════════════════════════════
   STM32와 ESP32만 아는 값이라고 가정. 실제 제품이었다면 기기마다
   다른 키를 안전하게 심어두고(프로비저닝) HMAC-SHA256 같은 진짜
   암호학적 해시를 썼겠지만, 여기서는 "인증이라는 개념"을 짧은 시간에
   보여주기 위해 CRC8/CRC32에 이 비밀값을 섞어 넣는 간단한 방식으로
   구현했습니다. (CRC는 선형이라 진짜 공격자가 마음먹으면 역산이
   가능한 수준 - 발표 때 이 한계는 솔직하게 말씀하시면 됩니다) */
#define SHARED_SECRET_32  0x5A3C1E77UL

/* ══════════════════════════════════════════════════════════
   [OTA 시연용] 펌웨어 버전 - 이 두 숫자만 바꿔서 두 번 빌드하면
   "OTA 전/후" 데모용 .bin 파일 2개가 만들어집니다.
   예: 1,2로 한 번 빌드(v1.2, 원래 것) -> 1,3으로 바꿔서 다시 빌드(v1.3, 새 것)
   UDS "SW 버전(F190)" 조회 버튼으로 이 값이 바뀌었는지 눈으로 확인 가능 */
#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  2

/* Fault 비트 정의 (문서 16번 DTC) */
#define DTC_B1001_OVER_TEMP        (1 << 0)
#define DTC_B1002_VALVE_FAIL       (1 << 1)
#define DTC_B1003_TEMP_SENSOR_FAIL (1 << 2)
#define DTC_B1004_OTA_FAIL         (1 << 3)

#define SERVO_PULSE_CLOSED   1000
#define SERVO_PULSE_WARN     1500
#define SERVO_PULSE_OPEN     2000

/* ── Flash 이력 저장 영역 (문서 10번) ───────────────────── */
#define FLASH_LOG_SECTOR   FLASH_SECTOR_7
#define FLASH_LOG_ADDR     0x08060000UL   /* [수정] STM32F446RE(512KB) Sector 7 실제 시작 주소. 0x080E0000은 더 큰 용량 칩 기준이라 잘못됐었음 */
#define FLASH_MAGIC        0xB3A55A3BUL

typedef struct {
    uint32_t magic;
    float    max_temp;
    uint16_t valve_count;
    uint16_t overheat_count;
    uint8_t  temp_threshold;
    uint8_t  season;
    uint16_t ota_count;      /* OTA 파라미터 적용 횟수 (문서 10번 "OTA 적용 이력") */
} bms_history_t;

static bms_history_t hist;

/* ── 실시간 상태 변수 ────────────────────────────────────── */
static float    T_ambient      = 25.0f;
static float    T_cell         = 25.0f;
static uint8_t  actual_load    = 0;
static uint8_t  target_load    = 0;
static const float K_FACTOR    = 0.33f;  /* [수정] 0.2 -> 0.33: 최대 부하(100%) 시 온도가 50~60도대에 오도록 조정 */

static uint8_t  soc            = 100;
static float    soc_f          = 100.0f;
static uint8_t  soh            = 100;
static uint8_t  fault_byte     = 0;
static uint8_t  valve_angle    = 0;
static uint8_t  prev_valve_angle = 0;
static uint8_t  was_overheating  = 0;

/* ── OTA 상태 변수 ───────────────────────────────────────── */
typedef enum {
    OTA_STATE_NORMAL = 0,     /* 평소 모드 */
    OTA_STATE_WAITING,        /* 대기 모드 (OTA 패널 진입, 파일 아직 안 옴) */
    OTA_STATE_RECEIVING       /* 파일 조각을 받는 중 */
} ota_state_t;

static ota_state_t ota_state = OTA_STATE_NORMAL;
static uint32_t     ota_total_size    = 0;
static uint32_t     ota_expected_crc  = 0;
static uint32_t     ota_bytes_written = 0;   /* Staging에 실제로 기록된 바이트 수 */
static uint8_t       ota_ram_buf[256];        /* 4바이트씩 flash에 쓰기 전 잠깐 모아두는 RAM 버퍼 */
static uint16_t       ota_ram_buf_len = 0;
static uint8_t        ota_flash_write_error = 0;   /* [추가] Flash 쓰기 중 하나라도 실패하면 1 */

/* [보안] CAN 제어 명령 재전송 방지용 - 마지막으로 받아들인 카운터 값 */
static uint16_t can_auth_last_counter = 0;
static uint16_t can_auth_fail_count = 0;   /* 위조/재전송 시도가 몇 번 있었는지 (Fault/DTC로도 연결) */
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static void CAN_Filter_Config(void);
static uint8_t crc8_calc(const uint8_t *data, int len);
static uint32_t crc32_calc(const uint8_t *data, uint32_t len);
static uint32_t crc32_calc_seeded(const uint8_t *data, uint32_t len, uint32_t seed);
static uint8_t compute_can_auth_tag(uint8_t cmd, uint8_t p1, uint8_t p2, uint16_t counter);
static void update_bms_logic(void);
static void send_status_frame(void);
static void send_history_frame(void);
static void handle_ctrl_frame(uint8_t *data, uint8_t dlc);
static void handle_uds_request(uint8_t *rx, uint8_t dlc);
static void send_uds_response(const uint8_t *data, uint8_t len);
static void flash_load_history(void);
static void flash_save_history(void);
static void ota_enter_waiting(void);
static void ota_cancel(void);
static void ota_handle_start(uint8_t *data, uint8_t dlc);
static void ota_handle_data(uint8_t *data, uint8_t dlc);
static void ota_handle_done(void);
static void ota_flush_ram_buf(uint8_t final);
static void ota_send_result(uint8_t ok);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (CAN_ID_BMS_CTRL << 5);
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (0x7FF << 5);
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* [추가] 필터 뱅크 1 - UDS 진단 요청(0x7A0)도 받도록 */
    CAN_FilterTypeDef sFilterConfig2;
    sFilterConfig2.FilterBank = 1;
    sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig2.FilterIdHigh = (CAN_ID_UDS_REQ << 5);
    sFilterConfig2.FilterIdLow = 0x0000;
    sFilterConfig2.FilterMaskIdHigh = (0x7FF << 5);
    sFilterConfig2.FilterMaskIdLow = 0x0000;
    sFilterConfig2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig2.FilterActivation = ENABLE;
    sFilterConfig2.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK)
    {
        Error_Handler();
    }

    /* [추가] 필터 뱅크 2 - OTA 메시지(0x300~0x303: START/DATA/DONE 등)도 받도록
       마스크로 하위 2비트를 무시해서 0x300,0x301,0x302,0x303을 한 번에 수용 */
    CAN_FilterTypeDef sFilterConfig3;
    sFilterConfig3.FilterBank = 2;
    sFilterConfig3.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig3.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig3.FilterIdHigh = (CAN_ID_OTA_START << 5);
    sFilterConfig3.FilterIdLow = 0x0000;
    sFilterConfig3.FilterMaskIdHigh = (0x7FCU << 5);   /* 하위 2비트 무시 */
    sFilterConfig3.FilterMaskIdLow = 0x0000;
    sFilterConfig3.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig3.FilterActivation = ENABLE;
    sFilterConfig3.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig3) != HAL_OK)
    {
        Error_Handler();
    }
}

static uint8_t crc8_calc(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
            else crc <<= 1;
        }
    }
    return crc;
}

/* [보안] CAN 제어 명령용 인증 태그 계산.
   비밀값(SHARED_SECRET_32) + 명령 + 파라미터2개 + 카운터를 한꺼번에 CRC8로
   묶어서 계산 -> 이 비밀값을 모르면 올바른 태그를 만들 수 없음 (위조 방지),
   카운터가 매번 달라지므로 예전에 캡처해둔 메시지를 재전송해도 카운터가
   과거 값이라 거부됨 (재전송 방지) */
static uint8_t compute_can_auth_tag(uint8_t cmd, uint8_t p1, uint8_t p2, uint16_t counter)
{
    uint8_t buf[9];
    buf[0] = (uint8_t)(SHARED_SECRET_32 >> 24);
    buf[1] = (uint8_t)(SHARED_SECRET_32 >> 16);
    buf[2] = (uint8_t)(SHARED_SECRET_32 >> 8);
    buf[3] = (uint8_t)(SHARED_SECRET_32);
    buf[4] = cmd;
    buf[5] = p1;
    buf[6] = p2;
    buf[7] = (uint8_t)(counter & 0xFF);
    buf[8] = (uint8_t)((counter >> 8) & 0xFF);
    return crc8_calc(buf, 9);
}

/* ── Flash 이력 로드/저장 (문서 10번) ────────────────────── */
static void flash_load_history(void)
{
    memcpy(&hist, (void *)FLASH_LOG_ADDR, sizeof(hist));
    if (hist.magic != FLASH_MAGIC) {
        /* Flash가 비어있거나(첫 부팅) 손상됨 -> 초기값으로 시작 */
        hist.magic = FLASH_MAGIC;
        hist.max_temp = 0.0f;
        hist.valve_count = 0;
        hist.overheat_count = 0;
        hist.temp_threshold = 40;
        hist.season = 0;
        hist.ota_count = 0;
        printf("[FLASH] 이력 없음 - 초기화\r\n");
    } else {
        printf("[FLASH] 이력 로드: max_temp=%.1f valve_count=%d overheat_count=%d threshold=%d season=%d ota_count=%d\r\n",
               hist.max_temp, hist.valve_count, hist.overheat_count, hist.temp_threshold, hist.season, hist.ota_count);
    }
}

static void flash_save_history(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = FLASH_LOG_SECTOR;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        printf("[FLASH] Erase 실패 (sectorError=0x%08lX)\r\n", sectorError);
        HAL_FLASH_Lock();
        return;
    }

    uint32_t *src = (uint32_t *)&hist;
    uint32_t addr = FLASH_LOG_ADDR;
    for (size_t i = 0; i < sizeof(hist) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) {
            printf("[FLASH] Program 실패 @0x%08lX\r\n", addr);
            break;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    printf("[FLASH] 이력 저장 완료\r\n");
}

/* [추가] 자동 저장은 최소 10초 간격으로 제한 (섹터 지우기가 몇 초씩 걸려서 그 사이 CAN이 멈추는 것 방지).
   데모/OTA 버튼 같은 명시적 사용자 액션은 이 제한 없이 즉시 저장 (flash_save_history 직접 호출) */
static uint32_t last_auto_save_tick = 0;
static void flash_save_history_throttled(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_auto_save_tick < 10000) {
        return;  /* 너무 자주 저장하지 않음 - RAM 값은 이미 최신이라 화면 표시엔 문제 없음 */
    }
    last_auto_save_tick = now;
    flash_save_history();
}

/* ESP32로부터 받은 제어 명령(0x200) 처리 */
static void handle_ctrl_frame(uint8_t *data, uint8_t dlc)
{
    if (dlc < 1) return;
    uint8_t command = data[0];

    /* [보안] 인증 태그 + 카운터 검증 (프레임이 6바이트 이상 갖춰진 경우만 - 구버전 호환용 완충)
       - 태그가 안 맞으면: 이 명령이 진짜 우리 ESP32가 보낸 게 아닐 수 있음 -> 거부
       - 카운터가 이전보다 안 커졌으면: 예전에 캡처해둔 메시지를 재전송한 것일 수 있음 -> 거부 */
    if (dlc >= 6) {
        uint8_t p1 = data[1];
        uint8_t p2 = data[2];
        uint16_t counter = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
        uint8_t receivedTag = data[5];
        uint8_t expectedTag = compute_can_auth_tag(command, p1, p2, counter);

        if (receivedTag != expectedTag) {
            can_auth_fail_count++;
            printf("[보안] 인증 태그 불일치! 위조된 명령으로 의심되어 거부합니다 (누적 %d회)\r\n", can_auth_fail_count);
            return;
        }
        if (can_auth_last_counter != 0 && counter <= can_auth_last_counter) {
            can_auth_fail_count++;
            printf("[보안] 재전송 의심 (counter=%u <= 마지막 승인값=%u) -> 거부 (누적 %d회)\r\n",
                   counter, can_auth_last_counter, can_auth_fail_count);
            return;
        }
        can_auth_last_counter = counter;
    }

    switch (command) {
        case CMD_LOAD_CHANGE:
            if (dlc >= 2) {
                target_load = data[1];
                printf("[CTRL] Load 변경 -> %d%%\r\n", target_load);
            }
            break;
        case CMD_SUMMER_MODE:
            hist.season = 0;
            hist.temp_threshold = 40;
            printf("[CTRL] 여름 모드 (임계 40C, 런타임 변경만)\r\n");
            break;
        case CMD_WINTER_MODE:
            hist.season = 1;
            hist.temp_threshold = 45;
            printf("[CTRL] 겨울 모드 (임계 45C, 런타임 변경만)\r\n");
            break;
        case CMD_DTC_CLEAR:
            fault_byte = 0;
            printf("[CTRL] DTC Clear\r\n");
            break;
        case CMD_CHARGE_FULL:
            soc_f = 100.0f;
            soc = 100;
            printf("[CTRL] 완충 -> SOC 100%%\r\n");
            break;
        case CMD_DEMO_STRESS:
            /* [테스트 전용] SOH 공식/실제 계산 로직은 그대로 두고, 이력 카운터만 시연용으로 강제 누적.
               2~3회 클릭으로 정상->주의->위험까지 확실히 이동하도록 세게 누적 (overheat 가중치 1.0이 제일 큼) */
            /* [수정] 2번 누르면 주의, 4번 누르면 위험이 뜨도록 강도 재조정 (overheat 가중치 1.0이 제일 크게 작용) */
            hist.overheat_count += 12;
            hist.valve_count += 5;
            if (hist.max_temp < 45.0f) hist.max_temp = 45.0f;
            flash_save_history();
            printf("[DEMO] 이력 강제 누적 (테스트 전용) -> overheat=%d valve=%d\r\n",
                   hist.overheat_count, hist.valve_count);
            break;
        case CMD_HISTORY_RESET:
            /* [테스트 전용] 이력 초기화 - SOH를 다시 100%로 되돌림 */
            hist.max_temp = 0.0f;
            hist.valve_count = 0;
            hist.overheat_count = 0;
            hist.ota_count = 0;
            flash_save_history();
            printf("[DEMO] 이력 초기화 완료 (OTA 횟수 포함)\r\n");
            break;
        case CMD_OTA_THRESHOLD:
            /* [OTA 파라미터 업데이트] 문서 9번 - 계절별 임계치를 Flash에 영구 저장 */
            if (dlc >= 2) {
                hist.season = data[1];
                hist.temp_threshold = (hist.season == 1) ? 45 : 40;
                hist.ota_count++;
                send_status_frame();  /* [추가] Flash 쓰기(최대 1~2초 멈춤) 전에 새 값을 먼저 방송 - ESP32 화면 깜빡임 방지 */
                flash_save_history();
                printf("[OTA] 파라미터 업데이트 -> season=%d threshold=%d (누적 %d회, Flash 영구저장)\r\n",
                       hist.season, hist.temp_threshold, hist.ota_count);
            }
            break;
        case CMD_OTA_ENTER:
            ota_enter_waiting();
            break;
        case CMD_OTA_CANCEL:
            ota_cancel();
            break;
        default:
            break;
    }
}

/* [추가] UDS 응답 전송 (싱글 프레임 ISO-TP: PCI 바이트 + 데이터) */
static void send_uds_response(const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    txData[0] = len;  /* PCI: 싱글 프레임, 뒤따르는 바이트 수 */
    for (int i = 0; i < len && i < 7; i++) {
        txData[1 + i] = data[i];
    }

    txHeader.StdId = CAN_ID_UDS_RESP;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeader, txData, &txMailbox);
}

/* [추가] UDS 진단 요청(0x7A0) 처리 (문서 15번) */
static void handle_uds_request(uint8_t *rx, uint8_t dlc)
{
    if (dlc < 2) return;
    uint8_t pci = rx[0];   /* 뒤따르는 바이트 수 */
    uint8_t sid = rx[1];   /* Service ID */

    if (sid == UDS_SID_DIAG_SESSION_CONTROL) {
        /* 0x10 Diagnostic Session Control */
        uint8_t session = (pci >= 2 && dlc >= 3) ? rx[2] : 0x01;
        uint8_t resp[2] = { (uint8_t)(sid + 0x40), session };
        send_uds_response(resp, 2);
        printf("[UDS] 0x10 DiagSessionControl session=0x%02X -> OK\r\n", session);
    }
    else if (sid == UDS_SID_READ_DATA_BY_ID) {
        /* 0x22 Read Data By Identifier */
        if (pci < 3 || dlc < 4) return;
        uint16_t did = ((uint16_t)rx[2] << 8) | rx[3];
        uint8_t resp[8];
        int rlen;
        resp[0] = (uint8_t)(sid + 0x40);
        resp[1] = rx[2];
        resp[2] = rx[3];

        switch (did) {
            case DID_SOFTWARE_VERSION:
                resp[3] = FW_VERSION_MAJOR; resp[4] = FW_VERSION_MINOR;
                rlen = 5;
                break;
            case DID_MAX_TEMPERATURE:
                resp[3] = (uint8_t)hist.max_temp;
                rlen = 4;
                break;
            case DID_SOH:
                resp[3] = soh;
                rlen = 4;
                break;
            case DID_VALVE_COUNT:
                resp[3] = (uint8_t)(hist.valve_count >> 8);
                resp[4] = (uint8_t)(hist.valve_count & 0xFF);
                rlen = 5;
                break;
            case DID_TEMP_THRESHOLD:
                resp[3] = hist.temp_threshold;
                rlen = 4;
                break;
            default: {
                uint8_t neg[3] = { UDS_NEG_RESPONSE, sid, 0x31 };  /* NRC 0x31: requestOutOfRange */
                send_uds_response(neg, 3);
                printf("[UDS] 0x22 ReadDataByID DID=0x%04X -> 알 수 없는 DID (NRC 0x31)\r\n", did);
                return;
            }
        }
        send_uds_response(resp, rlen);
        printf("[UDS] 0x22 ReadDataByID DID=0x%04X -> OK\r\n", did);
    }
    else if (sid == UDS_SID_READ_DTC_INFO) {
        /* 0x19 Read DTC Information (subfunction 0x02: reportDTCByStatusMask, 간이 구현) */
        uint8_t resp[8];
        resp[0] = (uint8_t)(sid + 0x40);
        resp[1] = 0x02;   /* subfunction 에코 */
        resp[2] = 0xFF;   /* DTCStatusAvailabilityMask */
        int idx = 3;

        /* 문서 16번 DTC 중 현재 활성화된 것만 보고 (싱글 프레임이라 최대 1개 정도만 담김) */
        if (fault_byte & DTC_B1001_OVER_TEMP) {
            resp[idx++] = 0xB1; resp[idx++] = 0x00; resp[idx++] = 0x01;  /* DTC: B1001 */
            resp[idx++] = 0x08;  /* Status: testFailed */
        } else if (fault_byte & DTC_B1003_TEMP_SENSOR_FAIL) {
            resp[idx++] = 0xB1; resp[idx++] = 0x00; resp[idx++] = 0x03;  /* DTC: B1003 */
            resp[idx++] = 0x08;
        }
        send_uds_response(resp, idx);
        printf("[UDS] 0x19 ReadDTCInfo -> Fault=0x%02X\r\n", fault_byte);
    }
    else if (sid == UDS_SID_WRITE_DATA_BY_ID) {
        /* 0x2E Write Data By Identifier */
        if (pci < 3 || dlc < 5) return;
        uint16_t did = ((uint16_t)rx[2] << 8) | rx[3];

        if (did == DID_TEMP_THRESHOLD) {
            hist.temp_threshold = rx[4];
            hist.season = (hist.temp_threshold >= 45) ? 1 : 0;
            send_status_frame();
            flash_save_history();
            uint8_t resp[3] = { (uint8_t)(sid + 0x40), rx[2], rx[3] };
            send_uds_response(resp, 3);
            printf("[UDS] 0x2E WriteDataByID DID=0x%04X -> %d (Flash 저장됨)\r\n", did, rx[4]);
        } else {
            uint8_t neg[3] = { UDS_NEG_RESPONSE, sid, 0x31 };
            send_uds_response(neg, 3);
            printf("[UDS] 0x2E WriteDataByID DID=0x%04X -> 쓰기 불가능한 DID (NRC 0x31)\r\n", did);
        }
    }
}

/* ══════════════════════════════════════════════════════════
   OTA (무선 펌웨어 업데이트) 구현
   ══════════════════════════════════════════════════════════ */

/* ESP32와 완전히 동일한 CRC32 알고리즘 (poly 0xEDB88320, 반사) - 값이 안 맞으면 절대 안 됨 */
static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    return crc32_calc_seeded(data, len, 0xFFFFFFFFUL);
}

/* [보안] OTA 펌웨어 서명(간이 버전)에 쓰는 "비밀값이 섞인" CRC32.
   일반 CRC32는 시작값이 항상 0xFFFFFFFF로 고정인데, 여기서는 그 시작값 자체를
   비밀값과 섞은 값으로 바꿔서 계산합니다. 그래서 파일 내용을 알아도 비밀값을
   모르면 우리가 기대하는 서명값을 만들어낼 수 없습니다 (= 아무 펌웨어나 CRC만
   맞춰서 밀어넣을 수 없게 됨). CRC는 선형 연산이라 진짜 공격자가 정교하게
   분석하면 뚫릴 수 있는 수준이고, 실제 제품은 HMAC-SHA256 등을 씁니다 -
   여기서는 "서명이라는 개념"을 보여주는 간단한 구현입니다. */
static uint32_t crc32_calc_seeded(const uint8_t *data, uint32_t len, uint32_t seed)
{
    uint32_t crc = seed;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320UL;
            else crc = crc >> 1;
        }
    }
    return ~crc;
}

static uint32_t ota_signature_calc(const uint8_t *data, uint32_t len)
{
    uint32_t seed = 0xFFFFFFFFUL ^ SHARED_SECRET_32;
    return crc32_calc_seeded(data, len, seed);
}

/* [매우 중요] 새 펌웨어를 실제 앱 영역에 덮어쓰는 부분.
   이 함수 전체가 반드시 RAM에서 실행되어야 함 (Flash 지우는 동안 Flash를 못 읽기 때문).
   그래서 HAL 함수를 쓰지 않고 FLASH 레지스터를 직접 건드리는 최소한의 코드로 작성함.
   -> 이 프로젝트의 링커 스크립트(.ld)에는 .RamFunc 섹션이 .data 섹션 안에
      이미 포함되어 있어서(확인 완료), 별도로 손댈 것 없이 __attribute__ 만으로 동작함
      (이 파일 맨 아래 안내 주석 참고) */
__attribute__((section(".RamFunc"), noinline))
static void ota_commit_from_ram(uint32_t byte_size)
{
    __disable_irq();

    /* Flash 잠금 해제 */
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123UL;
        FLASH->KEYR = 0xCDEF89ABUL;
    }

    /* 앱 영역(Sector 0~4) 지우기 - 지금 실행 중인 바로 그 영역! */
    for (uint32_t sector = 0; sector < OTA_APP_SECTOR_COUNT; sector++) {
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_PSIZE_Msk;
        FLASH->CR |= FLASH_CR_PSIZE_1;                 /* 32비트 단위 프로그래밍 */
        FLASH->CR &= ~FLASH_CR_SNB_Msk;
        FLASH->CR |= (sector << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_SER;
        FLASH->CR |= FLASH_CR_STRT;
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_SER;
    }

    /* Staging(Sector5)에 받아둔 새 펌웨어를 앱 영역으로 그대로 복사 */
    uint32_t words = (byte_size + 3) / 4;
    volatile uint32_t *src = (volatile uint32_t *)OTA_STAGING_ADDR;
    volatile uint32_t *dst = (volatile uint32_t *)OTA_APP_START_ADDR;

    for (uint32_t i = 0; i < words; i++) {
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR |= FLASH_CR_PG;
        dst[i] = src[i];
        while (FLASH->SR & FLASH_SR_BSY) { }
        FLASH->CR &= ~FLASH_CR_PG;
    }

    FLASH->CR |= FLASH_CR_LOCK;

    NVIC_SystemReset();   /* 여기서 바로 재부팅 -> 새 펌웨어로 시작 */
}

static void ota_send_result(uint8_t ok)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t data[1] = { ok };
    txHeader.StdId = CAN_ID_OTA_RESULT;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 1;
    txHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan2, &txHeader, data, &txMailbox);
}

/* 웹에서 "OTA 업데이트" 버튼 눌러서 패널에 들어왔을 때 - 대기 모드로 전환 */
static void ota_enter_waiting(void)
{
    if (ota_state == OTA_STATE_NORMAL) {
        ota_state = OTA_STATE_WAITING;
        printf("[OTA] 대기 모드 진입\r\n");
    }
}

/* "뒤로가기" 눌렀을 때 - 평소 모드로 복귀, 받던 것 있으면 버림 */
static void ota_cancel(void)
{
    ota_state = OTA_STATE_NORMAL;
    ota_total_size = 0;
    ota_bytes_written = 0;
    ota_ram_buf_len = 0;
    printf("[OTA] 취소됨 - 평소 모드로 복귀\r\n");
}

/* 파일 크기 + CRC32 수신 (8바이트: size 4바이트 + crc32 4바이트, 리틀엔디안) */
static void ota_handle_start(uint8_t *data, uint8_t dlc)
{
    if (dlc < 8) return;
    if (ota_state != OTA_STATE_WAITING && ota_state != OTA_STATE_RECEIVING) return;

    ota_total_size = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    ota_expected_crc = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                       ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

    if (ota_total_size == 0 || ota_total_size > OTA_MAX_SIZE) {
        printf("[OTA] 파일 크기 이상함 (%lu bytes) -> 거부\r\n", ota_total_size);
        ota_send_result(0);
        return;
    }

    /* Staging 영역(Sector5) 미리 지워두기 */
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = OTA_STAGING_SECTOR;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_StatusTypeDef eraseStatus = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    /* [추가] 지우기가 실패하면 그 즉시 거부. sectorError가 0xFFFFFFFF가 아니면
       "몇 번째 섹터에서 실패했는지"를 가리키는 값이라 이것도 같이 확인해야 함.
       이걸 확인 안 하면, 덜 지워진 영역에 값을 덮어써도 겉으로는 "성공"처럼 보이다가
       나중에 CRC만 안 맞는 이상한 증상으로 나타남 (바이트 수는 맞는데 내용만 틀어짐) */
    if (eraseStatus != HAL_OK || sectorError != 0xFFFFFFFFU) {
        printf("[OTA] Staging 영역 지우기 실패! (status=%d, sectorError=0x%08lX) -> 거부\r\n",
               eraseStatus, sectorError);
        ota_send_result(0);
        return;
    }

    /* [추가] 진짜로 다 지워졌는지(전부 0xFF인지) 직접 한 번 더 확인 - 위 상태값만으로는
       못 잡는 경우까지 대비 */
    volatile uint32_t *stagingCheck = (volatile uint32_t *)OTA_STAGING_ADDR;
    for (uint32_t i = 0; i < (OTA_MAX_SIZE / 4); i++) {
        if (stagingCheck[i] != 0xFFFFFFFFUL) {
            printf("[OTA] Staging 영역이 완전히 지워지지 않음 (offset %lu) -> 거부\r\n", i * 4);
            ota_send_result(0);
            return;
        }
    }

    ota_bytes_written = 0;
    ota_ram_buf_len = 0;
    ota_flash_write_error = 0;
    ota_state = OTA_STATE_RECEIVING;
    printf("[OTA] 수신 시작: %lu bytes, 서명=0x%08lX\r\n", ota_total_size, ota_expected_crc);
    ota_send_result(2);   /* 2 = "임시 공간 준비 완료, 데이터 보내도 됨" (0=실패, 1=반영성공) */
}

/* RAM 버퍼에 모아둔 걸 4바이트 단위로 Staging Flash에 실제로 기록.
   final=true면 마지막 호출이라는 뜻이라, 4바이트로 안 나눠떨어지는 자투리도
   0xFF로 채워서 마저 씀 (그렇게 안 하면 파일 끝 1~3바이트가 영원히 안 써져서
   CRC가 항상 틀어짐 - 실제로 있었던 버그) */
static void ota_flush_ram_buf(uint8_t final)
{
    if (ota_ram_buf_len == 0) return;

    HAL_FLASH_Unlock();
    uint32_t addr = OTA_STAGING_ADDR + ota_bytes_written;
    uint32_t i = 0;
    while (i + 4 <= ota_ram_buf_len) {
        uint32_t word;
        memcpy(&word, &ota_ram_buf[i], 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word) != HAL_OK) {
            ota_flash_write_error = 1;
            printf("[OTA] Flash 쓰기 실패! addr=0x%08lX\r\n", addr);
        }
        addr += 4;
        i += 4;
    }

    uint32_t remain = ota_ram_buf_len - i;
    if (final && remain > 0) {
        /* 마지막 자투리(1~3바이트)를 0xFF로 채워서 워드 하나로 마저 기록 */
        uint8_t lastWord[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        memcpy(lastWord, &ota_ram_buf[i], remain);
        uint32_t word;
        memcpy(&word, lastWord, 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word) != HAL_OK) {
            ota_flash_write_error = 1;
            printf("[OTA] Flash 쓰기 실패! addr=0x%08lX (마지막 자투리)\r\n", addr);
        }
        i += remain;
        remain = 0;
    }
    HAL_FLASH_Lock();

    if (remain > 0) memmove(&ota_ram_buf[0], &ota_ram_buf[i], remain);
    ota_bytes_written += i;
    ota_ram_buf_len = (uint16_t)remain;
}

/* 펌웨어 조각(최대 8바이트) 수신 - 순서 그대로 스트리밍, 개별 ACK 없음 */
static void ota_handle_data(uint8_t *data, uint8_t dlc)
{
    if (ota_state != OTA_STATE_RECEIVING) return;

    for (uint8_t i = 0; i < dlc; i++) {
        if (ota_ram_buf_len < sizeof(ota_ram_buf)) {
            ota_ram_buf[ota_ram_buf_len++] = data[i];
        }
    }
    /* [수정] 매 프레임마다 바로바로 flash에 반영 (버퍼링 안 함).
       한 번에 지울 양을 최소화해서, flash 쓰는 그 짧은 순간 때문에
       다음 CAN 프레임을 놓치는 일이 없도록 함 (하드웨어 수신 버퍼가 3개뿐이라
       한 번에 오래 flash를 붙잡고 있으면 이후 프레임을 놓칠 수 있었음) */
    ota_flush_ram_buf(0);
}

/* 전송 완료 신호 수신 - CRC 검증하고, 통과하면 진짜로 반영 */
static void ota_handle_done(void)
{
    if (ota_state != OTA_STATE_RECEIVING) return;

    ota_flush_ram_buf(1);   /* 남은 것까지, 자투리 바이트까지 전부 flash에 반영 */

    printf("[OTA] 받은 바이트 수: %lu / 기대값: %lu\r\n", ota_bytes_written, ota_total_size);

    if (ota_bytes_written < ota_total_size) {
        printf("[OTA] 받은 크기(%lu)가 예상(%lu)보다 작음 -> 실패\r\n", ota_bytes_written, ota_total_size);
        ota_send_result(0);
        ota_state = OTA_STATE_WAITING;
        return;
    }

    if (ota_flash_write_error) {
        printf("[OTA] Flash 쓰기 도중 오류가 있었음 -> 반영 취소 (내용을 신뢰할 수 없음)\r\n");
        ota_send_result(0);
        ota_state = OTA_STATE_WAITING;
        return;
    }

    uint32_t crc = ota_signature_calc((const uint8_t *)OTA_STAGING_ADDR, ota_total_size);
    printf("[OTA] 수신 완료. 계산된 서명=0x%08lX / 기대값=0x%08lX\r\n", crc, ota_expected_crc);

    if (crc != ota_expected_crc) {
        printf("[OTA] CRC 불일치 -> 반영 취소, 기존 펌웨어 그대로 유지\r\n");
        ota_send_result(0);
        ota_state = OTA_STATE_WAITING;
        return;
    }

    printf("[OTA] CRC 통과! 앱 영역에 반영하고 재부팅합니다...\r\n");
    ota_send_result(1);   /* 반영 직전에 성공 응답 먼저 보내둠 */
    HAL_Delay(50);         /* 응답이 CAN 버스로 나갈 시간 확보 */

    hist.ota_count++;
    flash_save_history();

    ota_commit_from_ram(ota_total_size);   /* 여기서 끝 - 이 함수는 리턴하지 않고 바로 재부팅됨 */
}

static void update_bms_logic(void)
{
    /* [수정] 램프 속도를 늦춤: 5 -> 1 (250ms마다 1%씩, 0→100%까지 약 25초 소요) */
    if (actual_load < target_load) {
        actual_load = (uint8_t)((actual_load + 1 > target_load) ? target_load : actual_load + 1);
    } else if (actual_load > target_load) {
        actual_load = (uint8_t)((actual_load < 1) ? 0 : ((actual_load - 1 < target_load) ? target_load : actual_load - 1));
    }

    T_cell = T_ambient + ((float)actual_load * K_FACTOR);
    if (T_cell > hist.max_temp) hist.max_temp = T_cell;

    if (T_cell < 37.0f) {
        valve_angle = 0;
    } else if (T_cell < 40.0f) {
        valve_angle = 45;
    } else {
        valve_angle = 90;
    }

    if (valve_angle != prev_valve_angle) {
        hist.valve_count++;
        prev_valve_angle = valve_angle;

        uint32_t pulse = (valve_angle == 0) ? SERVO_PULSE_CLOSED
                        : (valve_angle == 45) ? SERVO_PULSE_WARN
                                              : SERVO_PULSE_OPEN;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        printf("[VALVE] %d도로 변경 (누적 %d회)\r\n", valve_angle, hist.valve_count);
        /* [수정] 여기서 바로 Flash 저장 안 함 - 전환 순간에 저장하면 그 순간 렉이 눈에 띄어서 별도 타이머로 분리 */
    }

    if (T_cell >= (float)hist.temp_threshold) {
        fault_byte |= DTC_B1001_OVER_TEMP;
        if (!was_overheating) {
            hist.overheat_count++;
            was_overheating = 1;
            printf("[DTC] B1001 Battery Over Temperature 발생 (누적 %d회)\r\n", hist.overheat_count);
            /* [수정] 여기서도 즉시 저장 안 함 (아래와 동일한 이유) */
        }
    } else {
        fault_byte &= ~DTC_B1001_OVER_TEMP;
        was_overheating = 0;
    }

    /* [수정] SOH를 먼저 계산해서, SOH가 나쁠수록 SOC가 더 빨리 닳도록 함 */
    float soh_f = 100.0f - (hist.max_temp * 0.2f) - (hist.valve_count * 0.05f) - (hist.overheat_count * 1.0f);
    if (soh_f > 100.0f) soh_f = 100.0f;
    if (soh_f < 0.0f)  soh_f = 0.0f;
    soh = (uint8_t)soh_f;

    /* SOH 100일 때 1.0배, SOH 0일 때 1.5배까지 방전 속도 증가 (건강 나쁠수록 배터리가 더 빨리 닳음) */
    /* [수정] 배율을 더 세게: SOH 100일 때 1.0배, SOH 0일 때 2.5배까지 (기존 1.5배는 체감이 약했음) */
    float discharge_multiplier = 1.0f + ((100.0f - soh_f) / 66.7f);
    soc_f -= (float)actual_load * 0.00025f * discharge_multiplier;
    if (soc_f < 0.0f) soc_f = 0.0f;
    soc = (uint8_t)soc_f;
}

static void send_status_frame(void)
{
    uint8_t data[8];
    data[0] = (uint8_t)T_cell;
    data[1] = soc;
    data[2] = soh;
    data[3] = fault_byte;
    data[4] = valve_angle;
    data[5] = actual_load;
    data[6] = hist.season;
    data[7] = crc8_calc(data, 7);

    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    txHeader.StdId = CAN_ID_BMS_STATUS;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeader, data, &txMailbox);
}

/* [추가] 이력 프레임(0x101) 전송 - 문서 10번 이력 데이터를 대시보드에 보여주기 위함 */
static void send_history_frame(void)
{
    uint8_t data[8];
    data[0] = (uint8_t)hist.max_temp;
    data[1] = (uint8_t)(hist.valve_count & 0xFF);
    data[2] = (uint8_t)(hist.valve_count >> 8);
    data[3] = (uint8_t)(hist.overheat_count & 0xFF);
    data[4] = (uint8_t)(hist.overheat_count >> 8);
    data[5] = (uint8_t)(hist.ota_count & 0xFF);
    data[6] = hist.temp_threshold;
    data[7] = crc8_calc(data, 7);

    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    txHeader.StdId = CAN_ID_BMS_HISTORY;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeader, data, &txMailbox);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* [확인 완료] 이 프로젝트의 링커 스크립트(.ld)에는 이미 .RamFunc 섹션이
     .data 섹션 안에 포함되어 있어서, 시작 코드(startup 어셈블리)가 부팅 시
     자동으로 Flash -> RAM 복사를 해줍니다. 그래서 여기서 따로 복사할 필요가
     없습니다 (원래 넣었던 수동 복사 코드는 삭제함 - 존재하지도 않는 심볼을
     참조해서 빌드 에러가 났을 것). ota_commit_from_ram() 함수에 붙여둔
     __attribute__((section(".RamFunc"))) 만으로 충분합니다. */

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  flash_load_history();

  CAN_Filter_Config();

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    printf("[FATAL] CAN2 Start FAILED\r\n");
    Error_Handler();
  }
  printf("CAN2 Start OK\r\n");

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  init_dht11(&dht, &htim6, DHT11_DATA_GPIO_Port, DHT11_DATA_Pin);

  printf("\r\n=== STM32 BMS BOOT OK (Flash history + full spec + OTA) ===\r\n");
  printf("=== Firmware Version: v%d.%d ===\r\n", FW_VERSION_MAJOR, FW_VERSION_MINOR);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint32_t last_dht_tick = 0;
  uint32_t last_logic_tick = 0;
  uint32_t last_history_tick = 0;
  uint32_t last_print_tick = 0;
  uint32_t last_flash_periodic_tick = 0;
  static uint16_t saved_valve_count = 0;
  static uint16_t saved_overheat_count = 0;

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* [매우 중요] OTA로 파일을 받는 중에는 다른 모든 배경 작업(DHT 읽기, printf 로그 출력 등)을
       완전히 멈춤. printf는 UART로 실제 전송될 때까지 블로킹되는데, 그 몇 ms 사이에
       CAN 프레임이 여러 개 도착하면 하드웨어 버퍼(3개)를 넘쳐서 파일이 깨지는 게
       실제로 확인된 원인이라 OTA 중에는 CAN 수신에만 집중하도록 함 */
    if (ota_state != OTA_STATE_RECEIVING) {

    /* DHT11은 센서 특성상 너무 자주 읽으면 오작동하므로 2.2초 간격 유지 (하드웨어 제약) */
    if (now - last_dht_tick >= 2200) {
        last_dht_tick = now;
        if (readDHT11(&dht) == 1) {
            T_ambient = (float)dht.temperature;
            fault_byte &= ~DTC_B1003_TEMP_SENSOR_FAIL;
        } else {
            fault_byte |= DTC_B1003_TEMP_SENSOR_FAIL;
        }
    }

    /* [수정] 부하 램프/밸브 판단/CAN 상태 전송은 DHT11과 별개로 훨씬 빠르게 (250ms) 반영
       -> 가상 부하 눌렀을 때 온도/밸브 반응이 훨씬 빠르게 화면에 보임 */
    if (now - last_logic_tick >= 250) {
        last_logic_tick = now;
        update_bms_logic();
        send_status_frame();
    }

    /* [추가] Flash 저장은 밸브/과열 "그 순간"과 완전히 분리해서 20초마다 배경에서 조용히.
       값이 실제로 바뀐 게 있을 때만 저장해서 불필요한 Flash 마모도 줄임.
       단, OTA 파일을 받는 중에는 절대 하지 않음 - Flash 쓰기 중 CAN 수신이 잠깐 멈추면
       그 사이 하드웨어 버퍼(3개)가 넘쳐서 전송 중인 파일이 깨질 수 있기 때문 */
    if (ota_state == OTA_STATE_NORMAL && now - last_flash_periodic_tick >= 20000) {
        last_flash_periodic_tick = now;
        if (hist.valve_count != saved_valve_count || hist.overheat_count != saved_overheat_count) {
            flash_save_history();
            saved_valve_count = hist.valve_count;
            saved_overheat_count = hist.overheat_count;
        }
    }

    /* 이력 프레임은 5초마다 (자주 안 보내도 되는 데이터) */
    if (now - last_history_tick >= 5000) {
        last_history_tick = now;
        send_history_frame();
    }

    /* Tera Term 로그는 1초에 한 번만 (너무 자주 찍으면 오히려 안 보임) */
    if (now - last_print_tick >= 1000) {
        last_print_tick = now;
        printf("T_amb=%.1f T_cell=%.1f Load=%d%% Valve=%d SOC=%d SOH=%d Fault=0x%02X | max_temp=%.1f valve_cnt=%d overheat_cnt=%d ota_cnt=%d\r\n",
               T_ambient, T_cell, actual_load, valve_angle, soc, soh, fault_byte,
               hist.max_temp, hist.valve_count, hist.overheat_count, hist.ota_count);
    }

    } /* ota_state != OTA_STATE_RECEIVING */

    /* CAN 제어 명령 수신은 매 루프마다 즉시 확인 (지연 없음) */
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            if (rxHeader.StdId == CAN_ID_BMS_CTRL)
            {
                handle_ctrl_frame(rxData, rxHeader.DLC);
            }
            else if (rxHeader.StdId == CAN_ID_UDS_REQ)
            {
                handle_uds_request(rxData, rxHeader.DLC);
            }
            else if (rxHeader.StdId == CAN_ID_OTA_START)
            {
                ota_handle_start(rxData, rxHeader.DLC);
            }
            else if (rxHeader.StdId == CAN_ID_OTA_DATA)
            {
                ota_handle_data(rxData, rxHeader.DLC);
            }
            else if (rxHeader.StdId == CAN_ID_OTA_DONE)
            {
                ota_handle_done();
            }
        }
    }
  }
  /* USER CODE END WHILE */
}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

/* ══════════════════════════════════════════════════════════════════
   [확인 완료] 링커 스크립트(.ld) 수정 필요 없음!
   ══════════════════════════════════════════════════════════════════
   실제 STM32F446RETX_FLASH.ld 파일을 확인해보니, STM32CubeIDE가 기본으로
   생성해준 .data 섹션 안에 이미 *(.RamFunc)와 *(.RamFunc*)가 포함되어
   있었습니다 (138~150번째 줄 근처). 그래서:
   - ota_commit_from_ram() 함수에 붙여둔 __attribute__((section(".RamFunc")))
     만으로 이 함수가 자동으로 RAM에 배치됩니다.
   - .data 섹션은 원래 시작 코드(startup_stm32f446xx.s)가 부팅 시 자동으로
     Flash -> RAM 복사를 해주는 표준 메커니즘이라, 따로 복사 코드를 쓸 필요도
     없습니다 (전에 드렸던 수동 복사 코드는 존재하지 않는 심볼을 참조해서
     오히려 빌드 에러를 냈을 거라 제거했습니다).

   즉 .ld 파일은 손대지 마시고, 이 main.c만 반영해서 바로 빌드하시면 됩니다.

   [테스트 순서 제안]
   1. 이 상태로 그냥 지금 쓰던 것과 "동일한" .bin 파일로 먼저 OTA 테스트
      (실패해도 어차피 같은 코드라 위험 부담이 적음)
   2. 정상적으로 반영되고 재부팅되는 것 확인되면, 그 다음부터 실제
      변경된 펌웨어로 테스트
   3. 혹시 이상하게 멈추면 → USB(ST-Link)로 그냥 다시 구워 넣으면
      바로 복구됩니다. 겁먹지 마세요.
   ══════════════════════════════════════════════════════════════════ */