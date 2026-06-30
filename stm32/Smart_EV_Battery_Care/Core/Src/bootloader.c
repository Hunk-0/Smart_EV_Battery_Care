/* bootloader.c
 * CAN OTA 부트로더 구현
 *
 * 동작 흐름:
 *   1. BL_Init()  : CAN2 필터 설정 및 시작
 *   2. BL_Run()   : 메타데이터 확인
 *      - OTA 플래그 있음 → OTA 수신 루프
 *      - OTA 플래그 없음 → BL_JumpToApp()
 *   3. OTA 수신 루프:
 *      - OTA_START  수신 → 버퍼 Erase
 *      - OTA_DATA   수신 → Flash Write
 *      - OTA_DONE   수신 → CRC 검증 → 앱 영역 복사 → 재부팅
 */

/* bootloader.c — CAN OTA 부트로더 구현 */

#include "bootloader.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static OTA_State_t ota_state   = OTA_STATE_IDLE;
static uint32_t    fw_size_exp = 0;
static uint32_t    fw_crc_exp  = 0;
static uint32_t    chunk_count = 0;

static void UART_Log(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      strlen(msg), HAL_MAX_DELAY);
}

uint32_t CRC32_Calculate(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
    return ~crc;
}

static uint32_t GetSector(uint32_t addr)
{
    if (addr < 0x08004000) return FLASH_SECTOR_0;
    if (addr < 0x08008000) return FLASH_SECTOR_1;
    if (addr < 0x0800C000) return FLASH_SECTOR_2;
    if (addr < 0x08010000) return FLASH_SECTOR_3;
    if (addr < 0x08020000) return FLASH_SECTOR_4;
    if (addr < 0x08040000) return FLASH_SECTOR_5;
    if (addr < 0x08060000) return FLASH_SECTOR_6;
    return FLASH_SECTOR_7;
}

static HAL_StatusTypeDef EraseFlashSectors(uint32_t start_addr,
                                            uint32_t size)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t               error;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector       = GetSector(start_addr);
    erase.NbSectors    = GetSector(start_addr + size - 1)
                         - erase.Sector + 1;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef ret = HAL_FLASHEx_Erase(&erase, &error);
    HAL_FLASH_Lock();
    return ret;
}

static HAL_StatusTypeDef WriteFlash(uint32_t addr,
                                     const uint8_t *data,
                                     uint32_t len)
{
    HAL_FLASH_Unlock();

    /* 4바이트 정렬된 부분 Write */
    uint32_t i = 0;
    for (; i + 4 <= len; i += 4) {
        uint32_t word;
        memcpy(&word, &data[i], 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              addr + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    /* 나머지 1~3바이트: 0xFF 패딩 후 Write */
    if (i < len) {
        uint32_t word = 0xFFFFFFFF;
        memcpy(&word, &data[i], len - i);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              addr + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

static void SendACK(CAN_HandleTypeDef *hcan,
                    uint16_t chunk_num, uint8_t status)
{
    CAN_TxHeaderTypeDef tx_hdr;
    uint8_t             tx_data[8] = {0};
    uint32_t            tx_mailbox;

    tx_data[0] =  chunk_num & 0xFF;
    tx_data[1] = (chunk_num >> 8) & 0xFF;
    tx_data[2] =  status;

    tx_hdr.StdId              = CAN_ID_OTA_ACK;
    tx_hdr.ExtId              = 0;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = 8;
    tx_hdr.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hcan, &tx_hdr, tx_data, &tx_mailbox);
}

static void SendResult(CAN_HandleTypeDef *hcan,
                       uint8_t result, uint8_t crc_ok)
{
    CAN_TxHeaderTypeDef tx_hdr;
    uint8_t             tx_data[8] = {0};
    uint32_t            tx_mailbox;

    tx_data[0] = result;
    tx_data[1] = crc_ok;

    tx_hdr.StdId              = CAN_ID_OTA_RESULT;
    tx_hdr.ExtId              = 0;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = 8;
    tx_hdr.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hcan, &tx_hdr, tx_data, &tx_mailbox);
}

void BL_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter;

    filter.FilterBank           = 14;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &filter);
    HAL_CAN_Start(hcan);

    UART_Log("[BL] Bootloader v1.0 Started\r\n");
    UART_Log("[BL] CAN2: PB12=RX(CN10-16), PB13=TX(CN10-30), 500kbps\r\n");
}

/* ═══════════════════════════════════════════
   BL_Run: OTA_START를 무한 대기
   ─────────────────────────────────────────
   ⚠️ 타임아웃 없이 OTA_START가 올 때까지 대기
      테스트 완료 후 플래그 방식으로 변경 가능
   ═══════════════════════════════════════════*/
void BL_Run(CAN_HandleTypeDef *hcan)
{
    UART_Log("[BL] Waiting for OTA_START (no timeout)...\r\n");
    UART_Log("[BL] Send firmware from ESP32 now.\r\n");

    ota_state   = OTA_STATE_IDLE;
    chunk_count = 0;

    /* ── 무한 대기 — OTA_START가 올 때까지 ── */
    while (1)
    {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0)
            continue;

        CAN_RxHeaderTypeDef rx_hdr;
        uint8_t             rx_data[8] = {0};

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                  &rx_hdr, rx_data) != HAL_OK)
            continue;

        /* ── OTA_START 수신 ───────────────── */
        if (rx_hdr.StdId == CAN_ID_OTA_START)
        {
            fw_size_exp = (uint32_t)rx_data[0]
                        | ((uint32_t)rx_data[1] << 8)
                        | ((uint32_t)rx_data[2] << 16)
                        | ((uint32_t)rx_data[3] << 24);
            fw_crc_exp  = (uint32_t)rx_data[4]
                        | ((uint32_t)rx_data[5] << 8)
                        | ((uint32_t)rx_data[6] << 16)
                        | ((uint32_t)rx_data[7] << 24);

            char log[80];
            snprintf(log, sizeof(log),
                     "[BL] OTA START: size=%lu CRC=0x%08lX\r\n",
                     fw_size_exp, fw_crc_exp);
            UART_Log(log);

            UART_Log("[BL] Erasing Sector 6~7...\r\n");
            if (EraseFlashSectors(BL_ADDR_NEW_FW, fw_size_exp) != HAL_OK) {
                SendACK(hcan, 0xFFFF, 0x01);
                UART_Log("[BL] Erase FAILED\r\n");
                ota_state = OTA_STATE_ERROR;
                break;
            }
            UART_Log("[BL] Erase done. Ready.\r\n");

            ota_state   = OTA_STATE_RECEIVING;
            chunk_count = 0;
            SendACK(hcan, 0xFFFF, 0x00);
        }

        /* ── OTA_DATA 수신 ────────────────── */
        else if (rx_hdr.StdId == CAN_ID_OTA_DATA
                 && ota_state == OTA_STATE_RECEIVING)
        {
            uint16_t chunk_num = (uint16_t)rx_data[0]
                               | ((uint16_t)rx_data[1] << 8);
            uint32_t offset    = (uint32_t)chunk_num * OTA_CHUNK_SIZE;
            uint32_t remain    = fw_size_exp - offset;
            uint32_t pay_len   = (remain >= OTA_CHUNK_SIZE)
                                 ? OTA_CHUNK_SIZE : remain;

            /* 첫 번째와 마지막 청크 디버그 출력 */
            if (chunk_num == 0 || chunk_num >= 5173) {
                char log[80];
                snprintf(log, sizeof(log),
                         "[BL] chunk=%u offset=%lu pay=%lu data=%02X%02X%02X%02X\r\n",
                         chunk_num, offset, pay_len,
                         rx_data[2], rx_data[3], rx_data[4], rx_data[5]);
                UART_Log(log);
            }

            WriteFlash(BL_ADDR_NEW_FW + offset, &rx_data[2], pay_len);

//            if (WriteFlash(BL_ADDR_NEW_FW + offset, &rx_data[2], pay_len) != HAL_OK)
//            {
//                SendACK(hcan, chunk_num, 0x01);
//                UART_Log("[BL] Flash write FAILED\r\n");
//                ota_state = OTA_STATE_ERROR;
//                break;
//            }

            chunk_count++;
            SendACK(hcan, chunk_num, 0x00);

            if (chunk_count % 100 == 0) {
                char log[64];
                snprintf(log, sizeof(log),
                         "[BL] %lu chunks (%.1f%%)\r\n",
                         chunk_count,
                         100.0f * offset / fw_size_exp);
                UART_Log(log);
            }
        }

        /* ── OTA_DONE 수신 ────────────────── */
        else if (rx_hdr.StdId == CAN_ID_OTA_DONE
                 && ota_state == OTA_STATE_RECEIVING)
        {
            UART_Log("[BL] OTA_DONE. Verifying CRC...\r\n");
            ota_state = OTA_STATE_DONE;

            /* 4바이트 정렬 크기로 CRC 계산
               Flash Write 시 나머지를 0xFF로 패딩했으므로
               ESP32와 동일하게 정렬된 크기로 계산해야 함 */
            uint32_t aligned_size = (fw_size_exp + 3) & ~3U;
            uint32_t calc_crc = CRC32_Calculate(
                (const uint8_t *)BL_ADDR_NEW_FW, aligned_size);

            char log[80];
            snprintf(log, sizeof(log),
                     "[BL] CRC calc=0x%08lX exp=0x%08lX → %s\r\n",
                     calc_crc, fw_crc_exp,
                     (calc_crc == fw_crc_exp) ? "PASS" : "FAIL");
            UART_Log(log);

            if (calc_crc == fw_crc_exp)
            {
                UART_Log("[BL] CRC OK! Erasing App area...\r\n");
                EraseFlashSectors(BL_ADDR_APP, aligned_size);

                UART_Log("[BL] Copying firmware to App area...\r\n");
                WriteFlash(BL_ADDR_APP,
                           (const uint8_t *)BL_ADDR_NEW_FW,
                           aligned_size);

                SendResult(hcan, 0x00, 0x01);
                UART_Log("[BL] OTA complete! Rebooting...\r\n");
                HAL_Delay(100);
                //NVIC_SystemReset();
                BL_JumpToApp(BL_ADDR_APP);
            }
            else
            {
                UART_Log("[BL] CRC FAIL! Jumping to existing app.\r\n");
                SendResult(hcan, 0x01, 0x00);
                ota_state = OTA_STATE_ERROR;
            }
            break;
        }
    } /* while(1) */

    /* OTA 실패 시 기존 앱으로 점프 */
    UART_Log("[BL] Jumping to App...\r\n");
    HAL_Delay(50);
    BL_JumpToApp(BL_ADDR_APP);
}

void BL_JumpToApp(uint32_t app_addr)
{
    uint32_t sp = *(volatile uint32_t *)app_addr;

    if ((sp & 0xFF000000) != 0x20000000) {
        UART_Log("[BL] ERROR: No valid app!\r\n");
        while(1);
    }

    uint32_t reset_handler = *(volatile uint32_t *)(app_addr + 4);

    char log[64];
    snprintf(log, sizeof(log),
             "[BL] SP=0x%08lX, ResetHandler=0x%08lX\r\n",
             sp, reset_handler);
    UART_Log(log);

    __disable_irq();
    SCB->VTOR = app_addr;
    __set_MSP(sp);
    __enable_irq();

    void (*app_entry)(void) = (void (*)(void))reset_handler;
    app_entry();
}

