/* bootloader.h
 * CAN OTA 부트로더 헤더
 *
 * Flash 구조 (STM32F446RE 512KB):
 *   Sector 0 (0x08000000, 16KB) : 이 부트로더
 *   Sector 1 (0x08004000, 16KB) : OTA 메타데이터 (플래그/크기/CRC)
 *   Sector 2~5 (0x08008000, 224KB): 현재 실행 중인 애플리케이션
 *   Sector 6~7 (0x08040000, 256KB): 새 펌웨어 수신 버퍼
 *
 * CAN2 핀:
 *   PB12 : CAN2_RX (CN10-16)
 *   PB13 : CAN2_TX (CN10-30)
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "stm32f4xx_hal.h"

/* ── Flash 영역 주소 ──────────────────────*/
#define BL_ADDR_META    0x08004000UL   /* Sector 1: OTA 메타데이터 */
#define BL_ADDR_APP     0x0800C000UL   /* Sector 2: 앱 시작 주소   */
#define BL_ADDR_NEW_FW  0x08040000UL   /* Sector 6: 새 펌웨어 버퍼 */

#define OTA_FLAG_MAGIC  0xDEADBEEFUL

/* ── CAN OTA ID ───────────────────────────*/
#define CAN_ID_OTA_START  0x300U
#define CAN_ID_OTA_DATA   0x301U
#define CAN_ID_OTA_DONE   0x302U
#define CAN_ID_OTA_ACK    0x310U
#define CAN_ID_OTA_RESULT 0x311U

#define OTA_CHUNK_SIZE  4

typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_RECEIVING,
    OTA_STATE_DONE,
    OTA_STATE_ERROR
} OTA_State_t;

typedef struct __attribute__((packed)) {
    uint32_t ota_flag;
    uint32_t fw_size;
    uint32_t fw_crc32;
    uint32_t fw_version;
} OTA_Meta_t;

void     BL_Init(CAN_HandleTypeDef *hcan);
void     BL_Run(CAN_HandleTypeDef *hcan);
void     BL_JumpToApp(uint32_t app_addr);
uint32_t CRC32_Calculate(const uint8_t *data, uint32_t len);

#endif /* INC_BOOTLOADER_H_ */
