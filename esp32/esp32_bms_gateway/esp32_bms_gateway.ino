/*
 * esp32_bms_gateway.ino
 * Wi-Fi Web Server + CAN OTA 중계 및 BMS 데이터 관제 통합 게이트웨이 (최종 완결판)
 */

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include <string.h>

/* ── 1. 와이파이 설정 (환경에 맞게 수정) ───────────────── */
const char* WIFI_SSID = "kgitbank_405";   
const char* WIFI_PASS = "kgitbank@1004";  

// 공유기 연결 실패 시 켜질 자체 핫스팟망 정보 (노트북/폰 직결용)
const char* AP_SSID = "EV_BMS_Gateway_AP";
const char* AP_PASS = "12345678"; 

/* ── 2. CAN(TWAI) 핀 및 프로토콜 ID 정의 ───────────────── */
#define CAN_TX_PIN    GPIO_NUM_5
#define CAN_RX_PIN    GPIO_NUM_4

// [추가] STM32 데이터 수신용 UART 핀
#define STM32_RX_PIN  16
#define STM32_TX_PIN  17
HardwareSerial STM32Serial(2);

#define CAN_ID_BMS_DATA   0x100  
#define CAN_ID_BMS_HISTORY 0x101  /* [추가] 이력 데이터 프레임 */
#define CAN_ID_BMS_CTRL   0x200  
#define CAN_ID_UDS_REQ    0x7A0  /* [추가] UDS 진단 요청 */
#define CAN_ID_UDS_RESP   0x7A8  /* [추가] UDS 진단 응답 */

#define UDS_SID_DIAG_SESSION_CONTROL  0x10
#define UDS_SID_READ_DATA_BY_ID       0x22
#define UDS_SID_READ_DTC_INFO         0x19
#define UDS_SID_WRITE_DATA_BY_ID      0x2E

#define DID_SOFTWARE_VERSION   0xF190
#define DID_MAX_TEMPERATURE    0xF191
#define DID_SOH                0xF192
#define DID_VALVE_COUNT        0xF193
#define DID_TEMP_THRESHOLD     0xF194

/* ── OTA 관련 CAN ID (STM32와 동일해야 함) ──────────────── */
#define CAN_ID_OTA_START  0x300   /* ESP32 -> STM32: 파일 크기 + CRC32 */
#define CAN_ID_OTA_DATA   0x301   /* ESP32 -> STM32: 펌웨어 조각 (8바이트 스트리밍) */
#define CAN_ID_OTA_DONE   0x302   /* ESP32 -> STM32: 전송 완료 */
#define CAN_ID_OTA_RESULT 0x311   /* STM32 -> ESP32: 준비/성공/실패 결과 */

#define MAX_FW_SIZE       (48 * 1024)  /* [수정] 128KB -> 48KB: 실제 펌웨어(~36KB)에 맞게 줄여서 힙 할당 실패 방지 */

WebServer server(80);

/* ── 3. 실시간 글로벌 관제 데이터 변수 ─────────────────── */
float    bms_temp  = 0.0;
uint8_t  bms_soc   = 100;
uint8_t  bms_soh   = 100;
uint8_t  bms_valve = 0;          /* 0/45/90 도 */
uint8_t  bms_fault = 0;          /* DTC 비트마스크 */
uint8_t  bms_season = 0;         /* 0: 여름, 1: 겨울 */
uint8_t  virtual_load_state = 0; 
uint32_t last_can_rx_time = 0;  
uint8_t  can_comm_status  = 0;  

/* Command 정의 (STM32와 동일) */
#define CMD_LOAD_CHANGE   0x01
#define CMD_SUMMER_MODE   0x02
#define CMD_WINTER_MODE   0x03
#define CMD_DTC_CLEAR     0x04
#define CMD_CHARGE_FULL   0x05
#define CMD_DEMO_STRESS   0x06   /* [추가] 시연용 이력 강제 누적 (테스트 전용) */
#define CMD_OTA_THRESHOLD 0x07   /* [추가] OTA 파라미터 업데이트 (Flash 영구 저장) */
#define CMD_HISTORY_RESET 0x08   /* [추가] 이력 초기화 (테스트 전용) */
#define CMD_OTA_ENTER     0x09   /* [추가] OTA 패널 진입 -> STM32 대기 모드로 전환 */
#define CMD_OTA_CANCEL    0x0A   /* [추가] OTA 패널에서 뒤로가기 -> STM32 평소 모드로 복귀 */

/* [보안] STM32와 공유하는 비밀값 (main.c의 SHARED_SECRET_32와 반드시 동일해야 함) */
#define SHARED_SECRET_32  0x5A3C1E77UL
static uint16_t can_auth_counter = 1;  /* 0은 "아직 아무것도 안 옴" 취급이라 1부터 시작 */

/* [추가] 이력 데이터 (CAN 0x101 프레임에서 수신) */
float   hist_max_temp = 0.0;
uint16_t hist_valve_count = 0;
uint16_t hist_overheat_count = 0;
uint16_t hist_ota_count = 0;
uint8_t  hist_threshold = 40;

/* [추가] UDS 진단 결과 저장용 */
String   uds_last_result = "아직 조회 안 함";

/* ── 4. OTA 바이너리 임시 적재 버퍼 포인터 ────────────── */
static uint8_t  *fw_buf     = nullptr; 
static uint32_t fw_received = 0;

/* ── 5. CRC32 체크섬 계산 (STM32와 반드시 동일한 알고리즘) ── */
uint32_t crc32_calc(const uint8_t *data, size_t len) 
{
    return crc32_calc_seeded(data, len, 0xFFFFFFFF);
}

/* 표준 CRC-8 (poly 0x07, init 0x00) - STM32와 동일 알고리즘. 여기로 옮겨서
   compute_can_auth_tag()보다 먼저 정의되도록 함 */
static uint8_t crc8_calc(const uint8_t *data, int len) {
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

/* [보안] 시작값을 바꿀 수 있는 CRC32 - OTA 서명(비밀값을 처음부터 섞어서 계산)에 사용.
   STM32의 crc32_calc_seeded()와 완전히 동일해야 함 */
uint32_t crc32_calc_seeded(const uint8_t *data, size_t len, uint32_t seed) 
{
    uint32_t crc = seed;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else         crc = crc >> 1;
        }
    }
    return ~crc;
}

/* [보안] OTA 펌웨어 서명 계산 - STM32의 ota_signature_calc()와 완전히 동일해야 함 */
uint32_t ota_signature_calc(const uint8_t *data, size_t len)
{
    uint32_t seed = 0xFFFFFFFFUL ^ SHARED_SECRET_32;
    return crc32_calc_seeded(data, len, seed);
}

/* [보안] CAN 제어 명령용 인증 태그 - STM32의 compute_can_auth_tag()와 완전히 동일해야 함 */
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

/* ── 6. CAN 프레임 전송 엔진 ────────────────────────────── */
bool canSend(uint32_t id, const uint8_t *data, uint8_t len) 
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);

    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(50));
    return (res == ESP_OK);
}

/* [보안] 인증 태그 + 카운터를 자동으로 붙여서 제어 명령(0x200)을 보내는 함수.
   이제부터 모든 제어 명령은 이 함수를 통해서만 나감 */
bool canSendSecureCtrl(uint8_t cmd, uint8_t p1, uint8_t p2)
{
    uint8_t tag = compute_can_auth_tag(cmd, p1, p2, can_auth_counter);
    uint8_t payload[6];
    payload[0] = cmd;
    payload[1] = p1;
    payload[2] = p2;
    payload[3] = (uint8_t)(can_auth_counter & 0xFF);
    payload[4] = (uint8_t)((can_auth_counter >> 8) & 0xFF);
    payload[5] = tag;
    bool ok = canSend(CAN_ID_BMS_CTRL, payload, 6);
    can_auth_counter++;
    return ok;
}

/* ── 7. 웹 브라우저 프론트엔드 HTML ──────────────────────── */
void handleRoot() 
{
    static const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>스마트 EV 배터리 케어 플랫폼</title>
<style>
  :root, [data-theme="light"] {
    --glass:rgba(255,255,255,0.45); --glass-strong:rgba(255,255,255,0.68); --glass-border:rgba(255,255,255,0.6);
    --text:#171a21; --text-dim:rgba(23,26,33,0.62);
    --accent:#0060df; --green:#1f8a45; --yellow:#a5610a; --red:#d70015;
    --shadow: 0 20px 50px rgba(15,23,42,0.16);
    --wallpaper:
      radial-gradient(at 15% 15%, #ff5f6d 0px, transparent 55%),
      radial-gradient(at 85% 12%, #4facfe 0px, transparent 55%),
      radial-gradient(at 75% 85%, #ffb347 0px, transparent 55%),
      radial-gradient(at 12% 88%, #7c5cff 0px, transparent 55%),
      linear-gradient(160deg, #ff7a59 0%, #6a5cff 55%, #3a8dff 100%);
  }
  [data-theme="dark"] {
    --glass:rgba(24,20,40,0.28); --glass-strong:rgba(40,34,60,0.38); --glass-border:rgba(255,255,255,0.14);
    --text:#f5f3fa; --text-dim:rgba(245,243,250,0.66);
    --accent:#7db4ff; --green:#3ddc84; --yellow:#ffd60a; --red:#ff8484;
    --shadow: 0 20px 60px rgba(0,0,0,0.5);
    --wallpaper:
      radial-gradient(ellipse 70% 50% at 50% 38%, rgba(225,110,215,0.38) 0%, transparent 62%),
      linear-gradient(180deg, #2a1f7a 0%, #4a2a8f 26%, #6f2f8f 44%, #4a1f68 62%, #241238 82%, #0d0818 100%);
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body { height:100%; }
  body {
    margin:0; font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","Segoe UI",Roboto,Helvetica,Arial,sans-serif;
    background: var(--wallpaper); background-attachment: fixed;
    color:var(--text); display:flex; justify-content:center; align-items:flex-start;
    padding:28px 16px; min-height:100vh; transition: background 0.3s, color 0.3s;
    -webkit-font-smoothing: antialiased;
  }
  .box {
    background:var(--glass); backdrop-filter: blur(34px) saturate(180%); -webkit-backdrop-filter: blur(34px) saturate(180%);
    border:1px solid var(--glass-border); border-radius:28px; padding:26px; width:1060px; max-width:96vw;
    box-shadow: var(--shadow);
  }

  .top-bar { display:flex; justify-content:space-between; align-items:center; margin-bottom:2px; }
  h2 { font-size:1.35em; font-weight:700; letter-spacing:-0.01em; margin:0; flex:1; text-align:center; }
  h3 { color:var(--text-dim); margin:20px 0 10px 0; font-size:0.78em; font-weight:700; text-transform:uppercase; letter-spacing:0.06em; }

  #theme-toggle {
    background:var(--glass-strong); border:1px solid var(--glass-border); border-radius:50%;
    width:36px; height:36px; font-size:1em; cursor:pointer; color:var(--text);
    display:flex; align-items:center; justify-content:center; transition: transform 0.15s;
  }
  #theme-toggle:active { transform: scale(0.9); }

  .comm-bar { text-align:center; font-size:0.8em; margin:10px 0 18px 0; font-weight:700; letter-spacing:0.01em; }
  .status-on { color: var(--green); }
  .status-off { color: var(--red); }

  /* ── 메인 화면 2단: 왼쪽 대시보드 / 오른쪽 UDS 진단 ───────── */
  .main-grid { display:grid; grid-template-columns: 1.35fr 1fr; gap:20px; align-items:start; }

  .top-row { display:grid; grid-template-columns: repeat(4, 1fr); gap:12px; margin-bottom:12px; align-items:stretch; }
  .gauge-card, .stat-card {
    background:var(--glass-strong); border:1px solid var(--glass-border); border-radius:20px;
    padding:14px 8px; display:flex; flex-direction:column; align-items:center; justify-content:center; gap:6px;
  }
  .gauge {
    width:70px; height:70px; border-radius:50%; position:relative; display:flex; align-items:center; justify-content:center;
    background: conic-gradient(var(--g-color, var(--accent)) calc(var(--pct,0)*3.6deg), rgba(127,127,127,0.22) 0deg);
  }
  .gauge::before { content:""; position:absolute; inset:7px; border-radius:50%; background:var(--glass); backdrop-filter: blur(10px); }
  .gauge span { position:relative; z-index:1; font-size:0.92em; font-weight:700; font-variant-numeric: tabular-nums; }
  .gauge-label, .stat-title { font-size:0.64em; color:var(--text-dim); font-weight:700; text-transform:uppercase; letter-spacing:0.03em; text-align:center; }
  .stat-value { font-size:1.15em; font-weight:700; font-variant-numeric: tabular-nums; }

  .fault-row {
    background:var(--glass-strong); border:1px solid var(--glass-border); border-radius:18px; padding:10px 16px;
    text-align:center; margin-bottom:14px;
  }
  .fault-row .stat-title { margin-bottom:4px; }
  .fault-row .stat-value { font-size:1em; }

  .lower-row { display:grid; grid-template-columns: 1fr 1fr; gap:14px; align-items:start; }

  .hist-box { background:var(--glass-strong); border:1px solid var(--glass-border); border-radius:18px; padding:4px 14px; }
  .hist-row { display:flex; justify-content:space-between; padding:8px 0; font-size:0.82em; border-bottom:1px solid var(--glass-border); }
  .hist-row:last-child { border-bottom:none; }
  .hist-label { color:var(--text-dim); font-weight:600; }
  .hist-value { font-weight:700; font-variant-numeric: tabular-nums; text-align:right; }

  .info {
    background:var(--glass-strong); border:1px solid var(--glass-border); border-radius:16px; padding:12px;
    margin-bottom:10px; font-size:0.78em; line-height:1.5; color:var(--text-dim);
  }
  input[type=file] {
    width:100%; padding:11px; margin:10px 0; background:var(--glass-strong); color:var(--text);
    border:1px solid var(--glass-border); border-radius:12px; box-sizing:border-box; cursor:pointer; font-size:0.85em;
  }

  .ctrl-grid { display:grid; grid-template-columns:1fr 1fr; gap:9px; }
  .ctrl-grid .span2 { grid-column: span 2; }

  button, .tile {
    color:white; border:none; border-radius:15px; font-size:12.8px; font-weight:700;
    cursor:pointer; transition: transform 0.12s, filter 0.15s; letter-spacing:-0.005em;
  }
  .tile {
    padding:12px 8px; display:flex; flex-direction:column; align-items:center; justify-content:center; gap:3px;
    text-align:center; min-height:56px; line-height:1.25;
  }
  .tile .tile-icon { font-size:1.2em; }
  button:active, .tile:active { transform: scale(0.96); }
  button:hover, .tile:hover { filter: brightness(1.08); }

  .btn-upload { width:100%; padding:13px; margin-bottom:9px; background:var(--green); }
  .btn-back { background:var(--glass-strong); color:var(--text); border:1px solid var(--glass-border); }
  .tile-load-off { background:var(--accent); }
  .tile-load-on { background:var(--red); }
  .tile-back { background:var(--glass-strong); color:var(--text); border:1px solid var(--glass-border); }
  .tile-dev { background:#8e5cf7; }
  .tile-danger { background:var(--red); }
  .tile-dtc { background:#4a90a4; }
  .tile-charge { background:#30b158; }
  .tile-season { background:#5e5ce6; }
  .tile-ota { background:var(--accent); }
  .tile-uds { background:#c9518f; }

  #status { margin-top:10px; text-align:center; font-size:0.8em; min-height:20px; color:var(--accent); font-weight:700; }
  .hidden { display:none !important; }

  #toast {
    position: fixed; bottom: 28px; left: 50%; transform: translateX(-50%) translateY(16px);
    background:rgba(20,20,28,0.88); backdrop-filter: blur(20px); color: #fff; border:1px solid rgba(255,255,255,0.12);
    padding: 13px 22px; border-radius: 16px; font-weight:700; font-size: 0.86em; box-shadow: 0 20px 50px rgba(0,0,0,0.3);
    opacity: 0; pointer-events: none; transition: opacity 0.25s, transform 0.25s; z-index: 999;
  }
  #toast.show { opacity: 1; transform: translateX(-50%) translateY(0); }

  @media (max-width: 860px) {
    .main-grid { grid-template-columns: 1fr; }
    .top-row { grid-template-columns: 1fr 1fr; }
  }
</style>
<script>
  function applyTheme(t) {
    document.documentElement.setAttribute('data-theme', t);
    document.getElementById('theme-toggle').innerText = (t === 'light') ? '🌙' : '☀️';
    localStorage.setItem('ev_theme', t);
  }
  function toggleTheme() {
    let cur = document.documentElement.getAttribute('data-theme') === 'light' ? 'dark' : 'light';
    applyTheme(cur);
  }
  (function() {
    let saved = localStorage.getItem('ev_theme') || 'light';
    applyTheme(saved);
  })();
</script>
</head>
<body>
<div class="box">

  <div id="main-content">
    <div class="top-bar">
      <h2>🚗 스마트 EV 배터리 케어 플랫폼</h2>
      <button id="theme-toggle" onclick="toggleTheme()">🌙</button>
    </div>
    <div class="comm-bar">
      <span id="comm_status" class="status-off">🔍 통신 상태 확인 중...</span>
    </div>

    <div class="main-grid">
      <div class="left-col">
        <div class="top-row">
          <div class="gauge-card">
            <div class="gauge" id="soc-gauge" style="--pct:0; --g-color:var(--accent);"><span id="soc">--</span></div>
            <div class="gauge-label">배터리 잔량 (SOC)</div>
          </div>
          <div class="gauge-card">
            <div class="gauge" id="soh-gauge" style="--pct:0; --g-color:var(--green);"><span id="soh">--</span></div>
            <div class="gauge-label">배터리 건강 상태 (SOH)</div>
            <div id="soh-label" style="font-size:0.72em; font-weight:700;">--</div>
          </div>
          <div class="stat-card"><div class="stat-title">배터리 온도 (T_cell)</div><div id="temp" class="stat-value">--.- °C</div></div>
          <div class="stat-card"><div class="stat-title">냉각수 밸브 상태</div><div id="valve" class="stat-value" style="font-size:1em; color:var(--accent);">--</div></div>
        </div>

        <div class="fault-row">
          <div class="stat-title">Fault / DTC</div>
          <div id="fault" class="stat-value">--</div>
        </div>

        <div class="lower-row">
          <div>
            <h3>📜 상태 이력 (Flash 저장)</h3>
            <div class="hist-box">
              <div class="hist-row"><span class="hist-label">최고 온도</span><span id="hist_max_temp" class="hist-value">--</span></div>
              <div class="hist-row"><span class="hist-label">밸브 동작</span><span id="hist_valve" class="hist-value">--</span></div>
              <div class="hist-row"><span class="hist-label">과열 발생</span><span id="hist_overheat" class="hist-value">--</span></div>
              <div class="hist-row"><span class="hist-label">OTA 적용</span><span id="hist_ota" class="hist-value">--</span></div>
              <div class="hist-row"><span class="hist-label">과열 임계치</span><span id="hist_threshold" class="hist-value">--</span></div>
            </div>
          </div>

          <div>
            <h3>🎛 시뮬레이션 컨트롤</h3>
            <div class="ctrl-grid">
              <div id="btn_load" class="tile tile-load-off span2" onclick="toggleVirtualLoad()"><span class="tile-icon">⚡</span>가상 부하 시뮬레이터</div>
              <div id="btn_season" class="tile tile-season span2" data-season="0" onclick="toggleSeason()">☀️ 여름모드 (40°C) - 클릭 시 겨울 전환</div>
              <div class="tile tile-dtc" onclick="clearDtc()"><span class="tile-icon">🧹</span>DTC Clear</div>
              <div class="tile tile-ota" onclick="enterOtaPanel()"><span class="tile-icon">🔄</span>OTA 업데이트</div>
              <div class="tile tile-dev span2" onclick="showPanel('dev-view')"><span class="tile-icon">🛠</span>개발자 모드</div>
            </div>
          </div>
        </div>
      </div>

      <div class="right-col">
        <h3>🔬 UDS 진단 (ISO 14229)</h3>
        <div class="hist-box" style="margin-bottom:10px;">
          <div class="hist-row"><span class="hist-label">진단 결과</span></div>
          <div class="hist-row" style="border:none; padding-top:0;"><span id="uds-result" class="hist-value" style="text-align:left; font-size:0.9em;">--</span></div>
        </div>
        <div class="ctrl-grid">
          <div class="tile tile-uds span2" onclick="udsSession()"><span class="tile-icon">🔑</span>0x10 진단 세션 시작</div>
          <div class="tile tile-uds" onclick="udsRead('F190','SW 버전')">SW 버전</div>
          <div class="tile tile-uds" onclick="udsRead('F191','최고온도')">최고 온도</div>
          <div class="tile tile-uds" onclick="udsRead('F192','SOH')">SOH</div>
          <div class="tile tile-uds" onclick="udsRead('F193','밸브횟수')">밸브 횟수</div>
          <div class="tile tile-uds span2" onclick="udsRead('F194','임계치')">임계치 (F194)</div>
          <div class="tile tile-uds span2" onclick="udsDtc()"><span class="tile-icon">📋</span>0x19 DTC 조회</div>
        </div>
        <div class="info" style="margin-top:14px;">
          같은 데이터라도 <b>표준 UDS 요청-응답 프로토콜</b>(세션→서비스ID→DID)을 거쳐서 조회합니다. 왼쪽 대시보드는 STM32가 알아서 계속 출력하고, 여기는 직접 "조회 요청"을 보냅니다.
        </div>
      </div>
    </div>
  </div>

  <div id="dev-view" class="hidden">
    <div class="top-bar"><h2>🛠 개발자 모드</h2></div>
    <div class="info">테스트 및 관리용 기능입니다. 실제 BMS 계산 로직(SOH 공식 등)에는 영향을 주지 않고, 이력 데이터만 직접 조작합니다.</div>

    <div class="ctrl-grid">
      <div class="tile tile-charge" onclick="chargeFull()"><span class="tile-icon">🔋</span>배터리 충전</div>
      <div class="tile tile-dev" onclick="demoStress()"><span class="tile-icon">🎬</span>이력 강제 누적</div>
      <div class="tile tile-danger span2" onclick="historyReset()"><span class="tile-icon">🧾</span>이력 초기화</div>
      <div class="tile tile-back span2" onclick="backToMain()"><span class="tile-icon">🔙</span>뒤로가기</div>
    </div>
  </div>

  <div id="ota-progress-view" class="hidden">
    <div class="top-bar"><h2>🔄 펌웨어 무선 업데이트</h2></div>
    <div class="comm-bar" style="color:var(--accent);">🔓 STM32 대기 모드 (파일 수신 준비 완료)</div>

    <div class="info">
      📋 <b>사용 방법:</b><br>
      1. STM32 <b>.bin 파일</b>을 선택합니다.<br>
      2. 업로드 버튼을 누르면 CAN으로 전송되고, STM32가 검증 후 자동으로 반영·재부팅합니다.<br>
      3. 이 화면을 나가면(뒤로가기) STM32는 다시 평소 모드로 돌아갑니다.
    </div>

    <input type="file" id="firmware_file" accept=".bin"
           onchange="document.getElementById('status').textContent=this.files[0]?this.files[0].name+' 준비됨':''">

    <input type="text" id="firmware_sig" placeholder="서명값 8자리 (hex) - 예: A1B2C3D4  (모르면 비워두면 자동 계산)"
           style="width:100%; padding:11px; margin:0 0 10px 0; background:var(--glass-strong); color:var(--text); border:1px solid var(--glass-border); border-radius:12px; box-sizing:border-box; font-size:0.85em;">

    <button type="button" class="btn-upload" onclick="executeOTA()">🚀 무선 업로드 및 파일 전송</button>

    <div style="font-size:0.75em; color:var(--text-dim); margin:2px 0 10px 2px; line-height:1.5;">
      🔏 정상 릴리즈는 미리 계산된 서명값을 같이 받아옵니다. 서명값을 모르거나 틀리면 STM32가 거부합니다 (기존 펌웨어 유지).
    </div>

    <div id="status">파일을 선택하고 업로드 시작을 누르세요.</div>
    <pre id="ota-log" style="background:rgba(0,0,0,0.75); color:#7CFCA5; font-size:11px; line-height:1.5; padding:10px 12px; border-radius:10px; height:150px; overflow-y:auto; margin:8px 0; white-space:pre-wrap; word-break:break-all;"></pre>
    <button type="button" class="btn-back" onclick="exitOtaPanel()">🔙 뒤로가기</button>
  </div>

</div>
<div id="toast"></div>
<script>
  const SEASON_TEXT = {
    0: "☀️ 여름모드 (40°C) - 클릭 시 겨울 전환",
    1: "❄️ 겨울모드 (45°C) - 클릭 시 여름 전환"
  };

  setInterval(function() {
    if(document.getElementById('main-content').classList.contains('hidden')) return;

    fetch('/data').then(response => response.json()).then(data => {
      let commEl = document.getElementById('comm_status');
      if(data.comm === 1) {
        commEl.innerText = "🟢 통신 정상 연결됨";
        commEl.className = "status-on";

        document.getElementById('temp').innerText = data.temp.toFixed(1) + " °C";

        let socGauge = document.getElementById('soc-gauge');
        socGauge.style.setProperty('--pct', data.soc);
        document.getElementById('soc').innerText = data.soc + "%";

        let sohGauge = document.getElementById('soh-gauge');
        sohGauge.style.setProperty('--pct', data.soh);
        let sohColor = (data.soh >= 70) ? 'var(--green)' : (data.soh >= 50) ? 'var(--yellow)' : 'var(--red)';
        sohGauge.style.setProperty('--g-color', sohColor);
        document.getElementById('soh').innerText = data.soh + "%";
        let sohBadge = document.getElementById('soh-label');
        sohBadge.innerText = (data.soh >= 70) ? "정상" : (data.soh >= 50) ? "주의" : "위험";
        sohBadge.style.color = sohColor;

        let valveEl = document.getElementById('valve');
        if (data.valve >= 90) {
          valveEl.innerText = "열림 (90°)"; valveEl.style.color = 'var(--red)';
        } else if (data.valve >= 45) {
          valveEl.innerText = "열림 (45°)"; valveEl.style.color = 'var(--yellow)';
        } else {
          valveEl.innerText = "닫힘 (0°)"; valveEl.style.color = 'var(--accent)';
        }

        let faultEl = document.getElementById('fault');
        if (data.fault === 0) {
          faultEl.innerText = "정상 (DTC 없음)"; faultEl.style.color = 'var(--green)';
        } else {
          let dtcList = [];
          if (data.fault & 0x01) dtcList.push("B1001 과열");
          if (data.fault & 0x02) dtcList.push("B1002 밸브고장");
          if (data.fault & 0x04) dtcList.push("B1003 센서고장");
          if (data.fault & 0x08) dtcList.push("B1004 OTA실패");
          faultEl.innerText = dtcList.join(", "); faultEl.style.color = 'var(--red)';
        }

        let btnLoad = document.getElementById('btn_load');
        if(data.vload === 1) {
          btnLoad.innerHTML = '<span class="tile-icon">🛑</span>가상 부하 시뮬레이터 중지'; btnLoad.className = "tile tile-load-on span2";
        } else {
          btnLoad.innerHTML = '<span class="tile-icon">⚡</span>가상 부하 시뮬레이터'; btnLoad.className = "tile tile-load-off span2";
        }

        let seasonBtn = document.getElementById('btn_season');
        let now = Date.now();
        if (!window._seasonLockUntil || now > window._seasonLockUntil) {
          seasonBtn.innerText = SEASON_TEXT[data.season];
          seasonBtn.setAttribute('data-season', data.season);
        }
      } else {
        commEl.innerText = "⚠️ 통신 상태 확인 필요 (STM32 연결 끊김)";
        commEl.className = "status-off";
        document.getElementById('temp').innerText = "--.- °C";
        document.getElementById('soc').innerText = "--";
        document.getElementById('soh').innerText = "--";
        document.getElementById('soh-label').innerText = "--";
        document.getElementById('valve').innerText = "--";
        document.getElementById('fault').innerText = "--";
      }

      document.getElementById('hist_max_temp').innerText = data.hist_max_temp.toFixed(1) + " °C";
      document.getElementById('hist_valve').innerText = data.hist_valve + " 회";
      document.getElementById('hist_overheat').innerText = data.hist_overheat + " 회";
      document.getElementById('hist_ota').innerText = data.hist_ota + " 회";
      document.getElementById('hist_threshold').innerText = data.hist_threshold + " °C";
    }).catch(err => {
      let commEl = document.getElementById('comm_status');
      commEl.innerText = "❌ 웹 서버 연결 끊김 (ESP32 전원 확인)";
      commEl.className = "status-off";
    });
  }, 200);

  function showToast(msg) {
    let toast = document.getElementById('toast');
    toast.innerText = msg;
    toast.classList.add('show');
    clearTimeout(window._toastTimer);
    window._toastTimer = setTimeout(() => toast.classList.remove('show'), 2000);
  }

  function toggleVirtualLoad() { fetch('/toggle-load'); }

  function toggleSeason() {
    let btn = document.getElementById('btn_season');
    let currentSeason = parseInt(btn.getAttribute('data-season') || '0');
    let newSeason = currentSeason === 1 ? 0 : 1;
    showToast(newSeason === 1 ? "❄️ 겨울 모드로 전환 완료 (Flash 저장됨)" : "☀️ 여름 모드로 전환 완료 (Flash 저장됨)");
    btn.setAttribute('data-season', newSeason);
    btn.innerText = SEASON_TEXT[newSeason];
    window._seasonLockUntil = Date.now() + 2500;
    fetch('/ota-threshold?season=' + newSeason);
  }

  function clearDtc() { showToast("🧹 DTC가 초기화되었습니다"); fetch('/dtc-clear'); }
  function chargeFull() { showToast("🔋 충전되었습니다"); fetch('/charge'); }
  function demoStress() { showToast("🎬 시연용 이력이 누적되었습니다 (테스트 전용)"); fetch('/demo-stress'); }
  function historyReset() { showToast("🧾 이력이 초기화되었습니다 (테스트 전용)"); fetch('/history-reset'); }

  function fetchUdsResult() {
    fetch('/uds-result').then(r => r.text()).then(t => {
      document.getElementById('uds-result').innerText = t;
    });
  }
  function udsSession() {
    showToast("🔑 진단 세션 시작 요청 (0x10)");
    fetch('/uds-session').then(() => setTimeout(fetchUdsResult, 300));
  }
  function udsRead(didHex, label) {
    showToast("📡 " + label + " 조회 중... (0x22)");
    fetch('/uds-read?did=' + didHex).then(() => setTimeout(fetchUdsResult, 300));
  }
  function udsDtc() {
    showToast("📋 DTC 조회 중... (0x19)");
    fetch('/uds-dtc').then(() => setTimeout(fetchUdsResult, 300));
  }

  function showPanel(id) {
    document.getElementById('main-content').classList.add('hidden');
    document.getElementById('dev-view').classList.add('hidden');
    document.getElementById('ota-progress-view').classList.add('hidden');
    document.getElementById(id).classList.remove('hidden');
  }
  function backToMain() { showPanel('main-content'); }

  function enterOtaPanel() {
    showPanel('ota-progress-view');
    fetch('/ota-enter');
    showToast("🔓 STM32가 대기 모드로 전환되었습니다");
  }
  function exitOtaPanel() {
    fetch('/ota-cancel');
    showToast("🔒 STM32가 평소 모드로 복귀했습니다");
    showPanel('main-content');
  }

  function executeOTA() {
    let fileInput = document.getElementById('firmware_file');
    let sigInput = document.getElementById('firmware_sig');
    let statusEl = document.getElementById('status');
    let logEl = document.getElementById('ota-log');
    if (fileInput.files.length === 0) {
      alert("업로드할 STM32 펌웨어(.bin) 파일을 먼저 선택해주세요!");
      return;
    }
    let file = fileInput.files[0];
    let sigValue = sigInput.value.trim();
    let formData = new FormData();
    formData.append("firmware", file);
    statusEl.style.color = "var(--accent)";
    statusEl.textContent = sigValue
      ? "📡 펌웨어 + 서명값(" + sigValue + ") 전송 중..."
      : "📡 펌웨어 데이터 송신 중... (서명값 미입력 - 자동 계산)";
    logEl.textContent = "";

    let url = '/ota' + (sigValue ? ('?sig=' + encodeURIComponent(sigValue)) : '');
    fetch(url, { method: 'POST', body: formData })
    .then(response => {
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let leftover = "";

      function processLine(line) {
        if (line === "STATUS:OK") {
          statusEl.style.color = "var(--green)";
          statusEl.textContent = "🎉 펌웨어 반영 성공! STM32가 재부팅됩니다.";
          setTimeout(() => { backToMain(); }, 3000);
        } else if (line === "STATUS:FAIL") {
          statusEl.style.color = "var(--red)";
          statusEl.textContent = "❌ 반영 거부됨 - 위 로그를 확인하세요 (서명 불일치 시 정상 동작입니다).";
        } else if (line.length > 0) {
          logEl.textContent += line + "\n";
          logEl.scrollTop = logEl.scrollHeight;
        }
      }

      function read() {
        reader.read().then(({ done, value }) => {
          if (done) {
            if (leftover.length > 0) processLine(leftover);
            return;
          }
          let text = decoder.decode(value, { stream: true });
          let lines = (leftover + text).split("\n");
          leftover = lines.pop();  /* 마지막 줄은 아직 안 끝났을 수 있으니 다음 청크와 합침 */
          lines.forEach(processLine);
          read();
        });
      }
      read();
    })
    .catch(err => {
      statusEl.style.color = "var(--red)";
      statusEl.textContent = "❌ 웹서버 통신 실패 (네트워크 점검)";
    });
  }
</script>
</body>
</html>
)rawliteral";
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.send(200, "text/html; charset=utf-8", html);
}

/* ── 8. 실시간 JSON 데이터 리턴 핸들러 ──────────────────── */
void handleData() 
{
    char json[280];
    snprintf(json, sizeof(json), "{\"temp\":%.1f,\"soc\":%d,\"soh\":%d,\"valve\":%d,\"comm\":%d,\"vload\":%d,\"fault\":%d,\"season\":%d,"
             "\"hist_max_temp\":%.1f,\"hist_valve\":%d,\"hist_overheat\":%d,\"hist_ota\":%d,\"hist_threshold\":%d}",
             bms_temp, bms_soc, bms_soh, bms_valve, can_comm_status, virtual_load_state, bms_fault, bms_season,
             hist_max_temp, hist_valve_count, hist_overheat_count, hist_ota_count, hist_threshold);
    server.send(200, "application/json; charset=utf-8", json);
}

/* ── 9. 가상 부하 제어 핸들러 ───────────────────────────── */
void handleToggleLoad() 
{
    virtual_load_state = (virtual_load_state == 0) ? 1 : 0;

    uint8_t loadPercent = (virtual_load_state == 1) ? 100 : 0;  /* [수정] 60->100%: 90도(위험) 단계까지 확실히 시연되도록 */
    canSendSecureCtrl(CMD_LOAD_CHANGE, loadPercent, bms_season);

    server.send(200, "text/plain", "OK");
}

/* ── [추가] 계절 모드 토글 (문서 9번 - 여름/겨울 임계치 전환) ── */
void handleToggleSeason()
{
    bms_season = (bms_season == 0) ? 1 : 0;
    uint8_t command = (bms_season == 1) ? CMD_WINTER_MODE : CMD_SUMMER_MODE;
    canSendSecureCtrl(command, 0, 0);
    server.send(200, "text/plain", "OK");
}

/* ── [추가] DTC Clear (문서 14, 16번) ───────────────────── */
void handleDtcClear()
{
    canSendSecureCtrl(CMD_DTC_CLEAR, 0, 0);
    server.send(200, "text/plain", "OK");
}

/* ── [추가] 완충 (데모 편의 기능, 문서에는 없음) ────────── */
void handleCharge()
{
    canSendSecureCtrl(CMD_CHARGE_FULL, 0, 0);
    server.send(200, "text/plain", "OK");
}

/* ── [추가] 시연용 이력 강제 누적 (테스트 전용, SOH 공식은 그대로) ── */
void handleDemoStress()
{
    canSendSecureCtrl(CMD_DEMO_STRESS, 0, 0);
    server.send(200, "text/plain", "OK");
}

/* ── [추가] 이력 초기화 (테스트 전용) ────────────────────── */
void handleHistoryReset()
{
    canSendSecureCtrl(CMD_HISTORY_RESET, 0, 0);
    server.send(200, "text/plain", "OK");
}

/* ── [추가] OTA 대기 모드 진입/취소 (패널 열고 닫을 때) ───── */
void handleOtaEnter()
{
    canSendSecureCtrl(CMD_OTA_ENTER, 0, 0);
    server.send(200, "text/plain", "OK");
}
void handleOtaCancel()
{
    canSendSecureCtrl(CMD_OTA_CANCEL, 0, 0);
    server.send(200, "text/plain", "OK");
}

/* ── [추가] UDS 진단 요청 전송 함수들 (문서 15번) ────────── */
void udsSendSessionControl(uint8_t session)
{
    uint8_t payload[3] = { 0x02, UDS_SID_DIAG_SESSION_CONTROL, session };
    canSend(CAN_ID_UDS_REQ, payload, 3);
}
void udsSendReadDataByID(uint16_t did)
{
    uint8_t payload[4] = { 0x03, UDS_SID_READ_DATA_BY_ID, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF) };
    canSend(CAN_ID_UDS_REQ, payload, 4);
}
void udsSendReadDTC()
{
    uint8_t payload[4] = { 0x03, UDS_SID_READ_DTC_INFO, 0x02, 0xFF };
    canSend(CAN_ID_UDS_REQ, payload, 4);
}

void handleUdsSession()
{
    udsSendSessionControl(0x03);  /* Extended Diagnostic Session */
    server.send(200, "text/plain", "OK");
}
void handleUdsRead()
{
    uint16_t did = server.hasArg("did") ? (uint16_t)strtol(server.arg("did").c_str(), NULL, 16) : 0;
    udsSendReadDataByID(did);
    server.send(200, "text/plain", "OK");
}
void handleUdsDtc()
{
    udsSendReadDTC();
    server.send(200, "text/plain", "OK");
}
void handleUdsResult()
{
    server.send(200, "text/plain; charset=utf-8", uds_last_result);
}

/* ── [추가] OTA 파라미터 업데이트 (문서 9번 - Flash에 영구 저장) ── */
void handleOtaThreshold()
{
    uint8_t targetSeason = server.hasArg("season") ? server.arg("season").toInt() : 0;
    canSendSecureCtrl(CMD_OTA_THRESHOLD, targetSeason, 0);
    server.send(200, "text/plain", "OK");
}

/* ── 10. 웹 서버 바이너리 파일 스트림 적재 루틴 ─────────── */
void handleOTAUpload() 
{
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Stream Start: %s\n", upload.filename.c_str());
        fw_received = 0;

        if (fw_buf == nullptr) {
            Serial.println("[ERROR] fw_buf was never allocated at boot!");
            return;
        }
        memset(fw_buf, 0, MAX_FW_SIZE); 
    } 
    else if (upload.status == UPLOAD_FILE_WRITE) {
        if (fw_buf != nullptr && (fw_received + upload.currentSize <= MAX_FW_SIZE)) {
            memcpy(fw_buf + fw_received, upload.buf, upload.currentSize);
            fw_received += upload.currentSize;
        }
    } 
    else if (upload.status == UPLOAD_FILE_END) {
        Serial.printf("[OTA] Stream Completed. Total: %d Bytes\n", fw_received);
    }
}

/* [추가] 웹 화면에 실시간으로 로그를 흘려보내는 헬퍼.
   HTTP 청크 전송(chunked)으로 한 줄씩 바로바로 브라우저에 보내고, 동시에 시리얼에도 찍음 */
void otaLog(const String &msg)
{
    Serial.println(msg);
    server.sendContent(msg + "\n");
}

/* ── 11. STM32로 펌웨어 스트리밍 전송 (새 프로토콜: ACK 없이 스트리밍 + 최종 서명 검증) ──
   simulateAttack=true면 [보안 시연용] 일부러 틀린 서명을 보내서, STM32가 정상적으로
   거부하는지 확인하는 테스트 모드. 실제 파일 내용은 똑같이 보내고 서명값만 조작함.
   [수정] 응답을 한 번에 안 주고 chunked로 스트리밍 -> 진행 상황이 실시간으로 웹 화면에
   그대로 찍힘 (시리얼 모니터 없이 웹 화면만 녹화해도 과정이 다 보이도록) */
void handleOTA()
{
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain; charset=utf-8", "");   /* 헤더만 먼저 보내고, 본문은 아래에서 한 줄씩 흘려보냄 */

    if (fw_received == 0 || fw_buf == nullptr) {
        otaLog("[OTA] ❌ 오류: 업로드된 파일이 없습니다.");
        otaLog("STATUS:FAIL");
        return;
    }

    otaLog("[OTA] CAN 스트리밍 시작...");
    otaLog("[OTA] 파일 크기: " + String(fw_received) + " bytes");

    /* [보안] 서명값을 사용자가 직접 입력했으면 그 값을 그대로 쓰고(= "미리 서명된 정식 릴리즈"를
       흉내), 안 넣었으면 지금처럼 ESP32가 즉석에서 계산(= 평소 편의용 자동 모드).
       진짜 Secure OTA는 서명을 게이트웨이가 만드는 게 아니라 빌드/배포 시점에 이미 정해져서
       같이 오는 것이므로, 이 방식이 훨씬 더 실제와 가깝습니다. */
    uint32_t image_crc;
    if (server.hasArg("sig") && server.arg("sig").length() > 0) {
        image_crc = (uint32_t)strtoul(server.arg("sig").c_str(), NULL, 16);
        char buf[80];
        snprintf(buf, sizeof(buf), "[OTA] 입력된 서명값 사용: 0x%08X (직접 입력됨)", image_crc);
        otaLog(String(buf));
    } else {
        image_crc = ota_signature_calc(fw_buf, fw_received);
        char buf[80];
        snprintf(buf, sizeof(buf), "[OTA] 서명값 자동 계산: 0x%08X (편의 모드)", image_crc);
        otaLog(String(buf));
    }

    /* START 프레임: 파일 크기(4바이트) + 서명(4바이트) */
    uint8_t start_packet[8];
    start_packet[0] = (fw_received & 0xFF);
    start_packet[1] = ((fw_received >> 8) & 0xFF);
    start_packet[2] = ((fw_received >> 16) & 0xFF);
    start_packet[3] = ((fw_received >> 24) & 0xFF);
    start_packet[4] = (image_crc & 0xFF);
    start_packet[5] = ((image_crc >> 8) & 0xFF);
    start_packet[6] = ((image_crc >> 16) & 0xFF);
    start_packet[7] = ((image_crc >> 24) & 0xFF);
    canSend(CAN_ID_OTA_START, start_packet, 8);
    otaLog("[OTA] STM32에 크기·서명 전송 완료. 임시 저장 공간 정리 대기 중...");

    /* STM32가 임시 보관 공간(Staging)을 다 지울 때까지 기다림 */
    uint32_t waitStart = millis();
    bool ready = false;
    while (millis() - waitStart < 5000) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            if (rx_msg.identifier == CAN_ID_OTA_RESULT && rx_msg.data_length_code >= 1 && rx_msg.data[0] == 2) {
                ready = true;
                break;
            }
        }
    }
    if (!ready) {
        otaLog("[OTA] ❌ STM32 준비 신호 없음 (Timeout) -> 중단");
        otaLog("STATUS:FAIL");
        return;
    }
    otaLog("[OTA] STM32 준비 완료! 데이터 전송을 시작합니다.");

    /* 데이터 스트리밍: 8바이트씩, 개별 ACK 없이 순서대로 쭉 전송. 10%마다 진행률 로그 */
    uint32_t offset = 0;
    uint32_t lastReportedPct = 0;
    while (offset < fw_received) {
        uint8_t chunk_len = (uint8_t)((fw_received - offset > 8) ? 8 : (fw_received - offset));
        canSend(CAN_ID_OTA_DATA, fw_buf + offset, chunk_len);
        offset += chunk_len;
        delay(3);  /* STM32가 한 프레임씩 처리할 시간 확보 */

        uint32_t pct = (offset * 100UL) / fw_received;
        if (pct >= lastReportedPct + 10) {
            lastReportedPct = (pct / 10) * 10;
            otaLog("[OTA] 진행률: " + String(lastReportedPct) + "%  (" + String(offset) + " / " + String(fw_received) + " bytes)");
        }
    }
    otaLog("[OTA] 진행률: 100% - 전송 완료");

    /* DONE 신호 */
    uint8_t donePacket[1] = { 0x01 };
    canSend(CAN_ID_OTA_DONE, donePacket, 1);
    otaLog("[OTA] STM32의 서명 검증 결과를 기다리는 중...");

    /* STM32의 최종 결과(서명 검증 + 반영) 대기 */
    waitStart = millis();
    bool gotResult = false, otaOk = false;
    while (millis() - waitStart < 8000) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            if (rx_msg.identifier == CAN_ID_OTA_RESULT && rx_msg.data_length_code >= 1) {
                gotResult = true;
                otaOk = (rx_msg.data[0] == 1);
                break;
            }
        }
    }

    if (!gotResult) {
        otaLog("[OTA] ❌ STM32 응답 없음 (Timeout)");
        otaLog("STATUS:FAIL");
    } else if (otaOk) {
        otaLog("[OTA] ✅ 서명 검증 통과! 앱 영역에 반영하고 재부팅합니다...");
        otaLog("STATUS:OK");
    } else {
        otaLog("[OTA] ❌ 서명 불일치 - 반영 취소됨 (기존 펌웨어 그대로 유지)");
        otaLog("STATUS:FAIL");
    }
}

/* ── 12. 주변장치 셋업 및 하드웨어 바인딩 ────────────────── */
void setup() 
{
    Serial.begin(115200);
    // [추가] STM32와 UART 통신 개방 (지금은 안 쓰지만 배선 호환용으로 유지)
    STM32Serial.begin(9600, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

    delay(1000); 
    Serial.println("\n=== EV BMS GATEWAY BOOT START ===");

    // [추가] OTA 펌웨어 버퍼를 부팅 시 딱 한 번만 할당 (힙이 제일 깨끗할 때) - 매 업로드마다 malloc하면 실패하기 쉬움
    fw_buf = (uint8_t *)malloc(MAX_FW_SIZE);
    if (fw_buf == nullptr) {
        Serial.println("[FATAL] OTA buffer allocation failed at boot!");
    } else {
        Serial.printf("[SUCCESS] OTA buffer allocated (%d bytes)\n", MAX_FW_SIZE);
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();  // [수정] 500kbps -> 100kbps: 종단저항 없이 사용하기 위해
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println("[SUCCESS] CAN(TWAI) Peripheral Initialized (100Kbps).");
    } else {
        Serial.println("[FATAL ERROR] CAN Peripheral Initialization Failed!");
    }

    Serial.printf("Connecting to Wi-Fi SSID: %s ", WIFI_SSID);
    WiFi.mode(WIFI_MODE_APSTA); 
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int connection_timeout_counter = 0;
    while (WiFi.status() != WL_CONNECTED && connection_timeout_counter < 20) {
        delay(500);
        Serial.print(".");
        connection_timeout_counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[SUCCESS] Connected to Wi-Fi Router Network!");
        Serial.printf("🔗 Station URL: http://%s/\n", WiFi.localIP().toString().c_str());
    } 
    else {
        Serial.println("\n[WARNING] Router Connection Timeout! Activating Backup Hotspot...");
        WiFi.softAP(AP_SSID, AP_PASS);
        Serial.println("[SUCCESS] Emergency SoftAP Hotspot Is Now Online.");
        Serial.printf("📡 Hotspot SSID : %s\n", AP_SSID);
        Serial.printf("🔑 Hotspot Password : %s\n", AP_PASS);
        Serial.printf("🔗 Backup AP URL : http://%s/\n", WiFi.softAPIP().toString().c_str());
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.on("/toggle-load", HTTP_GET, handleToggleLoad);
    server.on("/toggle-season", HTTP_GET, handleToggleSeason);
    server.on("/dtc-clear", HTTP_GET, handleDtcClear);
    server.on("/charge", HTTP_GET, handleCharge);
    server.on("/demo-stress", HTTP_GET, handleDemoStress);
    server.on("/history-reset", HTTP_GET, handleHistoryReset);
    server.on("/uds-session", HTTP_GET, handleUdsSession);
    server.on("/uds-read", HTTP_GET, handleUdsRead);
    server.on("/uds-dtc", HTTP_GET, handleUdsDtc);
    server.on("/uds-result", HTTP_GET, handleUdsResult);
    server.on("/ota-threshold", HTTP_GET, handleOtaThreshold);
    server.on("/ota-enter", HTTP_GET, handleOtaEnter);
    server.on("/ota-cancel", HTTP_GET, handleOtaCancel);
    server.on("/ota", HTTP_POST, handleOTA, handleOTAUpload);

    server.begin();
    Serial.println("=== GATEWAY WEB SERVER INITIALIZATION COMPLETED ===");
}

/* ── 13. 메인 루프 ──────────────────────────────────────── */
void loop() 
{
    server.handleClient();

    /* [추가] 30초마다 현재 접속 IP를 다시 찍어줘서, 시리얼 모니터를 부팅 순간에 못 봤어도
       스크롤 없이 최근 로그에서 바로 IP를 확인할 수 있게 함 */
    static uint32_t lastIpReminder = 0;
    if (millis() - lastIpReminder >= 30000) {
        lastIpReminder = millis();
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[Wi-Fi] 접속 주소: http://%s/  (공유기 연결됨)\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.printf("[Wi-Fi] 접속 주소: http://%s/  (SoftAP 모드, SSID: %s)\n", WiFi.softAPIP().toString().c_str(), AP_SSID);
        }
    }

    // STM32가 CAN(0x100)으로 보내는 상태 프레임 수신 (CRC8 검증)
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (rx_msg.identifier == CAN_ID_BMS_DATA && rx_msg.data_length_code == 8) {
            uint8_t crcCalc = crc8_calc(rx_msg.data, 7);
            if (crcCalc == rx_msg.data[7]) {
                bms_temp   = (float)(int8_t)rx_msg.data[0];
                bms_soc    = rx_msg.data[1];
                bms_soh    = rx_msg.data[2];
                bms_fault  = rx_msg.data[3];
                bms_valve  = rx_msg.data[4];
                bms_season = rx_msg.data[6];

                last_can_rx_time = millis();
                can_comm_status = 1;
                /* [수정] 0.25초마다 찍으면 로그가 너무 빨리 밀려서 부팅 로그(Wi-Fi 연결/IP)를
                   찾기 힘들다는 피드백을 받아, 1초에 한 번만 출력하도록 줄임 */
                static uint32_t lastStatusPrint = 0;
                if (millis() - lastStatusPrint >= 1000) {
                    lastStatusPrint = millis();
                    Serial.printf("[STATUS RX] T=%.1f SOC=%d SOH=%d Fault=0x%02X Valve=%d Load=%d Season=%d\n",
                                  bms_temp, bms_soc, bms_soh, bms_fault, bms_valve, rx_msg.data[5], bms_season);
                }
            } else {
                Serial.println("[STATUS RX] CRC8 mismatch, frame discarded");
            }
        } else if (rx_msg.identifier == CAN_ID_BMS_HISTORY && rx_msg.data_length_code == 8) {
            uint8_t crcCalc = crc8_calc(rx_msg.data, 7);
            if (crcCalc == rx_msg.data[7]) {
                hist_max_temp       = (float)rx_msg.data[0];
                hist_valve_count    = rx_msg.data[1] | (rx_msg.data[2] << 8);
                hist_overheat_count = rx_msg.data[3] | (rx_msg.data[4] << 8);
                hist_ota_count      = rx_msg.data[5];
                hist_threshold      = rx_msg.data[6];
                Serial.printf("[HISTORY RX] max_temp=%.1f valve_cnt=%d overheat_cnt=%d ota_cnt=%d threshold=%d\n",
                              hist_max_temp, hist_valve_count, hist_overheat_count, hist_ota_count, hist_threshold);
            } else {
                Serial.println("[HISTORY RX] CRC8 mismatch, frame discarded");
            }
        } else if (rx_msg.identifier == CAN_ID_UDS_RESP) {
            /* UDS 진단 응답(0x7A8) 파싱 - 싱글 프레임 ISO-TP */
            uint8_t sid = rx_msg.data[1];
            String result;

            if (sid == 0x7F) {
                char buf[48];
                snprintf(buf, sizeof(buf), "❌ 오류 응답 (NRC 0x%02X)", rx_msg.data[3]);
                result = String(buf);
            } else if (sid == (UDS_SID_DIAG_SESSION_CONTROL + 0x40)) {
                char buf[48];
                snprintf(buf, sizeof(buf), "✅ 진단 세션 시작 (세션 0x%02X)", rx_msg.data[2]);
                result = String(buf);
            } else if (sid == (UDS_SID_READ_DATA_BY_ID + 0x40)) {
                uint16_t did = ((uint16_t)rx_msg.data[2] << 8) | rx_msg.data[3];
                char buf[48];
                if (did == DID_SOFTWARE_VERSION) {
                    snprintf(buf, sizeof(buf), "SW 버전: v%d.%d", rx_msg.data[4], rx_msg.data[5]);
                } else if (did == DID_MAX_TEMPERATURE) {
                    snprintf(buf, sizeof(buf), "최고 온도: %d°C", rx_msg.data[4]);
                } else if (did == DID_SOH) {
                    uint8_t sohVal = rx_msg.data[4];
                    const char *statusText = (sohVal >= 70) ? "정상" : (sohVal >= 50) ? "주의" : "위험";
                    snprintf(buf, sizeof(buf), "SOH: %d%% (%s)", sohVal, statusText);
                } else if (did == DID_VALVE_COUNT) {
                    uint16_t vc = ((uint16_t)rx_msg.data[4] << 8) | rx_msg.data[5];
                    snprintf(buf, sizeof(buf), "밸브 동작 횟수: %d회", vc);
                } else if (did == DID_TEMP_THRESHOLD) {
                    snprintf(buf, sizeof(buf), "과열 임계치: %d°C", rx_msg.data[4]);
                } else {
                    snprintf(buf, sizeof(buf), "알 수 없는 DID 응답");
                }
                result = String(buf);
            } else if (sid == (UDS_SID_READ_DTC_INFO + 0x40)) {
                /* [추가] 디버그용 원시 바이트 출력 - 프로토콜이 안 맞을 때 바로 확인 가능 */
                Serial.printf("[UDS DTC RAW] len=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                              rx_msg.data_length_code,
                              rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                              rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
                if (rx_msg.data_length_code > 3 && rx_msg.data[0] > 3) {
                    /* data[4]=DTC 상위바이트(예: 0xB1), data[5]/data[6]=DTC 나머지 2바이트를 합쳐서
                       "B1001"처럼 표시, data[7]=상태 바이트 */
                    uint16_t dtcLow = ((uint16_t)rx_msg.data[5] << 8) | rx_msg.data[6];
                    char buf[48];
                    snprintf(buf, sizeof(buf), "DTC %X%03X 발견 (상태 0x%02X)",
                             rx_msg.data[4], dtcLow, rx_msg.data[7]);
                    result = String(buf);
                } else {
                    result = "DTC 없음 (정상)";
                }
            } else if (sid == (UDS_SID_WRITE_DATA_BY_ID + 0x40)) {
                result = "✅ 쓰기 완료 (Flash 저장됨)";
            } else {
                result = "알 수 없는 UDS 응답";
            }

            uds_last_result = result;
            Serial.println("[UDS RESP] " + result);
        }
        /* CAN_ID_OTA_RESULT(0x311)는 handleOTA() 안에서 twai_receive()로 직접 받으므로 여기서는 처리 안 함 */
    }

    // 6초 이상 상태 프레임이 안 오면 연결 끊김으로 표시
    if (millis() - last_can_rx_time > 6000) {
        can_comm_status = 0;
    }
}
