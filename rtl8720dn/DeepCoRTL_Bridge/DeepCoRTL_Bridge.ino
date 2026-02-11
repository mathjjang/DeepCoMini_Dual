/*
  DeepCoRTL_Bridge.ino (RTL8720DN / BW16 등, Arduino IDE)

  목표:
  - RTL8720DN이 Wi‑Fi AP + WebSocket 서버를 제공 (로봇 IP = 192.168.4.1)
  - DeepCoConnector가 기대하는 엔드포인트를 유지
    - control: ws://192.168.4.1/ws      (port 80)
    - camera : ws://192.168.4.1:81/     (port 81)
  - 수신한 제어 문자열을 UART로 ESP32‑S3에게 전달

  필요 라이브러리(Arduino Library Manager):
  - WebSockets (Links2004/arduinoWebSockets)

  보드 패키지:
  - Realtek AmebaD (RTL8720DN) 보드 매니저 설치 필요

  핀/파라미터 변경은 config.h에서 수정하세요.
*/

#include "config.h"
#include <WiFi.h>
#include <SPI.h>

// WebSocketsServer (Links2004/arduinoWebSockets)
// Arduino min/max 매크로 충돌 방지
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include "src/dc_ws/WebSocketsServer.h" // project-local copy (no global library patch)

// Realtek/Ameba low-level Wi-Fi API (AP client list)
extern "C" {
  #include "wifi_conf.h"
}

// -------------------------------------------------------
// Ameba RTL8720DN: LOGUARTClass / UARTClassTwo에 printf()가 없음
// snprintf + print 로 에뮬레이션하는 헬퍼 매크로
// 스택 로컬 버퍼 사용 → RTOS 태스크 동시 접근 안전
// -------------------------------------------------------
#define RTL_PRINTF(ser, fmt, ...) \
  do { char _pf[192]; snprintf(_pf, sizeof(_pf), fmt, ##__VA_ARGS__); ser.print(_pf); } while(0)

// -----------------------------
// Optional: split work into RTOS tasks (if available)
// - Task A: WS accept/poll + UART (robot control + log pump)
// - Task B: SPI pull (camera frames) -> keep latest frame cached
// -----------------------------
#if defined(__has_include)
  #if __has_include("FreeRTOS.h")
    #include "FreeRTOS.h"
    #include "task.h"
    #include "semphr.h"
    #define RTL_HAS_FREERTOS 1
  #elif __has_include(<FreeRTOS.h>)
    #include <FreeRTOS.h>
    #include <task.h>
    #include <semphr.h>
    #define RTL_HAS_FREERTOS 1
  #elif __has_include("freertos/FreeRTOS.h")
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
    #define RTL_HAS_FREERTOS 1
  #else
    #define RTL_HAS_FREERTOS 0
  #endif
#else
  #define RTL_HAS_FREERTOS 0
#endif

// 사용자가 원하는 구조(태스크 분리)를 기본값으로 켬.
// 단, RTL Arduino 코어에서 FreeRTOS 헤더가 없으면 자동으로 loop() 방식으로 폴백됩니다.
static constexpr bool RTL_USE_TASKS = (RTL_HAS_FREERTOS != 0);
static bool g_tasksEnabledRuntime = RTL_USE_TASKS; // task 생성 실패 시 loop 모드로 런타임 폴백
#if RTL_HAS_FREERTOS
static bool g_tasksStartedRuntime = false;
#endif

// -----------------------------
// User config — 기본값은 config.h, 런타임 변경은 @set 명령
// -----------------------------
// v0.1.1: 런타임 변경 가능 변수 (config.h의 기본값으로 초기화)
static char g_apPassword[32] = CFG_AP_PASSWORD_DEFAULT;
static int  g_apChannel = CFG_AP_CHANNEL_DEFAULT;

// 로봇 고정 IP (DeepCoConnector 호환) — config.h
static const IPAddress AP_IP(CFG_AP_IP_A, CFG_AP_IP_B, CFG_AP_IP_C, CFG_AP_IP_D);
static const IPAddress AP_GW(CFG_AP_IP_A, CFG_AP_IP_B, CFG_AP_IP_C, CFG_AP_IP_D);
static const IPAddress AP_SN(CFG_AP_SUBNET_A, CFG_AP_SUBNET_B, CFG_AP_SUBNET_C, CFG_AP_SUBNET_D);
static const IPAddress AP_DNS(CFG_AP_DNS_A, CFG_AP_DNS_B, CFG_AP_DNS_C, CFG_AP_DNS_D);

// UART link to ESP32‑S3 — config.h
static const uint32_t LINK_BAUD = CFG_LINK_BAUD;

// -----------------------------
// Pin map (BW16 기준) — 참고용, variant.h에 의해 고정 (AMB_Dx 사용)
// -----------------------------
static constexpr int RTL_LOG_TX_PIN = AMB_D0;  // PA7 (USB 디버그)
static constexpr int RTL_LOG_RX_PIN = AMB_D1;  // PA8
static constexpr int RTL_LINK_TX_PIN = AMB_D4; // PB1, RTL→S3
static constexpr int RTL_LINK_RX_PIN = AMB_D5; // PB2, S3→RTL
static constexpr int RTL_SPI_SS_PIN_DOC   = AMB_D9;  // PA15
static constexpr int RTL_SPI_SCLK_PIN_DOC = AMB_D10; // PA14
static constexpr int RTL_SPI_MISO_PIN_DOC = AMB_D11; // PA13
static constexpr int RTL_SPI_MOSI_PIN_DOC = AMB_D12; // PA12

// -----------------------------
// SPI link — 설정은 config.h
// -----------------------------
static constexpr int SPI_SS_PIN = CFG_SPI_SS_PIN;
static constexpr uint32_t SPI_HZ = CFG_SPI_HZ;
static constexpr size_t SPI_BLOCK_BYTES = CFG_SPI_BLOCK_BYTES;

#define ENABLE_CAMERA_BRIDGE CFG_ENABLE_CAMERA_BRIDGE

// 설계 방침(사용자 요청):
// - RTL은 SPI로 "항상" 카메라 프레임을 당겨서 최신 프레임을 유지한다.
// - WS 카메라 클라이언트가 연결되어 있을 때만 네트워크로 송출한다.

// -----------------------------
// Helpers
// -----------------------------
static void buildSsidFromMac(char* out, size_t outLen) {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  // DCM-XXXXXXXX 스타일 유지 (마지막 4바이트)
  snprintf(out, outLen, "DCM-%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);
}

// MAC 기반 자동 채널 분산 (교실 다수 로봇 대응)
// MAC 마지막 바이트 % 채널풀 크기 → 인덱스로 채널 선택
// 30대 로봇 → 8채널에 ~4대씩 분산 → 같은 채널 간섭 1/8로 감소
static int pickChannelFromMac() {
#if CFG_AP_CHANNEL_AUTO
  static const int channelPool[] = CFG_AP_CHANNEL_POOL;
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  int idx = mac[5] % CFG_AP_CHANNEL_POOL_SIZE;
  int ch = channelPool[idx];
  RTL_PRINTF(Serial, "[RTL] Auto-channel: MAC[5]=0x%02X → pool[%d] = ch %d\n", mac[5], idx, ch);
  return ch;
#else
  return CFG_AP_CHANNEL_DEFAULT;
#endif
}

static void sendSafetyStopToS3() {
  // DeepcoMini v1.3 커맨드 규약 그대로
  Serial1.println("stop,100");
}

// =============================================================
// v0.1.5: 옵션 E — S3 패스스루 플래시 모드 (RTL 하이브리드)
//
// 동작 흐름:
//   1. USB에서 "@passthru" 명령 수신
//   2. RTL이 GPIO로 S3를 부트모드로 전환 (BOOT=LOW → EN 리셋)
//   3. USB↔UART 투명 브릿지 루프 진입
//      (PC의 esptool ↔ RTL USB ↔ RTL UART ↔ S3 부트로더)
//   4. 타임아웃(무통신 10초) 또는 플래시 완료 시 정상 모드 복귀
//   5. S3를 정상 부트로 리셋
// =============================================================
#if CFG_PASSTHRU_ENABLE

static volatile bool g_passthruActive = false;

// S3를 부트모드로 전환 (GPIO0=LOW 상태에서 EN 리셋)
static void enterS3BootMode() {
  pinMode(CFG_S3_BOOT_PIN, OUTPUT);
  pinMode(CFG_S3_EN_PIN, OUTPUT);

  // Step 1: GPIO0 = LOW (부트모드 선택)
  digitalWrite(CFG_S3_BOOT_PIN, LOW);
  delay(10);

  // Step 2: EN = LOW → HIGH (리셋)
  digitalWrite(CFG_S3_EN_PIN, LOW);
  delay(50);
  digitalWrite(CFG_S3_EN_PIN, HIGH);
  delay(50);

  Serial.println("[RTL] S3 entered BOOT mode (GPIO0=LOW, EN toggled)");
}

// S3를 정상 모드로 리셋 (GPIO0=HIGH 상태에서 EN 리셋)
static void resetS3Normal() {
  // Step 1: GPIO0 = HIGH (정상 부트 선택)
  digitalWrite(CFG_S3_BOOT_PIN, HIGH);
  delay(10);

  // Step 2: EN = LOW → HIGH (리셋)
  digitalWrite(CFG_S3_EN_PIN, LOW);
  delay(50);
  digitalWrite(CFG_S3_EN_PIN, HIGH);
  delay(100);

  // GPIO를 입력으로 전환 (S3가 자유롭게 사용하도록)
  pinMode(CFG_S3_BOOT_PIN, INPUT);
  pinMode(CFG_S3_EN_PIN, INPUT);

  Serial.println("[RTL] S3 reset to NORMAL mode");
}

// USB↔UART 투명 브릿지 루프
// esptool은 115200~921600 보드레이트를 사용할 수 있음
// 기본 115200으로 시작, esptool이 change_baud 시 UART도 변경 필요
// (현재는 고정 보드레이트, 향후 자동 감지 가능)
static void runPassthruBridge() {
  Serial.println("[RTL] === PASSTHRU MODE START ===");
  Serial.println("[RTL] USB <-> UART transparent bridge active");
  Serial.println("[RTL] Press Ctrl+C x3 rapidly to force-exit");
  RTL_PRINTF(Serial, "[RTL] Timeout: %d ms (no data = exit)\n", CFG_PASSTHRU_TIMEOUT_MS);

  g_passthruActive = true;

  // UART 보드레이트를 패스스루용으로 설정
  // (이미 CFG_LINK_BAUD와 같으면 재설정 불필요하지만 명시적으로)
  Serial1.end();
  Serial1.begin(CFG_PASSTHRU_BAUD);

  unsigned long lastDataTime = millis();
  uint8_t buf[256];

  // 탈출 시퀀스 감지: Ctrl+C (0x03) x 3회를 500ms 이내에 입력
  static const uint8_t ESC_BYTE = 0x03;  // Ctrl+C
  static const uint8_t ESC_COUNT = 3;
  static const uint32_t ESC_WINDOW_MS = 500;
  uint8_t escHits = 0;
  uint32_t escFirstMs = 0;

  while (g_passthruActive) {
    bool hadData = false;

    // USB → UART (PC esptool → S3 부트로더)
    int avail = Serial.available();
    if (avail > 0) {
      int toRead = (avail > (int)sizeof(buf)) ? (int)sizeof(buf) : avail;
      int n = Serial.readBytes(buf, toRead);
      if (n > 0) {
        // 탈출 시퀀스 검사 (Ctrl+C x3 연속)
        for (int i = 0; i < n; i++) {
          if (buf[i] == ESC_BYTE) {
            if (escHits == 0) escFirstMs = millis();
            escHits++;
            if (escHits >= ESC_COUNT && (millis() - escFirstMs) < ESC_WINDOW_MS) {
              Serial.println("\n[RTL] Ctrl+C x3 detected - force exiting passthru");
              g_passthruActive = false;
              break;
            }
          } else {
            escHits = 0; // 비 ESC 바이트 → 카운터 리셋
          }
        }
        if (!g_passthruActive) break;

        Serial1.write(buf, n);
        hadData = true;
      }
    }

    // UART → USB (S3 부트로더 → PC esptool)
    avail = Serial1.available();
    if (avail > 0) {
      int toRead = (avail > (int)sizeof(buf)) ? (int)sizeof(buf) : avail;
      int n = Serial1.readBytes(buf, toRead);
      if (n > 0) {
        Serial.write(buf, n);
        hadData = true;
      }
    }

    if (hadData) {
      lastDataTime = millis();
    }

    // 탈출 시퀀스 윈도우 만료 시 리셋
    if (escHits > 0 && (millis() - escFirstMs) >= ESC_WINDOW_MS) {
      escHits = 0;
    }

    // 타임아웃 체크
    if (millis() - lastDataTime > CFG_PASSTHRU_TIMEOUT_MS) {
      Serial.println("\n[RTL] Passthru timeout - no data, exiting");
      break;
    }

    // 최소 지연 (CPU 점유 방지)
    delay(1);
  }

  g_passthruActive = false;

  // UART를 원래 보드레이트로 복원
  Serial1.end();
  Serial1.begin(CFG_LINK_BAUD);

  Serial.println("[RTL] === PASSTHRU MODE END ===");
}

// 패스스루 전체 시퀀스 실행
static void startPassthruMode() {
  Serial.println("[RTL] ========================================");
  Serial.println("[RTL] S3 Flash Passthrough Mode (Option E)");
  Serial.println("[RTL] ========================================");
  Serial.println("[RTL] 1. Entering S3 boot mode...");
  enterS3BootMode();

  Serial.println("[RTL] 2. Starting USB<->UART bridge...");
  Serial.println("[RTL] >>> Use esptool / Arduino IDE to flash S3 now <<<");
  runPassthruBridge();

  Serial.println("[RTL] 3. Resetting S3 to normal mode...");
  resetS3Normal();

  Serial.println("[RTL] Passthru complete. Resuming normal operation.");
  Serial.println("[RTL] ========================================");
}

#endif // CFG_PASSTHRU_ENABLE

// -----------------------------
// WebSocket servers (Links2004/arduinoWebSockets)
// -----------------------------
WebSocketsServer wsControl(CFG_WS_CONTROL_PORT);
// volatile: taskWsUart에서 쓰기, taskSpiPull에서 읽기 가능
static volatile bool hasControlClient = false;
static volatile uint8_t g_controlClientNum = 0xFF;

#if ENABLE_CAMERA_BRIDGE
WebSocketsServer wsCamera(CFG_WS_CAMERA_PORT);
// volatile: taskWsUart에서 쓰기, taskSpiPull에서 읽기 가능
static volatile bool hasCameraClient = false;
static volatile uint8_t g_cameraClientNum = 0xFF;
#endif

static inline bool controlClientConnected() {
  return hasControlClient && g_controlClientNum != 0xFF && wsControl.clientIsConnected(g_controlClientNum);
}

#if ENABLE_CAMERA_BRIDGE
static inline bool cameraClientConnected() {
  return hasCameraClient && g_cameraClientNum != 0xFF && wsCamera.clientIsConnected(g_cameraClientNum);
}
#endif

// forward declaration (used by poll* functions below)
static void sendLedLinkStateToS3(bool force);

static void pollControlClient() {
  wsControl.loop();
  if (hasControlClient && !wsControl.clientIsConnected(g_controlClientNum)) {
    hasControlClient = false;
    g_controlClientNum = 0xFF;
    sendSafetyStopToS3();
    sendLedLinkStateToS3(true);
    return;
  }
  sendLedLinkStateToS3(false);
}

#if ENABLE_CAMERA_BRIDGE
static void pollCameraClient() {
  wsCamera.loop();
  if (hasCameraClient && !wsCamera.clientIsConnected(g_cameraClientNum)) {
    hasCameraClient = false;
    g_cameraClientNum = 0xFF;
    sendLedLinkStateToS3(true);
    return;
  }
  sendLedLinkStateToS3(false);
}
#endif

// -----------------------------
// LED link-state reporting to S3 (so S3 can reproduce v1.3 LED logic)
// - "@ws,0/1"  : control WS (/ws) connected
// - "@sta,N"   : number of stations associated to RTL SoftAP (real count via wifi_get_associated_client_list)
// -----------------------------
static uint8_t g_ledWs = 0;
static uint16_t g_ledStaCount = 0;
static uint32_t g_lastStaPollMs = 0;

static uint16_t getApStaCount() {
  // wifi_conf.h / wifi_structures.h:
  // - wifi_get_associated_client_list(void* buf, unsigned short len)
  // - rtw_maclist_t { unsigned int count; rtw_mac_t mac_list[1]; }
  // AP_STA_NUM is the max client count supported by driver/SDK.
  uint8_t buf[sizeof(rtw_maclist_t) + (AP_STA_NUM - 1) * sizeof(rtw_mac_t)] = {0};
  const int ret = wifi_get_associated_client_list(buf, (unsigned short)sizeof(buf));
  if (ret != RTW_SUCCESS) return 0;
  const rtw_maclist_t* ml = (const rtw_maclist_t*)buf;
  if (!ml) return 0;
  if (ml->count > AP_STA_NUM) return (uint16_t)AP_STA_NUM;
  return (uint16_t)ml->count;
}

static void sendLedLinkStateToS3(bool force) {
  // station count polling (1Hz)
  const uint32_t nowMs = millis();
  if (force || (nowMs - g_lastStaPollMs >= 1000)) {
    g_lastStaPollMs = nowMs;
    g_ledStaCount = getApStaCount();
  }

  // staCount==0이면 WS도 존재할 수 없으므로 즉시 0으로 강등(v1.3과 동일한 안전 로직)
  uint8_t wsNow = controlClientConnected() ? 1 : 0;
  if (g_ledStaCount == 0) wsNow = 0;

  if (force || wsNow != g_ledWs) {
    g_ledWs = wsNow;
    Serial1.print("@ws,");
    Serial1.println((int)g_ledWs);
  }
  static uint16_t lastSentSta = 0xFFFF;
  if (force || g_ledStaCount != lastSentSta) {
    lastSentSta = g_ledStaCount;
    Serial1.print("@sta,");
    Serial1.println((int)lastSentSta);
  }
}

// -----------------------------
// SPI framing (S3 -> RTL)
// -----------------------------
#pragma pack(push, 1)
struct DcmSpiHdr {
  char magic[4];        // "DCM2"
  uint8_t type;         // 0=idle, 1=jpeg
  uint8_t flags;        // bit0=start, bit1=end
  uint16_t seq;         // frame seq
  uint32_t total_len;   // JPEG total length
  uint32_t offset;      // offset of this payload in JPEG
  uint16_t payload_len; // bytes following header in this block
  uint16_t reserved;
};
#pragma pack(pop)

static constexpr uint8_t SPI_TYPE_IDLE = 0;
static constexpr uint8_t SPI_TYPE_JPEG = 1;
static constexpr uint8_t SPI_FLAG_START = 0x01;
static constexpr uint8_t SPI_FLAG_END   = 0x02;

static uint8_t g_spiRx[SPI_BLOCK_BYTES];

// 최신 프레임 캐시(WS 송출용). SPI task가 업데이트하고, WS task가 읽음.
static uint8_t g_frameBuf[CFG_FRAME_BUF_SIZE];
static volatile size_t  g_frameLen = 0;
static volatile uint16_t g_frameSeq = 0;

#if RTL_HAS_FREERTOS
static SemaphoreHandle_t g_frameMutex = nullptr;
#endif

static volatile bool g_wantStream = false;
static volatile uint32_t g_statBlocks = 0;
static volatile uint32_t g_statFrames = 0;
static volatile uint32_t g_statBytes = 0;
// v0.1.2: 에러 통계
static volatile uint32_t g_statSyncErrors = 0;   // 매직 불일치
static volatile uint32_t g_statSeqErrors = 0;    // 시퀀스/오프셋 불일치로 드랍
static volatile uint32_t g_statOversize = 0;     // 프레임 크기 초과로 드랍

// 통계 스냅샷: 매크로로 정의 (Arduino IDE 전처리기의 자동 프로토타입 생성이
// struct 정의보다 앞에 놓여 에러 발생하는 문제 회피 — hdrLooksOk과 동일 패턴)
// 사용법: SNAPSHOT_STATS(로컬변수prefix) → _블럭, _프레임 등 6개 로컬 변수 생성+리셋
#if RTL_HAS_FREERTOS
#define SNAPSHOT_AND_RESET_STATS(blk,frm,byt,syncE,seqE,ovs) \
  do { \
    taskENTER_CRITICAL(); \
    blk  = g_statBlocks;     g_statBlocks     = 0; \
    frm  = g_statFrames;     g_statFrames     = 0; \
    byt  = g_statBytes;      g_statBytes      = 0; \
    syncE = g_statSyncErrors; g_statSyncErrors = 0; \
    seqE = g_statSeqErrors;  g_statSeqErrors  = 0; \
    ovs  = g_statOversize;   g_statOversize   = 0; \
    taskEXIT_CRITICAL(); \
  } while(0)
#else
#define SNAPSHOT_AND_RESET_STATS(blk,frm,byt,syncE,seqE,ovs) \
  do { \
    blk  = g_statBlocks;     g_statBlocks     = 0; \
    frm  = g_statFrames;     g_statFrames     = 0; \
    byt  = g_statBytes;      g_statBytes      = 0; \
    syncE = g_statSyncErrors; g_statSyncErrors = 0; \
    seqE = g_statSeqErrors;  g_statSeqErrors  = 0; \
    ovs  = g_statOversize;   g_statOversize   = 0; \
  } while(0)
#endif

// 매크로로 정의 (Arduino IDE 전처리기의 자동 프로토타입 생성이
// struct 정의보다 앞에 놓여 에러 발생하는 문제 회피)
#define hdrLooksOk(h) \
  ((h).magic[0] == 'D' && (h).magic[1] == 'C' && (h).magic[2] == 'M' && (h).magic[3] == '2')

static bool spiReadBlock(uint8_t* dst, size_t len) {
  if (!dst || len == 0) return false;

  // AmebaD SPI API는 보드별로 구현이 조금 다를 수 있어,
  // "in-place transfer" 방식으로 최대 호환을 노린다.
  memset(dst, 0, len);

  // AmebaD SPI는 transfer(pin, void*, count, mode)를 제공하며 내부에서 write+read 스트림을 돌립니다.
  // byte 단위 루프보다 훨씬 빠릅니다.
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  SPI.transfer((byte)SPI_SS_PIN, dst, len, SPI_LAST);
  SPI.endTransaction();
  return true;
}

// v0.1.2: 재동기화 상수
static constexpr uint8_t SPI_MAX_SYNC_RETRIES = 5;   // 연속 매직 불일치 시 리셋까지 허용 횟수
static constexpr uint8_t SPI_MAX_SEQ_RETRIES  = 3;   // 시퀀스 깨짐 시 프레임 드랍 & 재시작 횟수

static bool pullOneJpegFrameOverSpi(uint32_t timeoutMs) {
  const uint32_t start = millis();

  bool haveHeader = false;
  uint16_t curSeq = 0;
  size_t expectedTotal = 0;
  size_t received = 0;
  uint8_t syncRetries = 0;
  uint8_t seqRetries = 0;

  while (millis() - start < timeoutMs) {
    if (!spiReadBlock(g_spiRx, SPI_BLOCK_BYTES)) {
      delay(1);
      continue;
    }
    g_statBlocks++;

    if (SPI_BLOCK_BYTES < sizeof(DcmSpiHdr)) continue;
    const DcmSpiHdr* hdr = (const DcmSpiHdr*)g_spiRx;

    if (!hdrLooksOk(*hdr)) {
      g_statSyncErrors++;
      syncRetries++;
      if (syncRetries >= SPI_MAX_SYNC_RETRIES) {
        // v0.1.2: 연속 동기 실패 시 CS 토글로 SPI 라인 리셋 시도
        digitalWrite(SPI_SS_PIN, HIGH);
        delay(5);
        digitalWrite(SPI_SS_PIN, LOW);
        delay(1);
        syncRetries = 0;
        haveHeader = false;
      }
      delay(1);
      continue;
    }
    syncRetries = 0; // 매직 OK → 리셋

    if (hdr->type == SPI_TYPE_IDLE) {
      delay(2);
      continue;
    }
    if (hdr->type != SPI_TYPE_JPEG) {
      continue;
    }

    if (!haveHeader) {
      if ((hdr->flags & SPI_FLAG_START) == 0 || hdr->offset != 0) {
        continue;
      }
      curSeq = hdr->seq;
      expectedTotal = hdr->total_len;
      received = 0;

      if (expectedTotal == 0 || expectedTotal > sizeof(g_frameBuf)) {
        g_statOversize++;
        haveHeader = false;
        continue;
      }
      haveHeader = true;
      seqRetries = 0;
    }

    // 같은 프레임인지 확인
    if (!haveHeader || hdr->seq != curSeq) {
      g_statSeqErrors++;
      seqRetries++;
      haveHeader = false;
      if (seqRetries >= SPI_MAX_SEQ_RETRIES) {
        // v0.1.2: 연속 시퀀스 오류 → 타임아웃 대기 대신 즉시 새 프레임 탐색
        seqRetries = 0;
      }
      continue;
    }
    if (hdr->offset != received) {
      g_statSeqErrors++;
      haveHeader = false;
      continue;
    }

    const size_t pl = hdr->payload_len;
    const size_t headerSz = sizeof(DcmSpiHdr);
    if (pl > SPI_BLOCK_BYTES - headerSz) { haveHeader = false; continue; }
    if (hdr->offset + pl > expectedTotal) { haveHeader = false; continue; }

    // mutex로 memcpy + 메타데이터 갱신을 원자적으로 보호
    // (WS 태스크의 sendBinary도 mutex 하에 g_frameBuf 읽으므로 레이스 방지)
#if RTL_HAS_FREERTOS
    if (RTL_USE_TASKS && g_frameMutex) xSemaphoreTake(g_frameMutex, portMAX_DELAY);
#endif
    memcpy(g_frameBuf + hdr->offset, g_spiRx + headerSz, pl);
    received += pl;
    g_statBytes += (uint32_t)pl;

    const bool isEnd = (hdr->flags & SPI_FLAG_END) != 0;
    bool frameComplete = false;
    if (isEnd && received == expectedTotal) {
      g_frameLen = expectedTotal;
      g_frameSeq = curSeq;
      frameComplete = true;
    }
#if RTL_HAS_FREERTOS
    if (RTL_USE_TASKS && g_frameMutex) xSemaphoreGive(g_frameMutex);
#endif
    if (frameComplete) {
      g_statFrames++;
      return true;
    }
  }

  return false;
}

// WebSocket 콜백: control 서버
static void onControlWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      const char* path = (payload != nullptr) ? (const char*)payload : "";
      // 기존 DeepCoConnector 규약: control는 /ws 경로
      if (strcmp(path, "/ws") != 0) {
        wsControl.disconnect(num);
        return;
      }

      // single-client 정책: 기존 연결 종료
      if (hasControlClient && g_controlClientNum != num && wsControl.clientIsConnected(g_controlClientNum)) {
        wsControl.disconnect(g_controlClientNum);
      }
      g_controlClientNum = num;
      hasControlClient = true;
      Serial.println("[RTL] control WS client connected");
      sendLedLinkStateToS3(true);
      // 안전: 연결되자마자 stop 한번
      sendSafetyStopToS3();
      break;
    }
    case WStype_DISCONNECTED: {
      if (num == g_controlClientNum) {
        hasControlClient = false;
        g_controlClientNum = 0xFF;
        sendSafetyStopToS3();
        sendLedLinkStateToS3(true);
      }
      break;
    }
    case WStype_TEXT: {
      // active control client의 텍스트만 S3로 전달
      if (num == g_controlClientNum && payload && length > 0) {
        Serial1.write(payload, length);
        Serial1.write('\n');
      }
      break;
    }
    default:
      break;
  }
}

#if ENABLE_CAMERA_BRIDGE
// WebSocket 콜백: camera 서버
static void onCameraWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  (void)length;
  switch (type) {
    case WStype_CONNECTED: {
      const char* path = (payload != nullptr) ? (const char*)payload : "";
      // camera는 "/" 또는 빈 경로를 허용
      if (!(strcmp(path, "/") == 0 || path[0] == '\0')) {
        wsCamera.disconnect(num);
        return;
      }

      if (hasCameraClient && g_cameraClientNum != num && wsCamera.clientIsConnected(g_cameraClientNum)) {
        wsCamera.disconnect(g_cameraClientNum);
      }
      g_cameraClientNum = num;
      hasCameraClient = true;
      Serial.println("[RTL] camera WS client connected");
      sendLedLinkStateToS3(true);
      break;
    }
    case WStype_DISCONNECTED: {
      if (num == g_cameraClientNum) {
        hasCameraClient = false;
        g_cameraClientNum = 0xFF;
        sendLedLinkStateToS3(true);
      }
      break;
    }
    default:
      break;
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[RTL] DeepCoRTL_Bridge boot");
  RTL_PRINTF(Serial, "[RTL] Build: %s %s\n", __DATE__, __TIME__);
  RTL_PRINTF(Serial, "[RTL] Compile detect: RTL_HAS_FREERTOS=%d, defaultTasks=%d\n",
                (int)RTL_HAS_FREERTOS, (int)RTL_USE_TASKS);
  RTL_PRINTF(Serial, "[RTL] Serial1 pins (fixed by variant): TX=%d(AMB_D4) RX=%d(AMB_D5)\n",
                RTL_LINK_TX_PIN, RTL_LINK_RX_PIN);
  RTL_PRINTF(Serial, "[RTL] SPI pins (fixed by variant): SS=%d(AMB_D9) SCLK=%d(AMB_D10) MISO=%d(AMB_D11) MOSI=%d(AMB_D12)\n",
                RTL_SPI_SS_PIN_DOC, RTL_SPI_SCLK_PIN_DOC, RTL_SPI_MISO_PIN_DOC, RTL_SPI_MOSI_PIN_DOC);

  // v0.1.5: S3 패스스루용 GPIO 초기화 (입력 모드 = S3가 자유롭게 사용)
#if CFG_PASSTHRU_ENABLE
  pinMode(CFG_S3_EN_PIN, INPUT);
  pinMode(CFG_S3_BOOT_PIN, INPUT);
  RTL_PRINTF(Serial, "[RTL] Passthru GPIO: S3_EN=%d(AMB_D8/PA26), S3_BOOT=%d(AMB_D7/PA25)\n",
                CFG_S3_EN_PIN, CFG_S3_BOOT_PIN);
#endif

  // UART to S3
  Serial1.begin(LINK_BAUD);

  // SPI master init (camera)
  SPI.begin();
  pinMode(SPI_SS_PIN, OUTPUT);
  digitalWrite(SPI_SS_PIN, HIGH);

#if RTL_HAS_FREERTOS
  if (RTL_USE_TASKS) {
    g_frameMutex = xSemaphoreCreateMutex();
    if (!g_frameMutex) {
      Serial.println("[RTL] WARN: frame mutex create failed, falling back to loop() mode");
      // 강제 폴백 (컴파일 타임 const라서 런타임 변경은 불가) -> mutex 없는 상태로도 동작은 하지만
      // 데이터 레이스 가능성이 있어, 아래에서는 mutex 존재 여부를 체크하고 사용합니다.
    }
  }
#endif

  // Wi-Fi AP start
  WiFi.disablePowerSave();
  WiFi.config(AP_IP, AP_DNS, AP_GW, AP_SN);  // AP 모드에도 적용됨(문서)

  char ssid[4 + 8 + 1] = {0};
  buildSsidFromMac(ssid, sizeof(ssid));

  // 자동 채널 분산: @set,channel 로 수동 설정하지 않은 경우 MAC 기반 자동 선택
  if (g_apChannel == CFG_AP_CHANNEL_DEFAULT) {
    g_apChannel = pickChannelFromMac();
  }

  RTL_PRINTF(Serial, "[RTL] AP SSID=%s, Channel=%d\n", ssid, g_apChannel);

  int status = WiFi.apbegin(ssid, g_apPassword, g_apChannel);
  if (status != WL_CONNECTED) {
    // AmebaD는 apbegin() 리턴이 status 형태(문서상 "status of AP")
    Serial.print("[RTL] apbegin() status = ");
    Serial.println(status);
  }

  Serial.print("[RTL] AP IP = ");
  Serial.println(WiFi.localIP());
  // Initial LED link state for S3
  sendLedLinkStateToS3(true);

  // WebSocket servers — config.h
  wsControl.begin();
  wsControl.onEvent(onControlWsEvent);
  RTL_PRINTF(Serial, "[RTL] WS control listening on :%d (/ws accepted)\n", CFG_WS_CONTROL_PORT);

#if ENABLE_CAMERA_BRIDGE
  wsCamera.begin();
  wsCamera.onEvent(onCameraWsEvent);
  RTL_PRINTF(Serial, "[RTL] WS camera listening on :%d (/ accepted)\n", CFG_WS_CAMERA_PORT);
#else
  Serial.println("[RTL] Camera bridge disabled (ENABLE_CAMERA_BRIDGE=0)");
#endif

#if RTL_HAS_FREERTOS
  if (RTL_USE_TASKS) {
    Serial.println("[RTL] Mode: RTOS tasks (will start on first loop)");
  } else {
    Serial.println("[RTL] Mode: loop() fallback (RTOS headers not found)");
  }
#else
  Serial.println("[RTL] Mode: loop() fallback (no RTOS headers)");
#endif
}

// -----------------------------
// WebSocket servers (Links2004/arduinoWebSockets)
// -----------------------------

// -----------------------------
// v0.1.1: USB Serial 설정 명령 처리
// @set,channel,149   → Wi-Fi 채널 변경 (재부팅 후 적용)
// @set,password,xxxx → 비밀번호 변경 (재부팅 후 적용)
// @reboot            → RTL 재부팅
// @info              → 현재 설정 출력
// -----------------------------
// volatile: pollUsbSerialCommands 콘텍스트에서 설정, @reboot 명령에서 읽기
static volatile bool g_needReboot = false;

// 토큰 기반 명령 파싱: 하드코딩 인덱스 제거, char* 직접 사용
static void handleSerialCommand(const char* cmd) {
  // @set,<key>,<value> — 쉼표 구분자로 토큰 분리
  if (strncmp(cmd, "@set,", 5) == 0) {
    const char* keyStart = cmd + 5;
    const char* sep = strchr(keyStart, ',');
    if (!sep) { Serial.println("[RTL] @set needs key,value"); return; }

    size_t keyLen = (size_t)(sep - keyStart);
    const char* val = sep + 1;

    if (keyLen == 7 && strncmp(keyStart, "channel", 7) == 0) {
      int ch = atoi(val);
      if (ch > 0 && ch < 200) {
        g_apChannel = ch;
        g_needReboot = true;
        RTL_PRINTF(Serial, "[RTL] Channel set to %d (reboot to apply: @reboot)\n", g_apChannel);
      } else {
        Serial.println("[RTL] Invalid channel");
      }
    } else if (keyLen == 8 && strncmp(keyStart, "password", 8) == 0) {
      size_t pwLen = strlen(val);
      if (pwLen >= 8 && pwLen < sizeof(g_apPassword)) {
        strncpy(g_apPassword, val, sizeof(g_apPassword) - 1);
        g_apPassword[sizeof(g_apPassword) - 1] = '\0';
        g_needReboot = true;
        RTL_PRINTF(Serial, "[RTL] Password set to '%s' (reboot to apply: @reboot)\n", g_apPassword);
      } else {
        Serial.println("[RTL] Password must be 8~31 chars");
      }
    } else {
      RTL_PRINTF(Serial, "[RTL] Unknown @set key: %.*s\n", (int)keyLen, keyStart);
    }
  } else if (strcmp(cmd, "@reboot") == 0) {
    Serial.println("[RTL] Rebooting...");
    delay(200);
    // AmebaD software reset
    NVIC_SystemReset();
  } else if (strcmp(cmd, "@diag") == 0) {
    // v0.1.3: 종합 진단 정보 출력 + WS 전달
    const uint32_t uptimeSec = millis() / 1000;
    const uint32_t uptimeMin = uptimeSec / 60;
    char diagBuf[512];
    snprintf(diagBuf, sizeof(diagBuf),
      "@diag,{"
      "\"uptime_sec\":%lu,"
      "\"ap_channel\":%d,"
      "\"sta_count\":%d,"
      "\"ws_ctrl\":%d,"
#if ENABLE_CAMERA_BRIDGE
      "\"ws_cam\":%d,"
      "\"frame_seq\":%u,"
      "\"frame_len\":%u,"
#endif
      "\"spi_hz\":%lu,"
      "\"buf_size\":%u"
      "}",
      (unsigned long)uptimeSec,
      g_apChannel,
      (int)g_ledStaCount,
      (int)hasControlClient,
#if ENABLE_CAMERA_BRIDGE
      (int)hasCameraClient,
      (unsigned)g_frameSeq,
      (unsigned)g_frameLen,
#endif
      (unsigned long)SPI_HZ,
      (unsigned)sizeof(g_frameBuf)
    );
    Serial.println(diagBuf);
    if (controlClientConnected()) {
      wsControl.sendTXT(g_controlClientNum, diagBuf);
    }
    RTL_PRINTF(Serial, "[RTL] Uptime: %lum %lus\n", (unsigned long)uptimeMin, (unsigned long)(uptimeSec % 60));
  } else if (strncmp(cmd, "@s3debug,", 9) == 0) {
    // v0.1.3: S3 USB 디버그 토글 명령 전달
    const char v = cmd[9] ? cmd[9] : '0';
    RTL_PRINTF(Serial1, "@debug,%c\n", v);
    RTL_PRINTF(Serial, "[RTL] Sent @debug,%c to S3\n", v);
  } else if (strcmp(cmd, "@info") == 0) {
    {
      IPAddress ip = WiFi.localIP();
      char ipStr[16];
      snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
      RTL_PRINTF(Serial, "[RTL] Channel=%d (%s), Password='%s', IP=%s, SPI_HZ=%lu\n",
                    g_apChannel,
                    CFG_AP_CHANNEL_AUTO ? "auto/MAC" : "fixed",
                    g_apPassword, ipStr, (unsigned long)SPI_HZ);
    }
    RTL_PRINTF(Serial, "[RTL] WS ctrl=%d, cam=%d, staCount=%d\n",
                  (int)hasControlClient,
#if ENABLE_CAMERA_BRIDGE
                  (int)hasCameraClient,
#else
                  0,
#endif
                  (int)g_ledStaCount);
#if CFG_PASSTHRU_ENABLE
    RTL_PRINTF(Serial, "[RTL] Passthru: enabled (S3_EN=%d, S3_BOOT=%d)\n",
                  CFG_S3_EN_PIN, CFG_S3_BOOT_PIN);
#else
    Serial.println("[RTL] Passthru: disabled");
#endif
  }
#if CFG_PASSTHRU_ENABLE
  else if (strcmp(cmd, "@passthru") == 0) {
    // v0.1.5: S3 패스스루 플래시 모드 진입
    startPassthruMode();
  }
#endif
  else {
    RTL_PRINTF(Serial, "[RTL] Unknown command: %s\n", cmd);
    Serial.println("[RTL] Available: @set,channel,N / @set,password,X / @s3debug,0|1 / @diag / @reboot / @info"
#if CFG_PASSTHRU_ENABLE
                   " / @passthru"
#endif
    );
  }
}

// char[] 기반 USB 시리얼 명령 수신 (String 파편화 방지)
static void pollUsbSerialCommands() {
  static char usbBuf[128];
  static size_t usbPos = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (usbPos > 0 && usbBuf[0] == '@') {
        usbBuf[usbPos] = '\0';
        handleSerialCommand(usbBuf);
      }
      usbPos = 0;
      continue;
    }
    if (usbPos < sizeof(usbBuf) - 1) usbBuf[usbPos++] = ch;
  }
}

// S3 -> RTL 텍스트(선택): 디버그/상태를 WS로 되돌리고 싶으면 사용
// char[] 기반 S3→RTL 텍스트 수신 (String 파편화 방지)
static void pumpS3TextToWs() {
  static char s3Buf[256];
  static size_t s3Pos = 0;
  while (Serial1.available()) {
    char ch = (char)Serial1.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (s3Pos > 0) {
        s3Buf[s3Pos] = '\0';
        // 로그 전용 채널: S3가 "@log,"로 보내면 RTL USB 콘솔에 출력
        if (strncmp(s3Buf, "@log,", 5) == 0) {
          Serial.print("[S3] ");
          Serial.println(s3Buf + 5);
        }
        // v0.1.2: 에러 전파 — S3가 "@err,"로 보내면 USB콘솔 + WS로 중계
        else if (strncmp(s3Buf, "@err,", 5) == 0) {
          Serial.print("[S3][ERR] ");
          Serial.println(s3Buf + 5);
          if (controlClientConnected()) {
            wsControl.sendTXT(g_controlClientNum, s3Buf);  // PC에 "@err,코드,메시지" 그대로 전달
          }
        } else {
          // 그 외 텍스트는 (선택) WS control로 echo
          if (controlClientConnected()) {
            wsControl.sendTXT(g_controlClientNum, s3Buf);
          }
        }
      }
      s3Pos = 0;
      continue;
    }
    // binary 혼입 방지용 제한
    if ((uint8_t)ch >= 0x20 || ch == '\t') {
      if (s3Pos < sizeof(s3Buf) - 1) s3Buf[s3Pos++] = ch;
    }
  }
}

void loop() {
#if RTL_HAS_FREERTOS
  // 첫 loop()에서 task 생성(Arduino core 호환성 확보)
  ensureTasksStartedOnce();
#endif

  // RTOS task 모드: 실제 작업은 task에서 수행, loop()는 유휴
#if RTL_HAS_FREERTOS
  if (g_tasksEnabledRuntime) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return;
  }
#endif

  // loop() 폴백 모드(기존 동작 유지)
  pollControlClient();

#if ENABLE_CAMERA_BRIDGE
  pollCameraClient();
  g_wantStream = cameraClientConnected();

  // 카메라: SPI로 프레임을 "항상" 받아서 최신 프레임 유지
  static uint32_t lastPullMs = 0;
  const uint32_t nowMs = millis();
  const uint32_t pullIntervalMs = g_wantStream ? 0 : CFG_SPI_IDLE_PULL_MS;

  if (g_wantStream || (nowMs - lastPullMs >= pullIntervalMs)) {
    lastPullMs = nowMs;
    const uint16_t beforeSeq = g_frameSeq;
    if (pullOneJpegFrameOverSpi(g_wantStream ? 120 : 20)) {
      if (g_wantStream && beforeSeq != g_frameSeq) {
        // DeepCoConnector는 WS Binary message 1개 = JPEG 1장 전제를 갖고 있음
        (void)wsCamera.sendBIN(g_cameraClientNum, (const uint8_t*)g_frameBuf, (size_t)g_frameLen);
      }
      // wantStream=false면: 최신 프레임만 캐시해두고 송출은 안 함
    }
  }

  // 성능 로그: QVGA 30fps 튜닝용
  static uint32_t lastStatMs = 0;
  if (nowMs - lastStatMs >= CFG_STAT_INTERVAL_MS) {
    lastStatMs = nowMs;
    uint32_t sBlk, sFrm, sByt, sSyncE, sSeqE, sOvs;
    SNAPSHOT_AND_RESET_STATS(sBlk, sFrm, sByt, sSyncE, sSeqE, sOvs);
    const float secs = (float)CFG_STAT_INTERVAL_MS / 1000.0f;
    const float fps = (float)sFrm / secs;
    const float kbps = ((float)sByt / 1024.0f) / secs;
    RTL_PRINTF(Serial, "[RTL][SPI] fps=%.1f blocks=%lu KB/s=%.1f sync_err=%lu seq_err=%lu oversize=%lu\n",
                  fps, (unsigned long)sBlk, kbps,
                  (unsigned long)sSyncE, (unsigned long)sSeqE, (unsigned long)sOvs);
  }
#endif

  // S3 로그/상태 수신
  pumpS3TextToWs();

  // v0.1.1: USB Serial 설정 명령 처리
  pollUsbSerialCommands();
}

#if RTL_HAS_FREERTOS
// -----------------------------
// RTOS tasks (preferred)
// -----------------------------
static void taskWsUart(void* arg) {
  (void)arg;
  uint16_t lastSentSeq = 0;
  uint32_t lastStatMs = millis();
  uint32_t lastBeatMs = millis();

  for (;;) {
    pollControlClient();

#if ENABLE_CAMERA_BRIDGE
    pollCameraClient();
    g_wantStream = cameraClientConnected();

    // 최신 프레임이 갱신되면 송출
    if (g_wantStream) {
      uint16_t curSeq = 0;
      size_t curLen = 0;
      bool needSend = false;
      uint8_t* txBuf = nullptr;

      if (g_frameMutex) xSemaphoreTake(g_frameMutex, portMAX_DELAY);
      // --- Mutex Critical Section Start ---
      curSeq = g_frameSeq;
      curLen = (size_t)g_frameLen;

      if (curLen > 0 && curSeq != 0 && curSeq != lastSentSeq && cameraClientConnected()) {
        // [개선] 정적 RAM 점유를 피하기 위해 전송 직전에 동적 버퍼 할당
        // 할당 성공 시: Mutex 구간에서는 memcpy만 수행 후 즉시 해제
        txBuf = (uint8_t*)malloc(curLen);
        if (txBuf != nullptr) {
          memcpy(txBuf, g_frameBuf, curLen);
          lastSentSeq = curSeq;
          needSend = true;
        }
      }
      // --- Mutex Critical Section End ---
      if (g_frameMutex) xSemaphoreGive(g_frameMutex);

      // Mutex 해제 후, 복사된 데이터로 네트워크 전송 수행
      if (needSend) {
        (void)wsCamera.sendBIN(g_cameraClientNum, (const uint8_t*)txBuf, curLen);
        free(txBuf);
      }
    }

    // 성능 로그: SPI task가 stat을 올리고, 출력만 여기서
    const uint32_t nowMs = millis();
    if (nowMs - lastStatMs >= CFG_STAT_INTERVAL_MS) {
      lastStatMs = nowMs;
      uint32_t sBlk, sFrm, sByt, sSyncE, sSeqE, sOvs;
      SNAPSHOT_AND_RESET_STATS(sBlk, sFrm, sByt, sSyncE, sSeqE, sOvs);
      const float secs = (float)CFG_STAT_INTERVAL_MS / 1000.0f;
      const float fps = (float)sFrm / secs;
      const float kbps = ((float)sByt / 1024.0f) / secs;
      RTL_PRINTF(Serial, "[RTL][SPI] fps=%.1f blocks=%lu KB/s=%.1f sync_err=%lu seq_err=%lu oversize=%lu\n",
                    fps, (unsigned long)sBlk, kbps,
                    (unsigned long)sSyncE, (unsigned long)sSeqE, (unsigned long)sOvs);
    }
#endif

    pumpS3TextToWs();
    pollUsbSerialCommands();

    // 태스크 동작 확인용 하트비트(5초마다)
    const uint32_t nowBeat = millis();
    if (nowBeat - lastBeatMs >= 5000) {
      lastBeatMs = nowBeat;
      RTL_PRINTF(Serial, "[RTL][TASK] ws_uart alive (wantStream=%d)\n", (int)g_wantStream);
    }

    // WS/UART은 응답성이 중요: 짧게 양보
    vTaskDelay(1);
  }
}

static void taskSpiPull(void* arg) {
  (void)arg;
  uint32_t lastPullMs = 0;
  uint32_t lastBeatMs = millis();

  for (;;) {
#if ENABLE_CAMERA_BRIDGE
    const uint32_t nowMs = millis();
    const uint32_t pullIntervalMs = g_wantStream ? 0 : CFG_SPI_IDLE_PULL_MS;

    if (g_wantStream || (nowMs - lastPullMs >= pullIntervalMs)) {
      lastPullMs = nowMs;
      (void)pullOneJpegFrameOverSpi(g_wantStream ? 120 : 20);
    } else {
      vTaskDelay(5);
    }

    // 태스크 동작 확인용 하트비트(5초마다)
    const uint32_t nowBeat = millis();
    if (nowBeat - lastBeatMs >= 5000) {
      lastBeatMs = nowBeat;
      RTL_PRINTF(Serial, "[RTL][TASK] spi_pull alive (seq=%u len=%u)\n",
                    (unsigned)g_frameSeq, (unsigned)g_frameLen);
    }
#else
    vTaskDelay(50);
#endif
  }
}

// Arduino core에 따라 setup() 이후 자동으로 task를 만들 수 있게,
// 여기서는 "첫 loop()"에서 한번만 생성하는 방식으로 안전하게 동작시킴.
static void ensureTasksStartedOnce() {
  static bool started = false;
  if (started) return;
  started = true;

  if (!RTL_USE_TASKS) return;

  // WS/UART task는 상대적으로 높은 우선순위
  BaseType_t okA = xTaskCreate(taskWsUart, "ws_uart", 4096, NULL, 2, NULL);
  // SPI pull task는 낮은 우선순위 (제어/통신이 우선)
  BaseType_t okB = xTaskCreate(taskSpiPull, "spi_pull", 4096, NULL, 1, NULL);

  if (okA == pdPASS && okB == pdPASS) {
    g_tasksStartedRuntime = true;
    Serial.println("[RTL] Runtime: tasks started OK (ws_uart + spi_pull)");
  } else {
    g_tasksEnabledRuntime = false;
    RTL_PRINTF(Serial, "[RTL] Runtime: task create FAILED (ws_uart=%d spi_pull=%d) -> falling back to loop()\n",
                  (int)okA, (int)okB);
  }
}
#endif

